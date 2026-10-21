#include <cstdint>
#include <cstddef>      // offsetof
#include <cstring>
#include <cmath>
#include <cstdio>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_crc.h"    // older IDF: "esp_rom_crc.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "calibration.h"
#include "imu.h"
#include "imu_config.h"

static const char *TAG = "CAL";

#define NVS_NAMESPACE "imu_cal"
#define NVS_KEY       "cal"

static cal_data_t s_cal; // Where NVS data is stored

static void fill_defaults(cal_data_t *c)
{
    memset(c, 0, sizeof(*c));       // Set all byte to zero
    c->magic   = CAL_MAGIC;
    c->version = CAL_VERSION;
    for (int i = 0; i < 3; i++) {  // Makes the default look valid data
        c->accel_scale[i] = 1.0f;
        c->mag_scale[i]   = 1.0f; // scales default to 1 because 0 will 0 every reading
    }
}

// 0 is starting value 
static uint32_t compute_crc(const cal_data_t *c)
{
    // read about the operator, first plain array of bytes, how many bytes before the crc   
    return esp_crc32_le(0, reinterpret_cast<const uint8_t *>(c), offsetof(cal_data_t, crc));
}

extern "C" void cal_set_defaults(void) // Turns of C++ name mangling so C files can link to this function
{
    fill_defaults(&s_cal);
}

extern "C" cal_data_t *cal_get(void) // Returns a pointer to the live struct so other code can read it
{
    return &s_cal;
}

extern "C" cal_status_t cal_load(void)
{
    fill_defaults(&s_cal);             /* safe fallback for every failure path */
    
    // Opens namespace read-only
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    // In read only mode, a namespace that doesn't exist returns this error
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "No calibration namespace (first boot)");
        return CAL_NOT_FOUND;
    }
    // Anything is unexpected
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open: %s", esp_err_to_name(err));
        return CAL_ERR;
    }

    /* Ask for the stored length first */
    size_t len = 0;
    err = nvs_get_blob(h, NVS_KEY, nullptr, &len);
    // The namespace exists, but the key doesn't. Close the handle so we don't leak it
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(h);
        return CAL_NOT_FOUND;
    }
    if (err != ESP_OK) {
        nvs_close(h);
        ESP_LOGE(TAG, "nvs_get_blob(len): %s", esp_err_to_name(err));
        return CAL_ERR;
    }
    if (len != sizeof(cal_data_t)) {
        nvs_close(h);
        ESP_LOGW(TAG, "Size mismatch: stored %u, expected %u",
                 (unsigned)len, (unsigned)sizeof(cal_data_t));
        return CAL_BAD_SIZE;
    }

    /* Read into a temporary, never straight into the live struct */
    cal_data_t tmp;
    err = nvs_get_blob(h, NVS_KEY, &tmp, &len);
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_get_blob: %s", esp_err_to_name(err));
        return CAL_ERR;
    }

    /* Validate: magic, version, CRC */
    if (tmp.magic != CAL_MAGIC || tmp.version != CAL_VERSION) {
        ESP_LOGW(TAG, "Bad header (magic 0x%08lX, version %u)",
                 (unsigned long)tmp.magic, (unsigned)tmp.version);
        return CAL_BAD_HEADER;
    }
    if (tmp.crc != compute_crc(&tmp)) {
        ESP_LOGW(TAG, "CRC mismatch");
        return CAL_BAD_CRC;
    }

    s_cal = tmp;
    ESP_LOGI(TAG, "Calibration loaded (flags=0x%02X)", (unsigned)s_cal.flags);
    return CAL_LOADED;
}

extern "C" esp_err_t cal_save(void)
{
    cal_data_t out = s_cal;
    out.magic   = CAL_MAGIC;
    out.version = CAL_VERSION;
    out.crc     = compute_crc(&out);

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open(rw): %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_blob(h, NVS_KEY, &out, sizeof(out));
    if (err == ESP_OK) {
        err = nvs_commit(h);           /* without this, the write may not persist */
    }
    nvs_close(h);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Save failed: %s", esp_err_to_name(err));
    } else {
        s_cal = out;
        ESP_LOGI(TAG, "Calibration saved");
    }
    return err;
}

// Blocks until the user types a character over serial, and returns it
static int wait_for_key(void)
{
    // Throw away any stray characters already waiting in the input buffer
    while (getchar() != EOF) {}

    // getchar() returns EOF immediately when nothing has been typed, so poll
    int ch;
    while ((ch = getchar()) == EOF) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return ch;
}

extern "C" esp_err_t cal_run_gyro(void)
{
    // Tell the user what to do, then wait so they control when we start
    printf("\n=== Gyro calibration ===\n");
    printf("Place the board on a stable surface and keep it completely still.\n");
    printf("Press any key to start...\n");
    wait_for_key();

    // Pressing a key can jostle the board, so let it settle first
    vTaskDelay(pdMS_TO_TICKS(CAL_GYRO_SETTLE_MS));
    printf("Sampling, do not touch the board...\n");

    // double accumulators: summing hundreds of floats in float loses precision
    double sum[3]   = {0.0, 0.0, 0.0};   // running sum of each axis
    double sumsq[3] = {0.0, 0.0, 0.0};   // running sum of squares, for variance
    int good  = 0;                       // samples accepted so far
    int fails = 0;                       // failed reads so far

    // Keep going until we have enough good samples
    while (good < CAL_GYRO_SAMPLES) {
        imu_sample_t s;

        // Skip a failed read, but give up if the sensor keeps failing
        if (imu_read_sample(&s) != ESP_OK) {
            if (++fails > CAL_MAX_READ_FAILS) {
                ESP_LOGE(TAG, "Too many read failures (%d), aborting", fails);
                return ESP_FAIL;
            }
            continue;
        }

        // Gather the three gyro axes so we can handle them in a loop
        const double g[3] = { s.gx, s.gy, s.gz };
        for (int i = 0; i < 3; i++) {
            sum[i]   += g[i];
            sumsq[i] += g[i] * g[i];
        }
        good++;
        // No vTaskDelay needed here: imu_read_sample() already blocks ~12-24 ms
        // on the latch delay, which also yields the CPU to other tasks
    }

    // Compute the mean and standard deviation for each axis
    double mean[3];
    double stddev[3];
    for (int i = 0; i < 3; i++) {
        mean[i] = sum[i] / good;
        // variance = mean of squares minus square of mean
        double var = sumsq[i] / good - mean[i] * mean[i];
        // Rounding can push a tiny variance slightly negative, which would break sqrt
        if (var < 0.0) var = 0.0;
        stddev[i] = sqrt(var);
    }

    printf("Mean   (dps): %.4f  %.4f  %.4f\n", mean[0], mean[1], mean[2]);
    printf("StdDev (dps): %.4f  %.4f  %.4f\n", stddev[0], stddev[1], stddev[2]);

    // If any axis is noisy, the board moved. Reject without touching the stored data
    for (int i = 0; i < 3; i++) {
        if (stddev[i] > CAL_GYRO_MAX_STDDEV_DPS) {
            printf("Board moved during sampling (axis %d). Calibration rejected, try again.\n", i);
            return ESP_ERR_INVALID_STATE;
        }
    }

    // The check passed, so it's now safe to modify the live struct.
    // Keep a backup in case the save to flash fails
    cal_data_t *c = cal_get();
    const cal_data_t backup = *c;

    for (int i = 0; i < 3; i++) {
        c->gyro_bias[i] = (float)mean[i];
    }
    // Mark the gyro as calibrated
    c->flags |= CAL_FLAG_GYRO;

    // Write to flash. If that fails, restore the old values so RAM matches flash
    esp_err_t err = cal_save();
    if (err != ESP_OK) {
        *c = backup;
        printf("Failed to save calibration: %s\n", esp_err_to_name(err));
        return err;
    }

    printf("Gyro bias saved: %.4f  %.4f  %.4f dps\n",
           c->gyro_bias[0], c->gyro_bias[1], c->gyro_bias[2]);
    return ESP_OK;
}

extern "C" void cal_apply(const imu_sample_t *in, imu_sample_t *out)
{
    // Copy every input into locals first. This makes in-place use safe:
    // after this point we never read from `in` again, so writing to `out`
    // can't clobber a value we still need
    const float g[3] = { in->gx, in->gy, in->gz };
    const float a[3] = { in->ax, in->ay, in->az };
    const float m[3] = { in->mx, in->my, in->mz };
    const bool  mag_ok = in->mag_valid;

    // Reads s_cal directly (no cal_get()) since we live in the same file.
    // NOTE: if calibration ever runs in a different task than the sample loop,
    // this needs a mutex or an atomic swap of a whole struct copy

    // Gyro: just subtract the bias
    out->gx = g[0] - s_cal.gyro_bias[0];
    out->gy = g[1] - s_cal.gyro_bias[1];
    out->gz = g[2] - s_cal.gyro_bias[2];

    // Accel: subtract the bias FIRST, then scale. The order matters whenever scale != 1
    out->ax = (a[0] - s_cal.accel_bias[0]) * s_cal.accel_scale[0];
    out->ay = (a[1] - s_cal.accel_bias[1]) * s_cal.accel_scale[1];
    out->az = (a[2] - s_cal.accel_bias[2]) * s_cal.accel_scale[2];

    // Mag: only correct it if the read actually succeeded. A failed read leaves
    // zeros, and applying a bias to zeros would fake a plausible-looking value
    if (mag_ok) {
        out->mx = (m[0] - s_cal.mag_bias[0]) * s_cal.mag_scale[0];
        out->my = (m[1] - s_cal.mag_bias[1]) * s_cal.mag_scale[1];
        out->mz = (m[2] - s_cal.mag_bias[2]) * s_cal.mag_scale[2];
    } else {
        // Pass the (empty) values straight through
        out->mx = m[0];
        out->my = m[1];
        out->mz = m[2];
    }

    // Carry the validity flag over, or `out` would hold garbage if it's a fresh struct
    out->mag_valid = mag_ok;
}

// Collects n accel samples and computes the mean and standard deviation per axis.
// Returns ESP_FAIL if the sensor keeps failing to read
static esp_err_t collect_accel_stats(int n, double mean[3], double stddev[3])
{
    // double accumulators: summing hundreds of floats in float loses precision
    double sum[3]   = {0.0, 0.0, 0.0};
    double sumsq[3] = {0.0, 0.0, 0.0};
    int good  = 0;
    int fails = 0;

    while (good < n) {
        imu_sample_t s;

        // Skip a failed read, but give up if the sensor keeps failing
        if (imu_read_sample(&s) != ESP_OK) {
            if (++fails > CAL_MAX_READ_FAILS) {
                ESP_LOGE(TAG, "Too many read failures (%d), aborting", fails);
                return ESP_FAIL;
            }
            // A failed transmit returns before the latch delay, so pause here
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Only the accel fields matter here; the gyro readings are ignored
        const double a[3] = { s.ax, s.ay, s.az };
        for (int i = 0; i < 3; i++) {
            sum[i]   += a[i];
            sumsq[i] += a[i] * a[i];
        }
        good++;
    }

    for (int i = 0; i < 3; i++) {
        mean[i] = sum[i] / good;
        // variance = mean of squares minus square of mean
        double var = sumsq[i] / good - mean[i] * mean[i];
        // Rounding can push a tiny variance slightly negative, which would break sqrt
        if (var < 0.0) var = 0.0;
        stddev[i] = sqrt(var);
    }
    return ESP_OK;
}

// One pose: which axis points up or down, and in which direction
typedef struct {
    const char *name;   // shown to the user
    int axis;           // 0=X, 1=Y, 2=Z
    int sign;           // +1 = axis points up, -1 = axis points down
} accel_pose_t;

static const accel_pose_t ACCEL_POSES[6] = {
    { "+Z up (chip face up, flat on the table)", 2, +1 },
    { "-Z up (flipped upside down)",             2, -1 },
    { "+X up (stand the board on its X+ side)",  0, +1 },
    { "-X up (stand the board on its X- side)",  0, -1 },
    { "+Y up (stand the board on its Y+ side)",  1, +1 },
    { "-Y up (stand the board on its Y- side)",  1, -1 },
};

extern "C" esp_err_t cal_run_accel(void)
{
    printf("\n=== Accel calibration (6 positions) ===\n");
    printf("Hold the board still in each pose. Check the axis labels on the silkscreen.\n");
    printf("A box or book helps hold it square. Press 'q' at any prompt to abort.\n");

    // hi[i] = average reading of axis i pointing up, lo[i] = pointing down
    double hi[3] = {0.0, 0.0, 0.0};
    double lo[3] = {0.0, 0.0, 0.0};

    for (int p = 0; p < 6; p++) {
        const accel_pose_t *pose = &ACCEL_POSES[p];
        bool pose_ok = false;

        // Retry just this pose until it succeeds, so one bad pose doesn't cost the others
        while (!pose_ok) {
            printf("\nPose %d/6: %s\n", p + 1, pose->name);
            printf("Press any key when ready ('q' to abort)...\n");
            int ch = wait_for_key();
            if (ch == 'q' || ch == 'Q') {
                printf("Accel calibration aborted. Nothing was changed.\n");
                return ESP_ERR_NOT_FINISHED;
            }

            // Pressing a key can jostle the board, so let it settle first
            vTaskDelay(pdMS_TO_TICKS(CAL_ACCEL_SETTLE_MS));
            printf("Sampling, do not touch the board...\n");

            double mean[3], stddev[3];
            if (collect_accel_stats(CAL_ACCEL_SAMPLES, mean, stddev) != ESP_OK) {
                return ESP_FAIL;
            }
            printf("Mean   (g): %.4f  %.4f  %.4f\n", mean[0], mean[1], mean[2]);
            printf("StdDev (g): %.4f  %.4f  %.4f\n", stddev[0], stddev[1], stddev[2]);

            // Check 1: was the board still?
            bool moved = false;
            for (int i = 0; i < 3; i++) {
                if (stddev[i] > CAL_ACCEL_MAX_STDDEV_G) moved = true;
            }
            if (moved) {
                printf("Board moved during sampling. Try this pose again.\n");
                continue;
            }

            // Check 2: is the right side up? The pose's axis must read about +-1 g with
            // the correct sign, and the other two axes about 0. This catches the user
            // holding the wrong face up, which would otherwise silently give garbage
            bool orient_ok = (mean[pose->axis] * pose->sign) >= CAL_ACCEL_ORIENT_MIN_G;
            for (int i = 0; i < 3; i++) {
                if (i != pose->axis && fabs(mean[i]) > CAL_ACCEL_ORIENT_MAX_OFF_G) {
                    orient_ok = false;
                }
            }
            if (!orient_ok) {
                printf("Wrong orientation for this pose. Check the axis labels and try again.\n");
                continue;
            }

            // Both checks passed: store the reading for this axis in its up/down slot
            if (pose->sign > 0) hi[pose->axis] = mean[pose->axis];
            else                lo[pose->axis] = mean[pose->axis];
            pose_ok = true;
        }
    }

    // Compute everything into locals first. The live struct isn't touched until
    // every result has been validated
    float bias[3], scale[3];
    for (int i = 0; i < 3; i++) {
        // If the sensor were perfect, hi and lo would cancel to 0; what's left is the offset
        double b = (hi[i] + lo[i]) / 2.0;
        // The true span between up and down is 2 g. The measured span is hi - lo
        double span = hi[i] - lo[i];
        // A span near zero means the poses were wrong; avoid dividing by it
        if (fabs(span) < 1e-6) {
            printf("Axis %d: up and down readings are identical. Calibration rejected.\n", i);
            return ESP_ERR_INVALID_STATE;
        }
        double sc = 2.0 / span;

        // Reject results that are implausible for a working sensor
        if (fabs(b) > CAL_ACCEL_MAX_BIAS_G ||
            sc < CAL_ACCEL_SCALE_MIN || sc > CAL_ACCEL_SCALE_MAX) {
            printf("Axis %d: bias %.4f g, scale %.4f out of range. Calibration rejected.\n",
                   i, b, sc);
            return ESP_ERR_INVALID_STATE;
        }
        bias[i]  = (float)b;
        scale[i] = (float)sc;
    }

    // Everything passed, so it's now safe to modify the live struct.
    // Keep a backup in case the save to flash fails
    cal_data_t *c = cal_get();
    const cal_data_t backup = *c;

    for (int i = 0; i < 3; i++) {
        c->accel_bias[i]  = bias[i];
        c->accel_scale[i] = scale[i];
    }
    // Mark the accel as calibrated
    c->flags |= CAL_FLAG_ACCEL;

    // Write to flash. If that fails, restore the old values so RAM matches flash
    esp_err_t err = cal_save();
    if (err != ESP_OK) {
        *c = backup;
        printf("Failed to save calibration: %s\n", esp_err_to_name(err));
        return err;
    }

    printf("\nAccel calibration saved:\n");
    printf("  Bias  (g): %.4f  %.4f  %.4f\n", c->accel_bias[0],  c->accel_bias[1],  c->accel_bias[2]);
    printf("  Scale    : %.4f  %.4f  %.4f\n", c->accel_scale[0], c->accel_scale[1], c->accel_scale[2]);
    return ESP_OK;
}