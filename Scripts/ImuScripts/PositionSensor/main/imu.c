#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"

#include "imu.h"
#include "imu_regs.h"
#include "imu_config.h"

static const char *TAG = "IMU";

// Handles are private to this file. Nothing outside the driver touches raw I2C.
static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_dev = NULL;

static bool  s_has_mag   = false;
static float s_acc_scale = 0.0f;   /* g per count   */
static float s_gyr_scale = 0.0f;   /* dps per count */

// Full-scale values matching each selector value. This is the single source
// of truth, so the selector and the scale can never disagree.
static const float ACC_RANGE_G[]    = { 2.0f, 4.0f, 8.0f, 16.0f };
static const float GYR_RANGE_DPS[]  = { 125.0f, 250.0f, 500.0f, 1000.0f, 2000.0f };

// Catch a bad selector at compile time instead of on the bench
_Static_assert(IMU_ACC_SEL >= 0 && IMU_ACC_SEL < 4, "IMU_ACC_SEL out of range");
_Static_assert(IMU_GYR_SEL >= 0 && IMU_GYR_SEL < 5, "IMU_GYR_SEL out of range");

/* ---------------- Low-level register access ---------------- */

static esp_err_t reg_read(uint16_t reg, uint8_t *out, size_t len)
{
    // Registers are 16-bit, sent low byte first
    uint8_t a[2] = { (uint8_t)(reg & 0xFF), (uint8_t)(reg >> 8) };

    // Step 1: tell the sensor which register we want
    esp_err_t err = i2c_master_transmit(s_dev, a, sizeof(a), IMU_XFER_TIMEOUT_MS);
    if (err != ESP_OK) return err;

    // Step 2: the firmware needs time to latch the data before we read it
    vTaskDelay(pdMS_TO_TICKS(IMU_LATCH_DELAY_MS));

    // Step 3: read the bytes back
    return i2c_master_receive(s_dev, out, len, IMU_XFER_TIMEOUT_MS);
}

static esp_err_t reg_read16(uint16_t reg, uint16_t *v)
{
    uint8_t b[2];
    esp_err_t err = reg_read(reg, b, 2);
    // Combine two bytes into one 16-bit value, low byte first (little-endian)
    if (err == ESP_OK) *v = (uint16_t)(b[0] | (b[1] << 8));
    return err;
}

static esp_err_t reg_write16(uint16_t reg, uint16_t val)
{
    // Message is: register (2 bytes) then value (2 bytes), all low byte first
    uint8_t b[4] = { (uint8_t)(reg & 0xFF), (uint8_t)(reg >> 8),
                     (uint8_t)(val & 0xFF), (uint8_t)(val >> 8) };
    return i2c_master_transmit(s_dev, b, sizeof(b), IMU_XFER_TIMEOUT_MS);
}

/* ---------------- Bus setup ---------------- */

static esp_err_t bus_init(void)
{
    // Describe the bus: which port, which pins, which clock source
    i2c_master_bus_config_t cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = IMU_I2C_PORT,
        .sda_io_num = IMU_SDA_IO,
        .scl_io_num = IMU_SCL_IO,
        // Ignore glitches shorter than 7 clock cycles
        .glitch_ignore_cnt = 7,
        // Weak internal pull-ups. Fine for short wires; use external 4.7k if flaky
        .flags.enable_internal_pullup = true,
    };

    // Create the bus. On failure, pass the error up instead of rebooting
    esp_err_t err = i2c_new_master_bus(&cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus: %s", esp_err_to_name(err));
        s_bus = NULL;
        return err;
    }

    // The address is set by a switch on the board, so try each possible one
    uint8_t found = 0;
    for (uint8_t a = IMU_ADDR_MIN; a <= IMU_ADDR_MAX; a++) {
        if (i2c_master_probe(s_bus, a, IMU_PROBE_TIMEOUT_MS) == ESP_OK) {
            found = a;
            break;
        }
    }

    // Nothing answered. Delete the bus so a retry doesn't fail with "already in use"
    if (!found) {
        ESP_LOGE(TAG, "No IMU at 0x%02X-0x%02X. Check wiring and I2C mode switch.",
                 IMU_ADDR_MIN, IMU_ADDR_MAX);
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
        return ESP_ERR_NOT_FOUND;
    }

    // Describe the device we found: 7-bit address at our chosen speed
    i2c_device_config_t dc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = found,
        .scl_speed_hz = IMU_I2C_FREQ_HZ,
    };

    // Attach the device to the bus. Clean up the bus if this fails too
    err = i2c_master_bus_add_device(s_bus, &dc, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "add_device: %s", esp_err_to_name(err));
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
        s_dev = NULL;
        return err;
    }

    ESP_LOGI(TAG, "IMU found at 0x%02X", found);
    return ESP_OK;
}

/* ---------------- Public API ---------------- */

esp_err_t imu_init(void)
{
    // Bring up the bus and find the sensor
    esp_err_t err = bus_init();
    if (err != ESP_OK) return err;

    // Read the identity registers
    uint16_t vid = 0, pid = 0, fw = 0;
    if ((err = reg_read16(R_VID, &vid)) != ESP_OK) return err;
    if ((err = reg_read16(R_PID, &pid)) != ESP_OK) return err;
    if ((err = reg_read16(R_FW,  &fw))  != ESP_OK) return err;
    ESP_LOGI(TAG, "VID=0x%04X PID=0x%04X FW=0x%04X", vid, pid, fw);

    // Wrong vendor ID means something else is at this address
    if (vid != IMU_VID_EXPECTED) {
        ESP_LOGE(TAG, "Unexpected VID (expected 0x%04X)", IMU_VID_EXPECTED);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // 9DOF and 10DOF models have a magnetometer; 6DOF does not
    s_has_mag = (pid >= IMU_PID_9DOF);
    ESP_LOGI(TAG, "Model: %s",
             pid >= IMU_PID_10DOF ? "10DOF" : s_has_mag ? "9DOF" : "6DOF (no magnetometer)");

    // Put the sensor in normal mode, then pause so it can settle
    if ((err = reg_write16(R_MODE, IMU_MODE_NORMAL)) != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(IMU_SETTLE_DELAY_MS));

    // Set the accelerometer range
    if ((err = reg_write16(R_ACC_RANGE, IMU_ACC_SEL)) != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(IMU_SETTLE_DELAY_MS));

    // Set the gyroscope range
    if ((err = reg_write16(R_GYR_RANGE, IMU_GYR_SEL)) != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(IMU_SETTLE_DELAY_MS));

#if IMU_ENABLE_EVENTS
    // Map the two event channels (INT1, INT2) to the events chosen in the config
    if ((err = reg_write16(R_INT1, IMU_INT1_EVENT)) != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(IMU_SETTLE_DELAY_MS));
    if ((err = reg_write16(R_INT2, IMU_INT2_EVENT)) != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(IMU_SETTLE_DELAY_MS));
#endif

    // Raw counts span -32768..32767, so scale = full-scale range / 32768
    s_acc_scale = ACC_RANGE_G[IMU_ACC_SEL]   / 32768.0f;
    s_gyr_scale = GYR_RANGE_DPS[IMU_GYR_SEL] / 32768.0f;

    return ESP_OK;
}

bool imu_has_mag(void)
{
    return s_has_mag;
}

esp_err_t imu_read_sample(imu_sample_t *out)
{
    if (out == NULL) return ESP_ERR_INVALID_ARG;

    // Read 12 bytes: accel X/Y/Z then gyro X/Y/Z, each a 16-bit value
    uint8_t d[12];
    esp_err_t err = reg_read(R_ACC_X, d, sizeof(d));
    if (err != ESP_OK) return err;

    // Rebuild each signed 16-bit value from its two bytes (low byte first)
    int16_t raw[6];
    for (int i = 0; i < 6; i++) {
        raw[i] = (int16_t)(d[2 * i] | (d[2 * i + 1] << 8));
    }

    // Convert raw counts to physical units
    out->ax = raw[0] * s_acc_scale;
    out->ay = raw[1] * s_acc_scale;
    out->az = raw[2] * s_acc_scale;
    out->gx = raw[3] * s_gyr_scale;
    out->gy = raw[4] * s_gyr_scale;
    out->gz = raw[5] * s_gyr_scale;

    // Start with mag marked invalid; only set it if the read below succeeds
    out->mx = out->my = out->mz = 0.0f;
    out->mag_valid = false;

    if (s_has_mag) {
        // 12 bytes: three 32-bit values, one per axis
        uint8_t m[12];
        if (reg_read(R_MAG_X_LO, m, sizeof(m)) == ESP_OK) {
            int32_t v[3];
            for (int i = 0; i < 3; i++) {
                // Assemble four bytes into one signed 32-bit value, low byte first
                v[i] = (int32_t)((uint32_t)m[4 * i]            |
                                ((uint32_t)m[4 * i + 1] << 8)  |
                                ((uint32_t)m[4 * i + 2] << 16) |
                                ((uint32_t)m[4 * i + 3] << 24));
            }
            // Each count is 0.01 uT
            out->mx = v[0] * 0.01f;
            out->my = v[1] * 0.01f;
            out->mz = v[2] * 0.01f;
            out->mag_valid = true;
        }
        // A failed mag read is not fatal: the accel/gyro data is still good
    }

    return ESP_OK;
}

esp_err_t imu_read_events(uint32_t *steps, uint16_t *tap, uint16_t *orient)
{
    if (!steps || !tap || !orient) return ESP_ERR_INVALID_ARG;

    // Step count is a 32-bit value spread over four bytes
    uint8_t s[4] = {0};
    esp_err_t err = reg_read(R_STEP_LO, s, sizeof(s));
    if (err != ESP_OK) return err;
    *steps = (uint32_t)s[0] | ((uint32_t)s[1] << 8) |
             ((uint32_t)s[2] << 16) | ((uint32_t)s[3] << 24);

    // Tap type: 1=single, 2=double, 3=triple
    if ((err = reg_read16(R_TAP, tap)) != ESP_OK) return err;

    // Orientation: high byte is orientation, low byte is face up/down
    return reg_read16(R_ORIENT, orient);
}