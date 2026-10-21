#include <cstdio>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "calibration.h"
#include "imu.h"
#include "esp_timer.h"
#include "mahony.h"
#include "position.h"
#include "normal.h"

static const char *TAG = "MAIN";

// app_main must have C linkage because this file is C++
extern "C" void app_main(void)
{
    // ---- NVS: shared infrastructure, initialized once here ----
    esp_err_t err = nvs_flash_init();
    // These two errors mean the partition is full or from an older IDF version
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Erase and retry. This wipes stored calibration, which is expected here
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // ---- Load stored calibration, or fall back to defaults ----
    switch (cal_load()) {
        case CAL_LOADED:
            break;
        case CAL_NOT_FOUND:
            printf("No calibration found, press 'c' to calibrate\n");
            break;
        case CAL_BAD_SIZE:
        case CAL_BAD_HEADER:
            printf("Old calibration format, press 'c' to recalibrate\n");
            break;
        case CAL_BAD_CRC:
            printf("Calibration corrupted, press 'c' to recalibrate\n");
            break;
        default:
            printf("Calibration load error\n");
            break;
    }

    // ---- IMU: imu_init() calls bus_init() internally, then configures the chip ----
    err = imu_init();
    if (err != ESP_OK) {
        // Don't hang or reboot: report the failure and park this task
        ESP_LOGE(TAG, "IMU init failed: %s", esp_err_to_name(err));
        while (1) vTaskDelay(portMAX_DELAY);
    }
    
    // ---- Mahony filter setup ----
    mahony_t filter;
    // Start with the big startup Kp so the estimate snaps to gravity quickly
    mahony_init(&filter, MAHONY_KP_STARTUP, MAHONY_KI);
    bool startup_done = false;

    // esp_timer_get_time() returns microseconds since boot
    int64_t start_us = esp_timer_get_time();
    int64_t last_us  = start_us;
    int print_count  = 0;

    // The filter wants rad/s, but our pipeline produces dps. Convert once here
    const float DEG2RAD = 0.01745329252f;

    // Reference gravity direction. Default = +Z, which is what accel calibration
    // leaves you with when the device sits flat. 'n' overwrites it.
    float g_ref[3] = {0.0f, 0.0f, 1.0f};
    float g_now[3] = {0.0f, 0.0f, 1.0f};   // latest gravity estimate, updated every loop

    // ---- Position tracker ----
    position_t pos;
    position_init(&pos);

    // ---- Main loop ----
    while (1) {
        // On the ESP32 console, getchar() returns EOF right away if nothing was typed
        int ch = getchar();
        if (ch == 'g' || ch == 'G') {
            esp_err_t cerr = cal_run_gyro();
            if (cerr != ESP_OK) printf("Gyro calibration did not complete: %s\n", esp_err_to_name(cerr));
            continue;
        }
        if (ch == 'a' || ch == 'A') {
            esp_err_t cerr = cal_run_accel();
            if (cerr != ESP_OK) printf("Accel calibration did not complete: %s\n", esp_err_to_name(cerr));
            continue;
        }
        if (ch == 'z' || ch == 'Z') {
            // Make "here" the new origin
            position_reset(&pos);
            printf("Position reset\n");
            continue;
        }
        if (ch == 'n' || ch == 'N') {
            // Capture the current gravity direction as the reference "normal"
            g_ref[0] = g_now[0]; g_ref[1] = g_now[1]; g_ref[2] = g_now[2];
            printf("Normal captured: %.3f, %.3f, %.3f\n", g_ref[0], g_ref[1], g_ref[2]);
            continue;
        }
        if (ch == 'm' || ch == 'M') {
            printf("Angle from reference: %.2f deg\n", angle_between_deg(g_now, g_ref));
            continue;
        }

        // Read one raw sample in physical units
        imu_sample_t raw;
        if (imu_read_sample(&raw) != ESP_OK) {
            ESP_LOGW(TAG, "Sample read failed");
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // Apply calibration (gyro bias removed, accel bias and scale corrected)
        imu_sample_t cor;
        cal_apply(&raw, &cor);

        // Measure the real time since the last sample. Our loop period isn't fixed
        // (read latency plus the delay), so assuming a constant would scale every angle wrong
        int64_t now_us = esp_timer_get_time();
        float dt = (float)(now_us - last_us) * 1e-6f;
        last_us = now_us;

        // Skip bad steps: the first iteration after a calibration routine (which blocks
        // for seconds) would otherwise inject one huge integration step
        if (dt <= 0.0f || dt > MAHONY_MAX_DT_S) continue;

        // After the startup window, drop to the normal gain
        if (!startup_done && (now_us - start_us) > (int64_t)MAHONY_STARTUP_MS * 1000) {
            filter.kp = MAHONY_KP;
            startup_done = true;
        }

        // One filter step: gyro in rad/s, accel in g, dt in seconds
        mahony_update(&filter,
                      cor.gx * DEG2RAD, cor.gy * DEG2RAD, cor.gz * DEG2RAD,
                      cor.ax, cor.ay, cor.az,
                      dt);

        // Feed the position tracker: it needs the filter's quaternion plus calibrated accel (g) and gyro (dps)
        float q[4];
        mahony_get_quaternion(&filter, q);
        gravity_from_quat(q, g_now);
        position_update(&pos, q,
                        cor.ax, cor.ay, cor.az,
                        cor.gx, cor.gy, cor.gz,
                        dt);

        if (++print_count >= MAHONY_PRINT_EVERY) {
            print_count = 0;
            float roll, pitch, yaw;
            mahony_get_euler_deg(&filter, &roll, &pitch, &yaw);
            // roll,pitch,yaw (deg), then x,y,z position (m), then 1 if held still
            printf("%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%d\n",
                   roll, pitch, yaw,
                   pos.p[0], pos.p[1], pos.p[2], pos.stationary ? 1 : 0);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}