#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"

#define TAG "IMU"

#define I2C_SDA_IO   21
#define I2C_SCL_IO   22
#define I2C_FREQ_HZ  400000

/* DFRobot I2C-mode register map (16-bit word registers) */
#define R_VID        0x0000   /* expect 0x3343 */
#define R_PID        0x0001   /* 0x0006=6DOF, 0x0009=9DOF, 0x0010=10DOF */
#define R_FW         0x0002
#define R_MODE       0x0005   /* 0 sleep,1 low power,2 normal,3 high perf */
#define R_ACC_RANGE  0x0006   /* 0=2g 1=4g 2=8g 3=16g */
#define R_GYR_RANGE  0x0007   /* 0=125 1=250 2=500 3=1000 4=2000 dps */
#define R_INT1       0x000C
#define R_INT2       0x000D
#define R_ACC_X      0x0010   /* 12 bytes: accXYZ, gyrXYZ (int16 LE) */
#define R_MAG_X_LO   0x0016   /* 12 bytes: magXYZ (int32 LE, 0.01 uT) - 9/10DOF only */
#define R_STEP_LO    0x001E   /* 4 bytes: step count (uint32 LE) */
#define R_TAP        0x0020   /* 1=single 2=double 3=triple */
#define R_ORIENT     0x0021   /* high byte: orientation, low byte: face up/down */

/* Ranges (selector value and matching full-scale value) */
#define ACC_SEL      2
#define ACC_RANGE_G  8.0f
#define GYR_SEL      4
#define GYR_RANGE_DPS 2000.0f

/* Motion-event mapping. The board has 2 event channels (INT1, INT2).
 * 0=off 1=data ready 2=any motion 3=no motion 4=significant motion
 * 5=step counter 6=flat 7=orientation 8=tap 9=tilt
 * Step count, tap and orientation are readable over I2C; the rest only
 * toggle the INT pin. */
#define ENABLE_EVENTS 1
#define INT1_EVENT    5
#define INT2_EVENT    8

static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t dev;

static esp_err_t reg_read(uint16_t reg, uint8_t *out, size_t len)
{
    uint8_t a[2] = { (uint8_t)(reg & 0xFF), (uint8_t)(reg >> 8) };
    esp_err_t err = i2c_master_transmit(dev, a, sizeof(a), 100);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(12));   /* firmware needs ~10 ms to latch data */
    return i2c_master_receive(dev, out, len, 100);
}

static esp_err_t reg_read16(uint16_t reg, uint16_t *v)
{
    uint8_t b[2];
    esp_err_t err = reg_read(reg, b, 2);
    if (err == ESP_OK) *v = (uint16_t)(b[0] | (b[1] << 8));
    return err;
}

static esp_err_t reg_write16(uint16_t reg, uint16_t val)
{
    uint8_t b[4] = { (uint8_t)(reg & 0xFF), (uint8_t)(reg >> 8),
                     (uint8_t)(val & 0xFF), (uint8_t)(val >> 8) };
    return i2c_master_transmit(dev, b, sizeof(b), 100);
}

static void bus_init(void)
{
    i2c_master_bus_config_t cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &bus));

    uint8_t found = 0;
    for (uint8_t a = 0x4A; a <= 0x4D; a++) {   /* switch-selectable addresses */
        if (i2c_master_probe(bus, a, 50) == ESP_OK) { found = a; break; }
    }
    if (!found) {
        ESP_LOGE(TAG, "No IMU at 0x4A-0x4D. Check wiring and I2C mode switch.");
        while (1) vTaskDelay(portMAX_DELAY);
    }
    i2c_device_config_t dc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = found,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dc, &dev));
    ESP_LOGI(TAG, "IMU found at 0x%02X", found);
}

void app_main(void)
{
    bus_init();

    uint16_t vid = 0, pid = 0, fw = 0;
    ESP_ERROR_CHECK(reg_read16(R_VID, &vid));
    ESP_ERROR_CHECK(reg_read16(R_PID, &pid));
    ESP_ERROR_CHECK(reg_read16(R_FW, &fw));
    ESP_LOGI(TAG, "VID=0x%04X PID=0x%04X FW=0x%04X", vid, pid, fw);
    if (vid != 0x3343) {
        ESP_LOGE(TAG, "Unexpected VID (expected 0x3343)");
        while (1) vTaskDelay(portMAX_DELAY);
    }
    const bool has_mag = (pid >= 0x0009);
    ESP_LOGI(TAG, "Model: %s", pid >= 0x0010 ? "10DOF" : has_mag ? "9DOF" : "6DOF (no magnetometer)");

    ESP_ERROR_CHECK(reg_write16(R_MODE, 2));            /* normal mode */
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_ERROR_CHECK(reg_write16(R_ACC_RANGE, ACC_SEL));
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_ERROR_CHECK(reg_write16(R_GYR_RANGE, GYR_SEL));
    vTaskDelay(pdMS_TO_TICKS(20));
#if ENABLE_EVENTS
    ESP_ERROR_CHECK(reg_write16(R_INT1, INT1_EVENT));
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_ERROR_CHECK(reg_write16(R_INT2, INT2_EVENT));
    vTaskDelay(pdMS_TO_TICKS(20));
#endif

    const float acc_scale = ACC_RANGE_G / 32768.0f;
    const float gyr_scale = GYR_RANGE_DPS / 32768.0f;

    printf(has_mag
        ? "t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,mx_uT,my_uT,mz_uT,steps,tap,orient\n"
        : "t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,steps,tap,orient\n");

    while (1) {
        uint8_t d[12];
        if (reg_read(R_ACC_X, d, 12) != ESP_OK) {
            ESP_LOGW(TAG, "6-axis read failed");
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        int16_t raw[6];
        for (int i = 0; i < 6; i++) raw[i] = (int16_t)(d[2 * i] | (d[2 * i + 1] << 8));

        float mx = 0, my = 0, mz = 0;
        if (has_mag) {
            uint8_t m[12];
            if (reg_read(R_MAG_X_LO, m, 12) == ESP_OK) {
                int32_t v[3];
                for (int i = 0; i < 3; i++) {
                    v[i] = (int32_t)((uint32_t)m[4*i] | ((uint32_t)m[4*i+1] << 8) |
                                     ((uint32_t)m[4*i+2] << 16) | ((uint32_t)m[4*i+3] << 24));
                }
                mx = v[0] * 0.01f; my = v[1] * 0.01f; mz = v[2] * 0.01f;
            }
        }

        uint8_t s[4] = {0};
        uint16_t tap = 0, ori = 0;
        reg_read(R_STEP_LO, s, 4);
        reg_read16(R_TAP, &tap);
        reg_read16(R_ORIENT, &ori);
        uint32_t steps = (uint32_t)s[0] | ((uint32_t)s[1] << 8) |
                         ((uint32_t)s[2] << 16) | ((uint32_t)s[3] << 24);

        uint32_t t_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (has_mag) {
            printf("%lu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%lu,%u,0x%04X\n",
                   (unsigned long)t_ms,
                   raw[0]*acc_scale, raw[1]*acc_scale, raw[2]*acc_scale,
                   raw[3]*gyr_scale, raw[4]*gyr_scale, raw[5]*gyr_scale,
                   mx, my, mz, (unsigned long)steps, tap, ori);
        } else {
            printf("%lu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%lu,%u,0x%04X\n",
                   (unsigned long)t_ms,
                   raw[0]*acc_scale, raw[1]*acc_scale, raw[2]*acc_scale,
                   raw[3]*gyr_scale, raw[4]*gyr_scale, raw[5]*gyr_scale,
                   (unsigned long)steps, tap, ori);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}