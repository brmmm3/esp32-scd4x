/* MIT License
*
* Copyright (c) 2022 ma-lwa-re
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <limits.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

#define FAHRENHEIT(celcius)         (((celcius * 9.0) / 5.0) + 32.0)
#define KELVIN(celcius)             (celcius + 273.15)
#define SCALE_CELCIUS               ('C')
#define SCALE_FAHRENHEIT            ('F')
#define SCALE_KELVIN                ('K')

#define TEMPERATURE_OFFSET          (4.0)
#define SENSOR_ALTITUDE             (0)

#define SCD4X_SENSOR_ADDR           (0x62)
#define SCD4X_READ_ERROR            (0xFFFF)
#define SCD4X_ADDR_SIZE             (0x02)

#define CRC8_POLYNOMIAL             (0x31)
#define CRC8_INIT                   (0xFF)

// Commands for state machine
#define SCD4X_CMD_FRC   0x362F

typedef struct scd4x_msb_lsb {
    uint8_t msb;
    uint8_t lsb;
} scd4x_msb_lsb_t;

typedef struct scd4x_sensor_value {
    scd4x_msb_lsb_t value;
    uint8_t crc;
} scd4x_sensor_value_t;

typedef struct __attribute__((packed)) scd4x_sensors_values {
    uint16_t co2;
    float temperature;
    float humidity;
} scd4x_values_t;

/**
 * Anonymous structure to driver settings.
 */
typedef struct scd4x_t {
    #if CONFIG_USE_I2C_MASTER_DRIVER
    // I2C master handle via port with configuration
    i2c_master_dev_handle_t dev_handle;
    // I2C master configuration
    i2c_device_config_t dev_cfg;
    // I2C master handle via port
    i2c_master_bus_handle_t bus_handle;
    #else
    // I2C port.
    i2c_port_t i2c_port;
    // Slave Address of sensor.
    uint8_t slave;
    #endif
    // Serial number of sensor
    uint64_t serial_number;
    float temperature_offset;   // °C
    uint16_t altitude;          // m
    uint16_t pressure;          // hPa
    scd4x_values_t values;
    uint8_t auto_adjust;
    bool enabled;
    uint8_t debug;
} scd4x_t;

extern uint8_t scd4x_st_machine_status;

scd4x_t *sensor_create_master(i2c_master_bus_handle_t bus_handle);

esp_err_t scd4x_device_create(scd4x_t *sensor_ptr);

esp_err_t scd4x_init_do(scd4x_t *sensor, bool low_power);

esp_err_t scd4x_init(scd4x_t** sensor_ptr, i2c_master_bus_handle_t bus_handle);

void scd4x_close(scd4x_t *sensor);

esp_err_t scd4x_device_init(scd4x_t *sensor);

esp_err_t scd4x_probe(scd4x_t *sensor);

esp_err_t scd4x_send_command(scd4x_t *sensor, uint8_t *command);

esp_err_t scd4x_read(scd4x_t *sensor, uint8_t *hex_code, uint8_t *measurements, uint8_t size);

esp_err_t scd4x_write(scd4x_t *sensor, uint8_t *hex_code, uint8_t *measurements, uint8_t size);

esp_err_t scd4x_send_command_and_fetch_result(scd4x_t *sensor, uint8_t *command, uint8_t *measurements, uint8_t size, uint16_t wait);

esp_err_t scd4x_start_periodic_measurement(scd4x_t *sensor);

esp_err_t scd4x_read_measurement(scd4x_t *sensor);

esp_err_t scd4x_stop_periodic_measurement(scd4x_t *sensor);

esp_err_t scd4x_set_temperature_offset(scd4x_t *sensor, float temperature);

float scd4x_get_temperature_offset(scd4x_t *sensor);

esp_err_t scd4x_set_sensor_altitude(scd4x_t *sensor, uint16_t altitude);

uint16_t scd4x_get_sensor_altitude(scd4x_t *sensor);

esp_err_t scd4x_set_ambient_pressure(scd4x_t *sensor, uint16_t pressure);

uint16_t scd4x_perform_forced_recalibration(scd4x_t *sensor, uint16_t co2_concentration);

esp_err_t scd4x_set_automatic_self_calibration_enabled(scd4x_t *sensor, bool asc_enabled);

bool scd4x_get_automatic_self_calibration_enabled(scd4x_t *sensor);

esp_err_t scd4x_start_low_power_periodic_measurement(scd4x_t *sensor);

bool scd4x_get_data_ready_status(scd4x_t *sensor);

esp_err_t scd4x_persist_settings(scd4x_t *sensor);

esp_err_t scd4x_get_serial_number(scd4x_t *sensor);

bool scd4x_perform_self_test(scd4x_t *sensor);

esp_err_t scd4x_perfom_factory_reset(scd4x_t *sensor);

esp_err_t scd4x_reinit(scd4x_t *sensor);

esp_err_t scd4x_measure_single_shot(scd4x_t *sensor);

esp_err_t scd4x_measure_single_shot_rht_only(scd4x_t *sensor);

esp_err_t scd4x_power_down(scd4x_t *sensor);

esp_err_t scd4x_wake_up(scd4x_t *sensor);

void scd4x_state_machine_cmd(uint16_t cmd, uint32_t arg);

int scd4x_state_machine(scd4x_t *sensor);

void scd4x_dump_values(scd4x_t *sensor, bool force);

#ifdef __cplusplus
};
#endif
