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

#include "scd4x.h"
#include "util.h"
#include "time.h"
#include <string.h>
#include "math.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "SCD4X";

typedef struct measurements {
    scd4x_sensor_value_t co2;
    scd4x_sensor_value_t temperature;
    scd4x_sensor_value_t humidity;
} measurements_t;

// State machine status
uint8_t scd4x_st_machine_status = SCD4X_ST_IDLE;
static time_t st_machine_time = 0;
static uint16_t st_machine_cmd = 0;
static uint32_t st_machine_arg = 0;

static uint8_t start_periodic_measurement[]             = {0x21, 0xB1};
static uint8_t read_measurement[]                       = {0xEC, 0x05};
static uint8_t stop_periodic_measurement[]              = {0x3F, 0x86};
static uint8_t set_temperature_offset[]                 = {0x24, 0x1D};
static uint8_t get_temperature_offset[]                 = {0x23, 0x18};
static uint8_t set_sensor_altitude[]                    = {0x24, 0x27};
static uint8_t get_sensor_altitude[]                    = {0x23, 0x22};
static uint8_t set_ambient_pressure[]                   = {0xE0, 0x00};
static uint8_t perform_forced_recalibration[]           = {0x36, 0x2F};
static uint8_t set_automatic_self_calibration_enabled[] = {0x24, 0x16};
static uint8_t get_automatic_self_calibration_enabled[] = {0x23, 0x13};
static uint8_t start_low_power_periodic_measurement[]   = {0x21, 0xAC};
static uint8_t get_data_ready_status[]                  = {0xE4, 0xB8};
static uint8_t persist_settings[]                       = {0x36, 0x15};
static uint8_t get_serial_number[]                      = {0x36, 0x82};
static uint8_t perform_self_test[]                      = {0x36, 0x39};
static uint8_t perfom_factory_reset[]                   = {0x36, 0x32};
static uint8_t reinit[]                                 = {0x36, 0x46};
static uint8_t measure_single_shot[]                    = {0x21, 0x9D};
static uint8_t measure_single_shot_rht_only[]           = {0x21, 0x96};
static uint8_t power_down[]                             = {0x36, 0xE0};
static uint8_t wake_up[]                                = {0x36, 0xF6};


scd4x_t *scd4x_create_master(i2c_master_bus_handle_t bus_handle)
{
    scd4x_t *sensor = pvPortMalloc(sizeof(scd4x_t));
    memset(sensor, 0, sizeof(scd4x_t));

    if (sensor != NULL) {
        sensor->bus_handle = bus_handle;
        sensor->dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        sensor->temperature_offset = 0.0;
        sensor->altitude = 0.0;
        sensor->auto_adjust = 60;
    } else {
        ESP_LOGE(TAG, "Failed to allocate memory for scd4x.");
        scd4x_close(sensor);
        return NULL;
    }
    return sensor;
}

/**
 * @param scd4x Driver Structure.
 */
esp_err_t scd4x_device_create(scd4x_t *sensor)
{
    ESP_LOGI(TAG, "device_create for SCD40/SCD41 sensors on ADDR %X", SCD4X_SENSOR_ADDR);
    sensor->dev_cfg.device_address = SCD4X_SENSOR_ADDR;
    sensor->dev_cfg.scl_speed_hz = CONFIG_SCD4X_I2C_CLK_SPEED_HZ;
    sensor->dev_cfg.flags.disable_ack_check = true;
    // Add device to the I2C bus
    esp_err_t err = i2c_master_bus_add_device(sensor->bus_handle, &sensor->dev_cfg, &sensor->dev_handle);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "device_create success on %X", SCD4X_SENSOR_ADDR);
        return err;
    } else {
        ESP_LOGE(TAG, "device_create error on %X", SCD4X_SENSOR_ADDR);
        return err;
    }
}

esp_err_t scd4x_init_do(scd4x_t *sensor, bool low_power)
{
    esp_err_t err;

    if ((err = scd4x_power_down(sensor)) != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));
    if ((err = scd4x_wake_up(sensor)) != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));
    if ((err = scd4x_device_init(sensor)) != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));
    // Disable auto calibration
    if ((err = scd4x_set_automatic_self_calibration_enabled(sensor, false)) != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));
    //if ((err = scd4x_measure_single_shot(sensor)) != ESP_OK) return err;
    //vTaskDelay(pdMS_TO_TICKS(10));
    sensor->enabled = true;
    if (low_power) {
        return scd4x_start_low_power_periodic_measurement(sensor);
    }
    return scd4x_start_periodic_measurement(sensor);
}

esp_err_t scd4x_init(scd4x_t **sensor_ptr, i2c_master_bus_handle_t bus_handle)
{
    scd4x_t *sensor = NULL;
    esp_err_t err;

    ESP_LOGI(TAG, "Initialize SCD4x");
    sensor = scd4x_create_master(bus_handle);
    if (sensor == NULL) { 
        ESP_LOGE(TAG, "Could not create scd4x driver.");
        return ESP_FAIL;
    }
    if ((err = scd4x_device_create(sensor)) != ESP_OK) return err;
    for (int i = 0; i < 5; i++) {
        if ((err = scd4x_init_do(sensor, false)) == ESP_OK) {
            *sensor_ptr = sensor;
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return err;
}

void scd4x_close(scd4x_t *sensor)
{
    #if CONFIG_USE_I2C_MASTER_DRIVER
    if (sensor != NULL && sensor->dev_handle != NULL)
        i2c_master_bus_rm_device(sensor->dev_handle);
    #endif
    vPortFree(sensor);
}

esp_err_t scd4x_device_init(scd4x_t *sensor)
{
    esp_err_t err;

    if (sensor == NULL) return ESP_ERR_INVALID_ARG;
    err = scd4x_probe(sensor) || scd4x_reinit(sensor);
    if (err != ESP_OK) return err;
    // Give the sensor 10 ms delay to reset.
    vTaskDelay(pdMS_TO_TICKS(10));
    err = scd4x_get_serial_number(sensor);
    ESP_LOGI(TAG, "Sensor serial number (err=%d) 0x%012llX", err, sensor->serial_number);
    vTaskDelay(pdMS_TO_TICKS(10));
    sensor->temperature_offset = scd4x_get_temperature_offset(sensor);
    vTaskDelay(pdMS_TO_TICKS(10));
    sensor->altitude = scd4x_get_sensor_altitude(sensor);
    if (sensor->temperature_offset != SCD4X_READ_ERROR && sensor->altitude != SCD4X_READ_ERROR) {
        if (sensor->temperature_offset != TEMPERATURE_OFFSET) {
            ESP_LOGW(TAG, "Temperature offset calibration from %.1f °C to %.1f °C",
                     sensor->temperature_offset, TEMPERATURE_OFFSET);
            vTaskDelay(pdMS_TO_TICKS(10));
            ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_set_temperature_offset(sensor, TEMPERATURE_OFFSET));
            vTaskDelay(pdMS_TO_TICKS(10));
            ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_persist_settings(sensor));
            vTaskDelay(pdMS_TO_TICKS(10));
            sensor->temperature_offset = scd4x_get_temperature_offset(sensor);
        }
        if (sensor->altitude != SENSOR_ALTITUDE) {
            ESP_LOGW(TAG, "Sensor altitude calibration from %d m to %d m",
                     sensor->altitude, SENSOR_ALTITUDE);
            vTaskDelay(pdMS_TO_TICKS(10));
            ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_set_sensor_altitude(sensor, SENSOR_ALTITUDE));
            vTaskDelay(pdMS_TO_TICKS(10));
            ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_persist_settings(sensor));
            vTaskDelay(pdMS_TO_TICKS(10));
            sensor->altitude = scd4x_get_sensor_altitude(sensor);
        }
        ESP_LOGI(TAG, "Temperature offset %.1f °C - Sensor altitude %d m",
                 sensor->temperature_offset, sensor->altitude);
    } else {
        ESP_LOGE(TAG, "Sensor offset/altitude read error!");
    }
    return ESP_OK;
}

esp_err_t scd4x_probe(scd4x_t *sensor)
{
    esp_err_t err = ESP_OK;
    int i;

    //ESP_LOGI(TAG, "Probing for SCD40/SCD41 sensor on I2C %X", sensor->dev_cfg.device_address);
    for (i = 0; i < 5; i++) {
        err = i2c_master_probe(sensor->bus_handle, sensor->dev_cfg.device_address, CONFIG_SCD4X_TIMEOUT);
        if (err == ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Probing for SCD40/SCD41 success after %d tries", i);
    } else {
        ESP_LOGE(TAG, "Probing for SCD40/SCD41 failed");
    }
    return err;
}

/*
* For the send command sequences, after writing the address and/or data to the sensor
* and sending the ACK bit, the sensor needs the execution time to respond to the I2C read header with an ACK bit.
* Hence, it is required to wait the command execution time before issuing the read header.
* Commands must not be sent while a previous command is being processed.
*/
esp_err_t scd4x_send_command(scd4x_t *sensor, uint8_t *command) {
    return i2c_master_transmit(sensor->dev_handle, command, 2, CONFIG_SCD4X_TIMEOUT);
}

/*
* Data sent to and received from the sensor consists of a sequence of 16-bit commands and/or 16-bit words
* (each to be interpreted as unsigned integer, most significant byte transmitted first). Each data word is
* immediately succeeded by an 8-bit CRC. In write direction it is mandatory to transmit the checksum.
* In read direction it is up to the master to decide if it wants to process the checksum.
*/
esp_err_t scd4x_read(scd4x_t *sensor, uint8_t *addr, uint8_t *dout, uint8_t size) {
    esp_err_t err;

    err = i2c_master_transmit(sensor->dev_handle, addr, SCD4X_ADDR_SIZE, CONFIG_SCD4X_TIMEOUT);
    if (err != ESP_OK) return err;
    return i2c_master_receive(sensor->dev_handle, dout, size, CONFIG_SCD4X_TIMEOUT);
}

/*
* Data sent to and received from the sensor consists of a sequence of 16-bit commands and/or 16-bit words
* (each to be interpreted as unsigned integer, most significant byte transmitted first). Each data word is
* immediately succeeded by an 8-bit CRC. In write direction it is mandatory to transmit the checksum.
* In read direction it is up to the master to decide if it wants to process the checksum.
*/
esp_err_t scd4x_write(scd4x_t *sensor, uint8_t *addr, uint8_t *din, uint8_t size) {
    i2c_master_transmit_multi_buffer_info_t buffer[2] = {
        {.write_buffer = addr, .buffer_size = SCD4X_ADDR_SIZE},
        {.write_buffer = din, .buffer_size = size},
    };

    return i2c_master_multi_buffer_transmit(sensor->dev_handle, buffer, 2, CONFIG_SCD4X_TIMEOUT);
}

/*
* For the send command and fetch results sequences, after writing the address and/or data to the sensor
* and sending the ACK bit, the sensor needs the execution time to respond to the I2C read header with an ACK bit.
* Hence, it is required to wait the command execution time before issuing the read header.
* Commands must not be sent while a previous command is being processed.
*/
esp_err_t scd4x_send_command_and_fetch_result(scd4x_t *sensor, uint8_t *command, uint8_t *measurements, uint8_t size, uint16_t wait) {
    esp_err_t err;

    err = i2c_master_transmit(sensor->dev_handle, command, 2, CONFIG_SCD4X_TIMEOUT);
    if (err != ESP_OK) return err;
    err = i2c_master_transmit(sensor->dev_handle, measurements, size, -1);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(wait));
    return i2c_master_receive(sensor->dev_handle, measurements, size, -1);
}

/*
* Start periodic measurement, signal update interval is 5 seconds.
*/
esp_err_t scd4x_start_periodic_measurement(scd4x_t *sensor) {
    scd4x_st_machine_status = SCD4X_ST_MEASURE;
    st_machine_time = time(NULL);
    return scd4x_send_command(sensor, start_periodic_measurement);
}

/*
* Read sensor output. The measurement data can only be read out once per signal update interval
* as the buffer is emptied upon read-out.
*/
esp_err_t scd4x_read_measurement(scd4x_t *sensor) {
    measurements_t measurements = {
        .co2 = {{0x00, 0x00}, 0x00},
        .temperature = {{0x00, 0x00}, 0x00},
        .humidity = {{0x00, 0x00}, 0x00}
    };
    esp_err_t err;

    if (!scd4x_get_data_ready_status(sensor)) return ESP_FAIL;
    err = scd4x_read(sensor, read_measurement, (uint8_t *) &measurements, sizeof(measurements));
    sensor->values.co2 = (measurements.co2.value.msb << 8) + measurements.co2.value.lsb;
    sensor->values.temperature = (175.0 * (((measurements.temperature.value.msb << 8) + measurements.temperature.value.lsb) / 65535.0)) - 45.0;
    sensor->values.humidity = 100.0 * ((measurements.humidity.value.msb << 8) + measurements.humidity.value.lsb) / 65535.0;
    return err;
}

/*
* Stop periodic measurement to change the sensor configuration or to save power. Note that the sensor will only
* respond to other commands after waiting 500 ms after issuing the stop_periodic_measurement command.
*/
esp_err_t scd4x_stop_periodic_measurement(scd4x_t *sensor) {
    scd4x_st_machine_status = SCD4X_ST_IDLE;
    return scd4x_send_command(sensor, stop_periodic_measurement);
}

/*
* The temperature offset has no influence on the SCD4x CO2 accuracy. Setting the temperature offset of the SCD4x inside
* the customer device correctly allows the user to leverage the RH and T output signal. Note that the temperature offset
* can depend on various factors such as the SCD4x measurement mode, self-heating of close components, the ambient temperature
* and air flow. Thus, the SCD4x temperature offset should be determined inside the customer device under its typical operation
* conditions (including the operation mode to be used in the application) and in thermal equilibrium. Per default,
* the temperature offset is set to 4° C. To save the setting to the EEPROM, the persist setting command must be issued.
*/
esp_err_t scd4x_set_temperature_offset(scd4x_t *sensor, float temperature) {
    uint16_t offset = (temperature * 65536.0) / 175.0;

    scd4x_sensor_value_t temperature_offset = {
        .value = {offset >> 8, offset & 0xFF}
    };
    temperature_offset.crc = scd4x_calc_cksum((uint8_t *)&temperature_offset.value, sizeof(temperature_offset.value));

    return scd4x_write(sensor, set_temperature_offset, (uint8_t *)&temperature_offset, sizeof(temperature_offset));
}

/*
* Getting the temperature offset of the SCD4x from the EEPROM.
*/
float scd4x_get_temperature_offset(scd4x_t *sensor) {
    uint8_t buf[3];
    esp_err_t err;

    err = scd4x_read(sensor, get_temperature_offset, buf, 3);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "get_temperature_offset failed with status code: %s", esp_err_to_name(err));
        return SCD4X_READ_ERROR;
    }
    if (!scd4x_is_data_valid(buf, 3)) {
        ESP_LOGE(TAG, "get_temperature_offset returned invalid data: %02X%02X%02X", buf[0], buf[1], buf[2]);
        return SCD4X_READ_ERROR;
    }
    return round((175.0 * ((float)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]) / 65536.0)) * 10.0) / 10.0;
}

/*
* Reading and writing of the sensor altitude must be done while the SCD4x is in idle mode.
* Typically, the sensor altitude is set once after device installation. To save the setting to the EEPROM,
* the persist setting command must be issued. Per default, the sensor altitude is set to 0 meter above sea-level.
*/
esp_err_t scd4x_set_sensor_altitude(scd4x_t *sensor, uint16_t altitude) {
    esp_err_t err;
    scd4x_sensor_value_t sensor_altitude = {
        .value = {altitude >> 8, altitude & 0xFF}
    };

    sensor_altitude.crc = scd4x_calc_cksum((uint8_t *)&sensor_altitude.value, sizeof(sensor_altitude.value));
    if ((err = scd4x_write(sensor, set_sensor_altitude, (uint8_t *)&sensor_altitude, sizeof(sensor_altitude))) != ESP_OK) return err;
    sensor->altitude = altitude;
    return ESP_OK;
}

/*
* Getting the sensor altitude of the SCD4x from the EEPROM.
*/
uint16_t scd4x_get_sensor_altitude(scd4x_t *sensor) {
    esp_err_t err;
    uint8_t buf[3];

    if ((err = scd4x_read(sensor, get_sensor_altitude, buf, 3)) != ESP_OK) {
        ESP_LOGE(TAG, "get_sensor_altitude failed with status code: %s", esp_err_to_name(err));
        return SCD4X_READ_ERROR;
    }
    sensor->altitude = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    return sensor->altitude;
}

/*
* The set_ambient_pressure command can be sent during periodic measurements to enable continuous pressure compensation.
* Note that setting an ambient pressure using set_ambient_pressure overrides any pressure compensation based on
* a previously set sensor altitude. Use of this command is highly recommended for applications experiencing significant
* ambient pressure changes to ensure sensor accuracy.
*/
esp_err_t scd4x_set_ambient_pressure(scd4x_t *sensor, uint16_t pressure) {
    esp_err_t err;
    scd4x_sensor_value_t ambient_pressure = {
        .value = {pressure >> 8, pressure & 0xFF}
    };

    ambient_pressure.crc = scd4x_calc_cksum((uint8_t *)&ambient_pressure.value, sizeof(ambient_pressure.value));
    if ((err = scd4x_write(sensor, set_ambient_pressure, (uint8_t *)&ambient_pressure, sizeof(ambient_pressure))) != ESP_OK) return err;
    sensor->pressure = pressure;
    return ESP_OK;
}

/*
* To successfully conduct an accurate forced recalibration, the following steps need to be carried out:
*   1. Operate the SCD4x in the operation mode later used in normal sensor operation
*      (periodic measurement, low power periodic measurement or single shot) for > 3 minutes in an environment
*      with homogenous and constant CO2 concentration.
*   2. Issue stop_periodic_measurement. Wait 500 ms for the stop command to complete.
*   3. Subsequently issue the perform_forced_recalibration command and optionally read out the FRC correction
*      (i.e. themagnitude of the correction) after waiting for 400 ms for the command to complete.
*      A return value of 0xffff indicates that the forced recalibration has failed.
* Note that the sensor will fail to perform a forced recalibration if it was not operated before sending the command. Please make sure that the sensor is operated at the voltage desired for the application when applying the forced recalibration sequence.
*/
uint16_t scd4x_perform_forced_recalibration(scd4x_t *sensor, uint16_t target_concentration) {
    scd4x_sensor_value_t co2_concentration = {
        .value = {target_concentration >> 8, target_concentration & 0xFF}
    };
    co2_concentration.crc = scd4x_calc_cksum((uint8_t *) &co2_concentration.value, sizeof(co2_concentration.value));

    esp_err_t err = scd4x_send_command_and_fetch_result(sensor, perform_forced_recalibration, (uint8_t *)&co2_concentration, sizeof(co2_concentration), 450);

    if (err != ESP_OK || (co2_concentration.value.msb = 0xFF && co2_concentration.value.lsb == 0xFF)) {
        ESP_LOGE(TAG, "perform_forced_recalibration failed with status code: %s", esp_err_to_name(err));
        return SCD4X_READ_ERROR;
    }
    return ((co2_concentration.value.msb << 8) + co2_concentration.value.lsb) - 0x8000;
}

/*
* Set the current state (enabled / disabled) of the automatic self-calibration. By default, ASC is enabled.
* To save the setting to the EEPROM, the persist_setting command must be issued.
*/
esp_err_t scd4x_set_automatic_self_calibration_enabled(scd4x_t *sensor, bool asc_enabled) {
    scd4x_sensor_value_t automatic_self_calibration = {
        .value = {0x00, asc_enabled ? 0x01 : 0x00},
        .crc = asc_enabled ? 0xB1 : 0x81
    };

    return scd4x_write(sensor, set_automatic_self_calibration_enabled, (uint8_t *) &automatic_self_calibration, sizeof(automatic_self_calibration));
}

/*
* Getting the automatic self calibration status of the SCD4x from the EEPROM.
*/
bool scd4x_get_automatic_self_calibration_enabled(scd4x_t *sensor) {
    scd4x_sensor_value_t automatic_self_calibration = {
        .value = {0x00, 0x00},
        .crc = 0x00
    };

    esp_err_t err = scd4x_read(sensor, get_automatic_self_calibration_enabled, (uint8_t *) &automatic_self_calibration, sizeof(automatic_self_calibration));

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "get_automatic_self_calibration_enabled failed with status code: %s", esp_err_to_name(err));
        return false;
    }
    return automatic_self_calibration.value.lsb ? true : false;
}

/*
* start low power periodic measurement, signal update interval is approximately 30 seconds.
*/
esp_err_t scd4x_start_low_power_periodic_measurement(scd4x_t *sensor) {
    scd4x_st_machine_status = SCD4X_ST_MEASURE;
    st_machine_time = time(NULL);
    return scd4x_send_command(sensor, start_low_power_periodic_measurement);
}

/*
* Getting the SCD4x sensor output read status.
*/
bool scd4x_get_data_ready_status(scd4x_t *sensor) {
    scd4x_sensor_value_t data_ready_status = {
        .value = {0x00, 0x00},
        .crc = 0x00
    };

    esp_err_t err = scd4x_read(sensor, get_data_ready_status, (uint8_t *) &data_ready_status, sizeof(data_ready_status));

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "get_data_ready_status failed with status code: %s", esp_err_to_name(err));
        return false;
    }
    return ((data_ready_status.value.msb & 0x0F) == 0x00 && data_ready_status.value.lsb == 0x00) ? false : true;
}

/*
* Configuration settings such as the temperature offset, sensor altitude and the ASC enabled/disabled parameter
* are by default stored in the volatile memory (RAM) only and will be lost after a power-cycle.
* The persist_settings command stores the current configuration in the EEPROM of the SCD4x, making them persistent
* across power-cycling. To avoid unnecessary wear of the EEPROM, the persist_settings command should only be sent when
* persistence is required and if actual changes to the configuration have been made. The EEPROM is guaranteed to endure
* at least 2000 write cycles before failure. Note that field calibration history is automatically stored in
* a separate EEPROM dimensioned for the specified sensor lifetime.
*/
esp_err_t scd4x_persist_settings(scd4x_t *sensor) {
    return scd4x_send_command(sensor, persist_settings);
}

/*
* Reading out the serial number can be used to identify the chip and to verify the presence of the sensor.
* The get serial number command returns 3 words, and every word is followed by an 8-bit CRC checksum.
* Together, the 3 words constitute a unique serial number with a length of 48 bits (big endian format).
*/
esp_err_t scd4x_get_serial_number(scd4x_t *sensor) {
    esp_err_t err;
    uint8_t buf[9]; // Serial number is 12 digits plus trailing NULL

    ESP_LOGI(TAG, "scd4x_get_serial_number");
    for (int i = 0; i < 5; i++) {
        err = scd4x_read(sensor, get_serial_number, (uint8_t *)&buf, sizeof(buf));
        if (err == ESP_OK && scd4x_is_data_valid(buf, 9)) {
            //ESP_LOG_BUFFER_HEXDUMP(TAG, buf, 9, ESP_LOG_INFO);
            sensor->serial_number = (uint64_t)buf[0] << 40 | (uint64_t)buf[1] << 32 | (uint64_t)buf[3] << 24 | (uint64_t)buf[4] << 16 | (uint64_t)buf[6] << 8 | (uint64_t)buf[7];
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return SCD4X_READ_ERROR;
}

/*
* The perform_self_test feature can be used as an end-of-line test to check sensor functionality and the customer
* power supply to the sensor.
*/
bool scd4x_perform_self_test(scd4x_t *sensor) {
    scd4x_sensor_value_t self_test = {
        .value = {0x00, 0x00},
        .crc = 0x00
    };

    esp_err_t err = scd4x_read(sensor, perform_self_test, (uint8_t *) &self_test, sizeof(self_test));

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "perform_self_test failed with status code: %s", esp_err_to_name(err));
        return false;
    }
    return (self_test.value.msb == 0x00 && self_test.value.lsb == 0x00) ? true : false;
}

/*
* The perform_factory_reset command resets all configuration settings stored in the EEPROM and erases the
* FRC and ASC algorithm history.
*/
esp_err_t scd4x_perfom_factory_reset(scd4x_t *sensor) {
    return scd4x_send_command(sensor, perfom_factory_reset);
}

/*
* The reinit command reinitializes the sensor by reloading user settings from EEPROM.
* Before sending the reinit command, the stop measurement command must be issued.
* If the reinit command does not trigger the desired re-initialization, a power-cycle should be applied to the SCD4x.
*/
esp_err_t scd4x_reinit(scd4x_t *sensor) {
    ESP_LOGI(TAG, "scd4x_reinit");
    for (int i = 0; i < 10; i++) {
        esp_err_t err = scd4x_send_command(sensor, reinit);
        if (err == ESP_OK) return err;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return ESP_OK; //scd4x_send_command(scd4x, reinit);
}

/* 
* On-demand measurement of CO2 concentration, relative humidity and temperature. The sensor output is read
* using the read_measurement command.
*/
esp_err_t scd4x_measure_single_shot(scd4x_t *sensor) {
    return scd4x_send_command(sensor, measure_single_shot);
}

/*
* On-demand measurement of relative humidity and temperature only. The sensor output is read using the
* read_measurement command. CO2 output is returned as 0 ppm.
*/
esp_err_t scd4x_measure_single_shot_rht_only(scd4x_t *sensor) {
    return scd4x_send_command(sensor, measure_single_shot_rht_only);
}

/*
* Put the sensor from idle to sleep to reduce current consumption. Can be used to power down when operating the
* sensor in power-cycled single shot mode.
*/
esp_err_t scd4x_power_down(scd4x_t *sensor) {
    if (sensor == NULL) return ESP_FAIL;
    sensor->enabled = false;
    scd4x_st_machine_status = SCD4X_ST_IDLE;
    return scd4x_send_command(sensor, power_down);
}

/*
* Wake up the sensor from sleep mode into idle mode. Note that the SCD4x does not acknowledge the wake_up command.
* To verify that the sensor is in the idle state after issuing the wake_up command, the serial number can be read out.
* Note that the first reading obtained using measure_single_shot after waking up the sensor should be discarded.
*/
esp_err_t scd4x_wake_up(scd4x_t *sensor) {
    return scd4x_send_command(sensor, wake_up);
}

void scd4x_state_machine_cmd(uint16_t cmd, uint32_t arg)
{
    st_machine_cmd = cmd;
    st_machine_arg = arg;
}

/* Return 0 for idle, 1 for busy and -1 for error */
int scd4x_state_machine(scd4x_t *sensor)
{
    time_t now = time(NULL);
    esp_err_t err;

    if (scd4x_st_machine_status == SCD4X_ST_MEASURE) {
        if (now - st_machine_time > 180) {
            ESP_LOGI(TAG, "SCD4x is measuring for >3min");
            scd4x_st_machine_status = SCD4X_ST_MEASURE_3MIN;
        }
    } else if (scd4x_st_machine_status == SCD4X_ST_FRC_INIT) {
        if (now - st_machine_time > 0) {
            ESP_LOGI(TAG, "SCD4x forced recalibration run with CO2 value %d", st_machine_arg);
            uint16_t result = scd4x_perform_forced_recalibration(sensor, st_machine_arg);
            if (result == 0xffff) {
                ESP_LOGE(TAG, "Failed to calibrate sensor");
                if ((err = scd4x_start_periodic_measurement(sensor)) != ESP_OK) return -1;
                scd4x_st_machine_status = SCD4X_ST_MEASURE;
                st_machine_time = now;
                return -1;
            }
            scd4x_st_machine_status = SCD4X_ST_FRC_RUN;
            st_machine_time = now;
        }
    } else if (scd4x_st_machine_status == SCD4X_ST_FRC_RUN) {
        if (now - st_machine_time > 0) {
            if ((err = scd4x_start_periodic_measurement(sensor)) != ESP_OK) return -1;
            ESP_LOGI(TAG, "SCD4x forced recalibration finished");
            scd4x_st_machine_status = SCD4X_ST_MEASURE;
            st_machine_time = now;
        }
    }
    if (st_machine_cmd == SCD4X_CMD_FRC) {
        st_machine_cmd = 0;
        if (scd4x_st_machine_status == SCD4X_ST_MEASURE) return -1;
        if ((err = scd4x_stop_periodic_measurement(sensor)) != ESP_OK) return -1;
        ESP_LOGI(TAG, "SCD4x forced recalibration init");
        scd4x_st_machine_status = SCD4X_ST_FRC_INIT;
        st_machine_time = now;
    }
    return scd4x_st_machine_status != SCD4X_ST_IDLE ? 1 : 0;
}

void scd4x_dump_values(scd4x_t *sensor, bool force)
{
    if (force || sensor->debug & 1) {
        scd4x_values_t *values = &sensor->values;

        ESP_LOGI(TAG, "temp_offs=%f °C  altitude=%d m  pressure=%d hPa * co2=%d ppm  temp=%.1f °C  hum=%.1f %%",
                 sensor->temperature_offset, sensor->altitude, sensor->pressure,
                 values->co2, values->temperature, values->humidity);
    }
}
