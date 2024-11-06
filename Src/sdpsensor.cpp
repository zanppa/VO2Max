/*
 * sdpsensor.c
 *
 *  Created on: Sep 7, 2021
 *      Author: Danylo Ulianych
 */

/* Modified slighly Jan 2024 
 * by Lauri Peltonen
 *
 * - Changed continuous mode to mass-flow calibrated, averaged
 * - Corrected the conversion of values to signed integer
 * - Changed pressure/temperature read to scale both to float directly
 *
 */

#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "sdpsensor.h"

#define SPD31_500_PID  0x03010100
#define SDP32_125_PID  0x03010200
#define SDP800_500_PID 0x03020100
#define SDP810_500_PID 0x03020A00
#define SDP801_500_PID 0x03020400
#define SDP811_500_PID 0x03020D00
#define SDP800_125_PID 0x03020200
#define SDP810_125_PID 0x03020B00

#define SDPSENSOR_I2C_CMD_LEN       2
#define SDPSENSOR_TEMPERATURE_SCALE (200.0f)

const TickType_t I2C_DEFAULT_TIMEOUT = pdMS_TO_TICKS(1);   // Maximum 1 ms wait if bus is busy

const char *TAG_SDPSENSOR = "sdpsensor";

/**
 * Compute CRC from data.
 * See http://www.sunshine2k.de/articles/coding/crc/understanding_crc.html
 * 
 * @param data - I2C read bytes of size 2
 * @returns crc - the CRC value from the data
 */
uint8_t computeCRC(uint8_t* data)
{
    const uint8_t generator = 0x31;  /* Polynomial */
    uint8_t crc = 0xFF; /* Initial value from the datasheet */

    for (int i = 0; i < 2; i++) {  /* two-bytes input data */
        crc ^= data[i];  /* XOR-in the next input byte */
        for (int j = 0; j < 8; j++) {
            if ((crc & 0x80) != 0) {
                crc = (uint8_t) ((crc << 1) ^ generator);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}


SDPSensor::SDPSensor(uint8_t i2c_addr, i2c_port_t i2c_port) {
    this->i2c_addr = i2c_addr;
    this->i2c_port = i2c_port;
    this->pressureScale = 0;  // will be read & set in the begin()
    this->pressureScaleFloat = 1.0; // Will be set in begin()
    this->initialized = false;
    this->modelNumber = 0;
    this->rangePa = 0;
    this->productId = 0;
    this->serialNumber = 0;
}


esp_err_t SDPSensor::initI2C(int pinSDA, int pinSCL) {
    int intr_flag_disable = 0;

    /* I2C master doesn't need buffer */
    size_t i2c_master_rx_buf_disable = 0;
    size_t i2c_master_tx_buf_disable = 0;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = pinSDA,
        .scl_io_num = pinSCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    };
    conf.master.clk_speed = 100000;  /*!< I2C master clock frequency */

    esp_err_t err;
    err = i2c_param_config(i2c_port, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SDPSENSOR, "Could not configure the I2C: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(i2c_port, conf.mode,
            i2c_master_rx_buf_disable, i2c_master_tx_buf_disable,
            intr_flag_disable);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SDPSENSOR, "Failed to initialize the I2C: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG_SDPSENSOR, "I2C%d line initialized", i2c_port);

    return err;
}


void logInitFailed(esp_err_t err) {
    ESP_LOGE(TAG_SDPSENSOR, "SDPSensor::begin failed: %s", esp_err_to_name(err));
}


esp_err_t SDPSensor::begin() {
    // commands to read product id
    uint8_t cmd0[SDPSENSOR_I2C_CMD_LEN] = { 0x36, 0x7C };
    uint8_t cmd1[SDPSENSOR_I2C_CMD_LEN] = { 0xE1, 0x02 };

    // command to trigger a one-time measurement
    uint8_t cmd_measure[SDPSENSOR_I2C_CMD_LEN] = { 0x36, 0x2F };

    uint8_t read_buffer[18] = { 0 };

    const TickType_t ticks_to_wait_long = pdMS_TO_TICKS(100);
    esp_err_t err;
    //err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd0, SDPSENSOR_I2C_CMD_LEN, ticks_to_wait_long);
    err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd0, SDPSENSOR_I2C_CMD_LEN, ticks_to_wait_long);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SDPSENSOR, "SDPSensor::begin failed: %s", esp_err_to_name(err));
        return err;
    }
    err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd1, SDPSENSOR_I2C_CMD_LEN, ticks_to_wait_long);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SDPSENSOR, "SDPSensor::begin failed: %s", esp_err_to_name(err));
        return err;
    }

    /*
       Read product id and serial number.
       Data Format:
       | Byte  | 0 | 1 | 2 | 3 | 4 | 5 | 6...17 |
       | Value |  pid1 |CRC|  pid2 |CRC| serial |
       */
    err = i2c_master_read_from_device(i2c_port, i2c_addr, read_buffer, 18, ticks_to_wait_long);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SDPSENSOR, "SDPSensor::begin failed: %s", esp_err_to_name(err));
        return err;
    }

    productId = (read_buffer[0] << 24) | (read_buffer[1] << 16)
        | (read_buffer[3] << 8) | (read_buffer[4] << 0);

    uint64_t *pserial = &serialNumber;
    for (int i = 6; i < 18; i++) {
        if ((i + 1) % 3 == 0) continue;  // CRC
        *pserial <<= 8;
        *pserial |= (uint64_t) read_buffer[i];
    }

    modelNumber, rangePa;
    switch (productId & 0xFFFFFF00) {
        case SPD31_500_PID:
            modelNumber = 31;
            rangePa = 500;
            break;
        case SDP32_125_PID:
            modelNumber = 32;
            rangePa = 125;
            break;
        case SDP800_500_PID:
            modelNumber = 800;
            rangePa = 500;
            break;
        case SDP810_500_PID:
            modelNumber = 810;
            rangePa = 500;
            break;
        case SDP801_500_PID:
            modelNumber = 801;
            rangePa = 500;
            break;
        case SDP811_500_PID:
            modelNumber = 811;
            rangePa = 500;
            break;
        case SDP800_125_PID:
            modelNumber = 800;
            rangePa = 125;
            break;
        case SDP810_125_PID:
            modelNumber = 810;
            rangePa = 125;
            break;
    }

    ESP_LOGI(TAG_SDPSENSOR, "Initialized SDP%d %dPa sensor (PID=0x%08X), serial=0x%016llX", modelNumber, rangePa, productId, serialNumber);

    err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd_measure, SDPSENSOR_I2C_CMD_LEN, ticks_to_wait_long);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SDPSENSOR, "SDPSensor::begin failed: %s", esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(90));  // theoretically 45 ms

    err = i2c_master_read_from_device(i2c_port, i2c_addr, read_buffer, 9, ticks_to_wait_long);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SDPSENSOR, "SDPSensor::begin failed: %s", esp_err_to_name(err));
        return err;
    }

    this->pressureScale = ((int16_t) read_buffer[6]) << 8 | read_buffer[7];
    this->pressureScaleFloat = (float)this->pressureScale;

    ESP_LOGI(TAG_SDPSENSOR, "SDP%d pressure scale: %d", modelNumber, this->pressureScale);

    this->initialized = true;

    return err;
}


void SDPSensor::getInfo(uint32_t *modelNumber, uint32_t *rangePa, uint32_t *productId, uint64_t *serialNumber) {
    if (modelNumber != NULL) *modelNumber = this->modelNumber;
    if (rangePa != NULL) *rangePa = this->rangePa;
    if (productId != NULL) *productId = this->productId;
    if (serialNumber != NULL) *serialNumber = this->serialNumber;
}


uint16_t SDPSensor::getPressureScale() {
    return pressureScale;
}


esp_err_t SDPSensor::startContinuous() {
    uint8_t cmd[SDPSENSOR_I2C_CMD_LEN] = { 0x36, 0x03 }; // Original: { 0x36, 0x1E };
    const TickType_t ticks_to_wait_long = pdMS_TO_TICKS(100);
    esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd,
            SDPSENSOR_I2C_CMD_LEN, ticks_to_wait_long);
    ESP_LOGI(TAG_SDPSENSOR, "SDPSensor::startContinuous %s",
            esp_err_to_name(err));
    // wait for sensor to start continuously making measurements
    vTaskDelay(pdMS_TO_TICKS(25));
    return err;
}


esp_err_t SDPSensor::stopContinuous() {
    uint8_t cmd[SDPSENSOR_I2C_CMD_LEN] = { 0x3F, 0xF9 };
    const TickType_t ticks_to_wait_long = pdMS_TO_TICKS(100);
    esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd,
            SDPSENSOR_I2C_CMD_LEN, ticks_to_wait_long);
    ESP_LOGI(TAG_SDPSENSOR, "SDPSensor::stopContinuous %s",
            esp_err_to_name(err));
    vTaskDelay(pdMS_TO_TICKS(25));
    return err;
}


esp_err_t SDPSensor::reset() {
    uint8_t cmd[1] = { 0x06 };
    const TickType_t ticks_to_wait_long = pdMS_TO_TICKS(100);
    esp_err_t err = i2c_master_write_to_device(i2c_port, 0x00, cmd,
            1, ticks_to_wait_long);
    ESP_LOGI(TAG_SDPSENSOR, "SDPSensor::reset %s", esp_err_to_name(err));
    vTaskDelay(pdMS_TO_TICKS(25));
    return err;
}


esp_err_t SDPSensor::attachIRQHandler(int irqGPIO, void (*irqHandler)() ) {
    gpio_config_t irq_io_conf = {
    		.pin_bit_mask = 1ULL << irqGPIO,            // IRQ pin
			.mode = GPIO_MODE_INPUT,                    // input mode
			.pull_up_en = GPIO_PULLUP_ENABLE,           // enable pull-up
			.pull_down_en = GPIO_PULLDOWN_DISABLE,      // disable pull-down
			.intr_type = GPIO_INTR_NEGEDGE              // falling edge interrupt
    };
    esp_err_t err;
    err = gpio_config(&irq_io_conf);
    if (err != ESP_OK) {
        return err;
    }
    // install the GPIO ISR service
    err = gpio_install_isr_service(0);
    if (err != ESP_OK) {
        return err;
    }
    // hook an ISR handler for the specific GPIO pin
    err = gpio_isr_handler_add((gpio_num_t) irqGPIO, (gpio_isr_t) irqHandler, NULL);
    if (err == ESP_OK) {
        ESP_LOGI(TAG_SDPSENSOR, "Attached an IRQ handler to GPIO pin %d", irqGPIO);
    }
    return err;
}


esp_err_t SDPSensor::readDiffPressure(float &diffPressure) {
    if (!initialized) return ESP_ERR_INVALID_STATE;

    uint8_t data[3] = { 0 };
    esp_err_t err = i2c_master_read_from_device(i2c_port, i2c_addr, data, 3,
            I2C_DEFAULT_TIMEOUT);
    if (err == ESP_OK) {
        if (data[2] != computeCRC(data)) {
            err = ESP_ERR_INVALID_CRC;
        } else {
            diffPressure = (float)((int16_t)((data[0] << 8) | data[1])) / this->pressureScaleFloat;
        }
    }
    return err;
}


esp_err_t SDPSensor::readDiffPressureTemperature(float &diffPressure, float &temperature) {
    if (!initialized) return ESP_ERR_INVALID_STATE;

    uint8_t data[6] = { 0 };

    /*
       Data Format:
       | Byte  |  0  |  1  |  2  |  3  |  4  |  5  |
       | Value | pressure  | CRC |temperature| CRC |
       */
    esp_err_t err = i2c_master_read_from_device(i2c_port, i2c_addr, data, 6, I2C_DEFAULT_TIMEOUT);

    if (err == ESP_OK) {
        if (( data[2] != computeCRC(data) ) || ( data[5] != computeCRC(&data[3]) )) {
            err = ESP_ERR_INVALID_CRC;
        } else {
          diffPressure = (float)((int16_t)((data[0] << 8) | data[1])) / this->pressureScaleFloat;
          int16_t temp_raw = ((int16_t) data[3]) << 8 | data[4];
          temperature = (float)temp_raw / SDPSENSOR_TEMPERATURE_SCALE;
        }
    }

    return err;
}
