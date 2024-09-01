/*
 * sdpsensor.h
 *
 *  Created on: Sep 7, 2021
 *      Author: Danylo Ulianych
 */

#ifndef SDPSENSOR_H_
#define SDPSENSOR_H_

#include "esp_err.h"
#include "driver/i2c.h"


class SDPSensor {
    private:
        uint8_t i2c_addr;                    /* I2C address */
        i2c_port_t i2c_port;                 /* I2C master port */
        uint16_t pressureScale;              /* Diff pressure scale */
        float pressureScaleFloat;
        bool initialized;

        /* Information parameters */
        uint32_t modelNumber;
        uint32_t rangePa;
        uint32_t productId;
        uint64_t serialNumber;

    public:

        /**
         * Constructor
         *
         * @param i2c_addr - I2C address.
         * @param i2c_port - I2C port.
         *                   ESP boards have two I2C peripherals.
         *                   Defaults to 0.
         */
        SDPSensor(uint8_t i2c_addr, i2c_port_t i2c_port = 0);


        /**
         * Initialize I2C. Same as `Wire.begin(SDA, SCL)`.
         *
         * @param pinSDA - SDA GPIO
         * @param pinSCL - SCL GPIO
         *
         * @returns the error code (defined in esp_err.h)
         */
        esp_err_t initI2C(int pinSDA, int pinSCL);


        /**
         * Initialize an SDP sensor.
         *
         * You may need to call `stopContinuous()` or `reset()` function
         * prior to initializing the sensor, if previously it was working in
         * a continuous mode.
         * The product ID and the diff pressure scale are read and saved here.
         * The returned error code other than ESP_OK (0) means that the sensor
         * has not been properly initialized.
         *
         * @returns the error code (defined in esp_err.h)
         */
        esp_err_t begin();

        /**
         * Get sensor info parameters. Arguments can be NULL.
         *
         * @param modelNumber  - a pointer to save the model number (3x or 8xx)
         * @param rangePa      - a pointer to save the measurement range (in Pa)
         * @param productId    - a pointer to save the product ID (combination of two above)
         * @param serialNumber - a pointer to save the unique serial number
         */
        void getInfo(uint32_t *modelNumber, uint32_t *rangePa, uint32_t *productId, uint64_t *serialNumber);

        /**
         * Return the diff pressure scale, saved in the `initSensor()` call.
         *
         * @returns diff pressure scale
         */
        uint16_t getPressureScale();


        /**
         * Start the sensor in the continuous mode.
         *
         * @returns the error code (defined in esp_err.h)
         */
        esp_err_t startContinuous();


        /**
         * Stop the sensor continuous mode.
         *
         * @returns the error code (defined in esp_err.h)
         */
        esp_err_t stopContinuous();


        /**
         * Reset the SDP sensor.
         * All other sensors connected to the same I2C line
         * (same port) will also receive the reset command.
         *
         * @returns the error code (defined in esp_err.h)
         */
        esp_err_t reset();

        /**
         * Attach the IRQ handler callback to a dedicated GPIO sensor pin.
         * Only the SDP3x sensor series have an IRQ pin.
         *
         * @param irqGPIO     - GPIO interrupt pin number
         * @param irqHandler  - interrupt handler callback function
         * @returns the error code (defined in esp_err.h)
         */
        esp_err_t attachIRQHandler(int irqGPIO, void (*irqHandler)() );

        /**
         * Read the raw (unnormalized) differential pressure value and
         * save the result in `diffPressureRaw`. To convert it to a
         * physical value in Pa, one should divide it by the pressure
         * scale (see the `getPressureScale()` function).
         *
         * This call is non-blocking (zero I2C timeout).
         *
         * @param diffPressureRaw - a pointer to save the result
         * @returns the error code (defined in esp_err.h):
         *    ESP_OK                 - success
         *    ESP_FAIL               - failure
         *    ESP_ERR_TIMEOUT        - timed out
         *    ESP_ERR_INVALID_CRC    - CRC mismatch
         *    ESP_ERR_INVALID_STATE  - not initialized
         */
        esp_err_t readDiffPressure(float &diffPressure);


        /**
         * Read the differential pressure value AND the temperature.
         * 
         * This call is non-blocking (zero I2C timeout).
         * 
         * @param diffPressureRaw - a pointer to save the diff pressure
         * @param temperature - a pointer to save the temperature in Celsius
         * @returns the error code (defined in esp_err.h)
         */
        esp_err_t readDiffPressureTemperature(float &diffPressure, float &temperature);
};

#endif /* SDPSENSOR_H_ */
