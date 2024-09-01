/*
  BMP388Sensor (BMP388_DEV) is an I2C/SPI compatible library for the Bosch BMP388 barometer.
	
	Copyright (C) Martin Lindupp 2020
	
	V1.0.0 -- Initial release 	
	V1.0.1 -- Fix uninitialised structures, thanks to David Jade for investigating and flagging up this issue
	V1.0.2 -- Modification to allow user-defined pins for I2C operation on the ESP32
	V1.0.3 -- Initialise "device" constructor member variables in the same order they are declared
	V1.0.4 -- Fix incorrect oversampling definition for x1, thanks to myval for raising the issue
	V1.0.5 -- Modification to allow ESP8266 SPI operation, thanks to Adam9850 for the generating the pull request
	V1.0.6 -- Include getErrorReg() and getStatusReg() functions
	V1.0.7 -- Fix compilation issue with Arduino Due
	V1.0.8 -- Allow for additional TwoWire instances
	V1.0.9 -- Fix compilation issue with STM32 Blue Pill
	V1.0.10 -- Removed default parameter causing ESP32 compilation error with user defined I2C pins	
	V1.0.11 -- Fixed uninitialised "Wire" pointer for ESP8266/ESP32 with user defined I2C pins 
	
	The MIT License (MIT)
	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:
	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.
	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/

/*
 * Modified Jan 2024
 * by Lauri Peltonen
 *
 * - Renamed to BMP388Sensor
 * - Got rid of the "Device" class
 * - From Wire to direct i2c
 * - Return values from 0=ok to esp_err_t and ESP_OK
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "esp_log.h"

#include "BMP388sensor.h"

#define I2C_NO_TIMEOUT              0  // non-blocking

static const TickType_t ticks_to_wait = pdMS_TO_TICKS(1);
const char *TAG_BMPSENSOR = "BMP388sensor";

////////////////////////////////////////////////////////////////////////////////
// BMP388Sensor Class Constructors
////////////////////////////////////////////////////////////////////////////////

BMP388Sensor::BMP388Sensor(uint8_t i2c_addr, i2c_port_t i2c_port) : i2c_addr{i2c_addr}, i2c_port{i2c_port} { }		// Constructor for I2C communications	
////////////////////////////////////////////////////////////////////////////////
// BMP388Sensor Public Member Functions
////////////////////////////////////////////////////////////////////////////////

uint8_t BMP388Sensor::begin(Mode mode, 																// Initialise BMP388 device settings
												  Oversampling presOversampling, 
													Oversampling tempOversampling,
													IIRFilter iirFilter,
													TimeStandby timeStandby)
{
  // I2c must be initialized before calling this
  esp_err_t err;
  uint8_t cmd0[1] = { 0x00 };
  uint8_t cmd1[1] = { BMP388_TRIM_PARAMS };

	if ((err = reset()) != ESP_OK)                                                     // Reset the BMP388 barometer
	{
		return err;																												// If unable to reset return 0
	}
	
	uint8_t chipId;
  err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd0, 1, ticks_to_wait);
  if(err != ESP_OK) {
    ESP_LOGE(TAG_BMPSENSOR, "Write register 0 failed (%s)", esp_err_to_name(err));
    return err;
  }
  err = i2c_master_read_from_device(i2c_port, i2c_addr, (uint8_t *)&chipId, 1,
              I2C_NO_TIMEOUT);    // Read the device ID	
  if (err != ESP_OK)     			// Check the device ID
  {
    ESP_LOGE(TAG_BMPSENSOR, "Device ID read failed (%s)", esp_err_to_name(err));
		return err;                                                     	// If the ID is incorrect return 0
  }	 
  if (chipId != BMP388_ID && chipId != BMP390_ID)     			// Check the device ID
  {
    ESP_LOGE(TAG_BMPSENSOR, "Device ID incorrect got (%x), expected (%x)", chipId, BMP388_ID);
		return ESP_FAIL;                                                     	// If the ID is incorrect return 0
  }	 


  err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd1, 1, ticks_to_wait);
  if(err != ESP_OK) {
    ESP_LOGE(TAG_BMPSENSOR, "Read from calibration failed (%s)", esp_err_to_name(err));
    return err;
  }
  // Read the trim parameters into the params structure
  err = i2c_master_read_from_device(i2c_port, i2c_addr, (uint8_t *)&params, sizeof(params), I2C_NO_TIMEOUT);
  if (err != ESP_OK)     			// Check the device ID
  {
    ESP_LOGE(TAG_BMPSENSOR, "Read calibration failed (%s)", esp_err_to_name(err));
		return err;                                                     	// If the ID is incorrect return 0
  }	 
  //readBytes(BMP388_TRIM_PARAMS, (uint8_t*)&params, sizeof(params));
	floatParams.param_T1 = (float)params.param_T1 / powf(2.0f, -8.0f); // Calculate the floating point trim parameters
	floatParams.param_T2 = (float)params.param_T2 / powf(2.0f, 30.0f);
	floatParams.param_T3 = (float)params.param_T3 / powf(2.0f, 48.0f);
	floatParams.param_P1 = ((float)params.param_P1 - powf(2.0f, 14.0f)) / powf(2.0f, 20.0f);
	floatParams.param_P2 = ((float)params.param_P2 - powf(2.0f, 14.0f)) / powf(2.0f, 29.0f);
	floatParams.param_P3 = (float)params.param_P3 / powf(2.0f, 32.0f);
	floatParams.param_P4 = (float)params.param_P4 / powf(2.0f, 37.0f);
	floatParams.param_P5 = (float)params.param_P5 / powf(2.0f, -3.0f);
	floatParams.param_P6 = (float)params.param_P6 / powf(2.0f, 6.0f);
	floatParams.param_P7 = (float)params.param_P7 / powf(2.0f, 8.0f);
	floatParams.param_P8 = (float)params.param_P8 / powf(2.0f, 15.0f);
	floatParams.param_P9 = (float)params.param_P9 / powf(2.0f, 48.0f);
	floatParams.param_P10 = (float)params.param_P10 / powf(2.0f, 48.0f);
	floatParams.param_P11 = (float)params.param_P11 / powf(2.0f, 65.0f);
	setIIRFilter(iirFilter);																					// Initialise the BMP388 IIR filter register
	setTimeStandby(timeStandby); 																			// Initialise the BMP388 standby time register
	setOversamplingRegister(presOversampling, tempOversampling);			// Initialise the BMP388 oversampling register	
	pwr_ctrl.bit.press_en = 1;																				// Set power control register to enable pressure sensor
	pwr_ctrl.bit.temp_en = 1;																					// Set power control register to enable temperature sensor
	setMode(mode);																										// Set the BMP388 mode

  if(chipId == BMP388_ID)
    ESP_LOGI(TAG_BMPSENSOR, "BMP388 Initialized, address 0x%x", i2c_addr);
  else if(chipId == BMP390_ID)
    ESP_LOGI(TAG_BMPSENSOR, "BMP390 Initialized, address 0x%x", i2c_addr);

	return ESP_OK;																													// Report successful initialisation
}

uint8_t BMP388Sensor::begin(Mode mode, uint8_t addr)									// Initialise BMP388 with default settings, but selected mode and
{																																		// I2C address
	setI2CAddress(addr);
	return begin(mode);
}

uint8_t BMP388Sensor::begin(uint8_t addr)															// Initialise BMP388 with default settings and selected I2C address
{
	setI2CAddress(addr);
	return begin();
}

uint8_t BMP388Sensor::reset()																					// Reset the BMP388 barometer
{
  uint8_t cmd0[2] = { BMP388_CMD, RESET_CODE };
  uint8_t cmd1[1] = { BMP388_EVENT };
  // Write the reset code to the command register
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd0, 2, ticks_to_wait);
  if(err != ESP_OK) {
    ESP_LOGE(TAG_BMPSENSOR, "Reset failed (%s)", esp_err_to_name(err));
    return err;
  }
	//writeByte(BMP388_CMD, RESET_CODE);
  vTaskDelay(pdMS_TO_TICKS(10));																												// Wait for 10ms

  // Read the BMP388's event register
  //event.reg = readByte(BMP388_EVENT);
  err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd1, 1, ticks_to_wait);
  if(err != ESP_OK) {
    ESP_LOGE(TAG_BMPSENSOR, "Write event failed (%s)", esp_err_to_name(err));
    return err;
  }
  err = i2c_master_read_from_device(i2c_port, i2c_addr, (uint8_t *)&event.reg, 1, I2C_NO_TIMEOUT);
  if(err != ESP_OK) {
    ESP_LOGE(TAG_BMPSENSOR, "Read event failed (%s)", esp_err_to_name(err));
    return err;
  }
	return event.bit.por_detected ? ESP_OK : ESP_FAIL; 																		// Return if device reset is complete																					
}

void BMP388Sensor::startNormalConversion() { setMode(NORMAL_MODE); }	// Start continuous measurement in NORMAL_MODE

void BMP388Sensor::startForcedConversion() 														// Start a one shot measurement in FORCED_MODE
{ 
	if (pwr_ctrl.bit.mode == SLEEP_MODE)															// Only set FORCED_MODE if we're already in SLEEP_MODE
	{
		setMode(FORCED_MODE);
	}	
}			

void BMP388Sensor::stopConversion() { setMode(SLEEP_MODE); }					// Stop the conversion and return to SLEEP_MODE

void BMP388Sensor::setPresOversampling(Oversampling presOversampling)	// Set the pressure oversampling rate
{
  osr.bit.osr_p = presOversampling;
  uint8_t cmd[2] = { BMP388_OSR, osr.reg };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 2, ticks_to_wait);
	//writeByte(BMP388_OSR, osr.reg);
}

void BMP388Sensor::setTempOversampling(Oversampling tempOversampling)	// Set the temperature oversampling rate
{
	osr.bit.osr_t = tempOversampling;
  uint8_t cmd[2] = { BMP388_OSR, osr.reg };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 2, ticks_to_wait);
	//writeByte(BMP388_OSR, osr.reg);
}

void BMP388Sensor::setIIRFilter(IIRFilter iirFilter)									// Set the IIR filter setting
{
	config.bit.iir_filter = iirFilter;
  uint8_t cmd[2] = { BMP388_CONFIG, config.reg };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 2, ticks_to_wait);
	//writeByte(BMP388_CONFIG, config.reg);
}

void BMP388Sensor::setTimeStandby(TimeStandby timeStandby)						// Set the time standby measurement interval
{
	odr.bit.odr_sel = timeStandby;
  uint8_t cmd[2] = { BMP388_ODR, odr.reg };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 2, ticks_to_wait);
	//writeByte(BMP388_ODR, odr.reg);
}

void BMP388Sensor::setSeaLevelPressure(float pressure)								// Set the sea level pressure value
{
	sea_level_pressure = pressure;
}

uint8_t BMP388Sensor::getTemperature(volatile float &temperature)			// Get the temperature
{
	if (!dataReady())																									// Check if a measurement is ready
	{
		return 0;
	}
	uint8_t data[3];                                                  // Create a data buffer
  uint8_t cmd[1] = { BMP388_DATA_3 };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 1, ticks_to_wait);
	//readBytes(BMP388_DATA_3, &data[0], 3);       						      		// Read the temperature
  i2c_master_read_from_device(i2c_port, i2c_addr, (uint8_t *)&data[0], 3, ticks_to_wait);
	int32_t adcTemp = (int32_t)data[2] << 16 | (int32_t)data[1] << 8 | (int32_t)data[0];  // Copy the temperature data into the adc variables
	temperature = bmp388_compensate_temp((float)adcTemp);       			// Temperature compensation (function from BMP388 datasheet)
	return 1;
}

uint8_t BMP388Sensor::getPressure(volatile float &pressure)						// Get the pressure
{
	float temperature;
	return getTempPres(temperature, pressure, false);
}

uint8_t BMP388Sensor::getTempPres(volatile float &temperature, 				// Get the temperature and pressure
																volatile float &pressure, bool quick)	
{
	if (!quick && !dataReady())																									// Check if a measurement is ready
	{	
		return 0;
	}
	uint8_t data[6];                                                  // Create a data buffer
  uint8_t cmd[1] = { BMP388_DATA_0 };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 1, ticks_to_wait);
	//readBytes(BMP388_DATA_0, &data[0], 6);             	  						// Read the temperature and pressure data
  i2c_master_read_from_device(i2c_port, i2c_addr, (uint8_t *)&data[0], 6, ticks_to_wait);

	uint32_t adcTemp = (uint32_t)data[5] << 16 | (uint32_t)data[4] << 8 | (uint32_t)data[3];  // Copy the temperature and pressure data into the adc variables
	uint32_t adcPres = (uint32_t)data[2] << 16 | (uint32_t)data[1] << 8 | (uint32_t)data[0];
	temperature = bmp388_compensate_temp((float)adcTemp);   					// Temperature compensation (function from BMP388 datasheet)
	pressure = bmp388_compensate_press((float)adcPres, temperature); 	// Pressure compensation (function from BMP388 datasheet)
	pressure /= 100.0f;                         											// Calculate the pressure in millibar/hPa
	return 1;
}

uint8_t BMP388Sensor::getAltitude(volatile float &altitude)						// Get the altitude
{
	float temperature, pressure;
	return getMeasurements(temperature, pressure, altitude);
}

uint8_t BMP388Sensor::getMeasurements(volatile float &temperature, 		// Get all measurements temperature, pressure and altitude
																		volatile float &pressure, 
																		volatile float &altitude)		
{  
	if (getTempPres(temperature, pressure, false))
	{
		altitude = ((float)powf(sea_level_pressure / pressure, 0.190223f) - 1.0f) * (temperature + 273.15f) / 0.0065f; // Calculate the altitude in metres 
		return 1;
	}
	return 0;
}

void BMP388Sensor::enableInterrupt(OutputDrive outputDrive, 					// Enable the BMP388's data ready interrupt on the INT pin
																 ActiveLevel activeLevel,
																 LatchConfig latchConfig)
{
	int_ctrl.reg = latchConfig << 2 | activeLevel << 1 | outputDrive;
	int_ctrl.bit.drdy_en = 1;
  uint8_t cmd[2] = { BMP388_INT_CTRL, int_ctrl.reg };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 2, ticks_to_wait);
	//writeByte(BMP388_INT_CTRL, int_ctrl.reg);
}

void BMP388Sensor::disableInterrupt()																	// Disable the data ready interrupt on the INT pin
{
	int_ctrl.bit.drdy_en = 0;
  uint8_t cmd[2] = { BMP388_INT_CTRL, int_ctrl.reg };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 2, ticks_to_wait);
	//writeByte(BMP388_INT_CTRL, int_ctrl.reg);
}

void BMP388Sensor::setIntOutputDrive(OutputDrive outputDrive)					// Set the interrupt output drive: PUSH_PULL or OPEN_COLLECTOR
{
	int_ctrl.bit.int_od = outputDrive;
  uint8_t cmd[2] = { BMP388_INT_CTRL, int_ctrl.reg };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 2, ticks_to_wait);
	//writeByte(BMP388_INT_CTRL, int_ctrl.reg);
}

void BMP388Sensor::setIntActiveLevel(ActiveLevel activeLevel)					// Set the interrupt active level: LOW or HIGH
{
	int_ctrl.bit.int_level = activeLevel;
  uint8_t cmd[2] = { BMP388_INT_CTRL, int_ctrl.reg };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 2, ticks_to_wait);
	//writeByte(BMP388_INT_CTRL, int_ctrl.reg);
}

void BMP388Sensor::setIntLatchConfig(LatchConfig latchConfig)					// Set the interrupt latch configuration
{
	int_ctrl.bit.int_latch = latchConfig;
  uint8_t cmd[2] = { BMP388_INT_CTRL, int_ctrl.reg };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 2, ticks_to_wait);
	//writeByte(BMP388_INT_CTRL, int_ctrl.reg);
}

void BMP388Sensor::enableFIFO(PressEnable pressEnable,								// Enable the FIFO
														AltEnable altEnable,
														TimeEnable timeEnable,
														Subsampling subsampling,
														DataSelect dataSelect,
														StopOnFull stopOnFull)
{
	alt_enable = altEnable;
	fifo_config_1.reg = 1 << 4 | pressEnable << 3 | timeEnable << 2 | stopOnFull << 1;	
	fifo_config_1.bit.fifo_mode = 1;	
	fifo_config_2.reg = dataSelect << 3 | subsampling;
  uint8_t cmd[4] = { BMP388_FIFO_CONFIG_1, fifo_config_1.reg, BMP388_FIFO_CONFIG_2, fifo_config_1.reg };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 4, ticks_to_wait);
	//writeByte(BMP388_FIFO_CONFIG_1, fifo_config_1.reg);
	//writeByte(BMP388_FIFO_CONFIG_2, fifo_config_2.reg);
}

void BMP388Sensor::disableFIFO()																			// Disable the FIFO
{
	fifo_config_1.bit.fifo_mode = 0;
  uint8_t cmd[2] = { BMP388_FIFO_CONFIG_1, fifo_config_1.reg };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 2, ticks_to_wait);
	//writeByte(BMP388_FIFO_CONFIG_1, fifo_config_1.reg);
}

uint8_t BMP388Sensor::setFIFONoOfMeasurements(uint16_t noOfMeasurements) 		// Set the FIFO watermark from the number of measurements								
{																		
	// Calculate the FIFO watermark from the number of measurements
	uint16_t fifoWatermark = noOfMeasurements * ((fifo_config_1.bit.fifo_press_en | fifo_config_1.bit.fifo_temp_en) + 		
													 3 * fifo_config_1.bit.fifo_press_en + 3 * fifo_config_1.bit.fifo_temp_en);
	return setFIFOWatermark(fifoWatermark);
}

uint8_t BMP388Sensor::setFIFOWatermark(uint16_t fifoWatermark)				// Set the FIFO watermark
{
	if (fifoWatermark + fifo_config_1.bit.fifo_time_en +  						// Check if the FIFO watermark + appended sensor time (4 bytes) 
			3 * fifo_config_1.bit.fifo_time_en + 2 > FIFO_SIZE)						// if enabled + 2 byte overhead is larger than the FIFO size of 512 bytes
	{
		return 0;																												// FIFO watermark larger than FIFO, return error
	}
	uint8_t fifoWatermarkLSB = (uint8_t)(fifoWatermark & 0xFF);
	uint8_t fifoWatermarkMSB = (uint8_t)(fifoWatermark >> 8 & 0x01);
  uint8_t cmd[4] = { BMP388_FIFO_WTM_0, fifoWatermarkLSB, BMP388_FIFO_WTM_1, fifoWatermarkMSB };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 4, ticks_to_wait);
	//writeByte(BMP388_FIFO_WTM_0, fifoWatermarkLSB);
	//writeByte(BMP388_FIFO_WTM_1, fifoWatermarkMSB);
	return 1;
}

uint16_t BMP388Sensor::getFIFOWatermark()															// Retrieve the FIFO watermark
{
	uint16_t fifoWatermark;
  uint8_t cmd[1] = { BMP388_FIFO_WTM_0 };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 1, ticks_to_wait);
  i2c_master_read_from_device(i2c_port, i2c_addr, (uint8_t*)&fifoWatermark, sizeof(fifoWatermark), I2C_NO_TIMEOUT);
	//readBytes(BMP388_FIFO_WTM_0, (uint8_t*)&fifoWatermark, sizeof(fifoWatermark));
	return fifoWatermark;
}

void BMP388Sensor::setFIFOPressEnable(PressEnable pressEnable)				// Enable pressure measurements
{
	fifo_config_1.bit.fifo_press_en = pressEnable;
  uint8_t cmd[2] = { BMP388_FIFO_CONFIG_1, fifo_config_1.reg };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 2, ticks_to_wait);
	//writeByte(BMP388_FIFO_CONFIG_1, fifo_config_1.reg);
}

void BMP388Sensor::setFIFOTimeEnable(TimeEnable timeEnable)						// Enable sensor time
{
	fifo_config_1.bit.fifo_time_en = timeEnable;
  uint8_t cmd[2] = { BMP388_FIFO_CONFIG_1, fifo_config_1.reg };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 2, ticks_to_wait);
	//writeByte(BMP388_FIFO_CONFIG_1, fifo_config_1.reg);
}

void BMP388Sensor::setFIFOSubsampling(Subsampling subsampling)				// Set the FIFO sub-sampling rate
{
	fifo_config_2.bit.fifo_subsampling = subsampling;
  uint8_t cmd[2] = { BMP388_FIFO_CONFIG_2, fifo_config_2.reg };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 2, ticks_to_wait);
	//writeByte(BMP388_FIFO_CONFIG_2, fifo_config_2.reg);
}

void BMP388Sensor::setFIFODataSelect(DataSelect dataSelect)						// Set if the FIFO data is unfiltered or filtered
{
	fifo_config_2.bit.data_select = dataSelect;
  uint8_t cmd[2] = { BMP388_FIFO_CONFIG_2, fifo_config_2.reg };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 2, ticks_to_wait);
	//writeByte(BMP388_FIFO_CONFIG_2, fifo_config_2.reg);
}

void BMP388Sensor::setFIFOStopOnFull(StopOnFull stopOnFull)						// Set if to stop reading the FIFO when full or overwrite data
{
	fifo_config_1.bit.fifo_stop_on_full = stopOnFull;
  uint8_t cmd[2] = { BMP388_FIFO_CONFIG_1, fifo_config_1.reg };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 2, ticks_to_wait);
	//writeByte(BMP388_FIFO_CONFIG_1, fifo_config_1.reg);
}

uint16_t BMP388Sensor::getFIFOLength()																// Get the FIFO length
{
	uint16_t fifoLength; 
  uint8_t cmd[1] = { BMP388_FIFO_LENGTH_0 };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 1, ticks_to_wait);
  i2c_master_read_from_device(i2c_port, i2c_addr, (uint8_t*)&fifoLength, sizeof(fifoLength), I2C_NO_TIMEOUT);
	//readBytes(BMP388_FIFO_LENGTH_0, (uint8_t*)&fifoLength, sizeof(fifoLength));
	return fifoLength;
}

FIFOStatus BMP388Sensor::getFIFOData(volatile float *temperature, volatile float *pressure, 		// Get FIFO data
																	 volatile float *altitude, volatile uint32_t &sensorTime)	
{
	if (!fifoReady())																									// Check if the FIFO data is ready
	{
		return DATA_PENDING;																						// Return if the measurements are pending
	}
	bool configError = false;																					// Set the configuration error flag
	uint16_t fifoLength = getFIFOLength() + fifo_config_1.bit.fifo_time_en + 		// Get the FIFO length plus sensor time bits if required
												3 * fifo_config_1.bit.fifo_time_en;
	uint16_t count = 0;																								// Initialise the byte count
	uint16_t measCount = 0;																						// Initialise the measurement count
	uint8_t packetSize = (fifo_config_1.bit.fifo_press_en | fifo_config_1.bit.fifo_temp_en) + 		// Determine the size of the data packets		
												3 * fifo_config_1.bit.fifo_press_en + 3 * fifo_config_1.bit.fifo_temp_en;
	uint8_t data[MAX_PACKET_SIZE];																		// Declare data packet memory
  uint8_t cmd[1] = { BMP388_FIFO_DATA };

	while (count < fifoLength)																				// Parse the data until the FIFO is empty
	{
    esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 1, ticks_to_wait);
    i2c_master_read_from_device(i2c_port, i2c_addr, (uint8_t *)&data[0], packetSize, I2C_NO_TIMEOUT);
		//readBytes(BMP388_FIFO_DATA, &data[0], packetSize);							// Acquire the next data packet
		uint8_t header = data[0];																				// Acquire the data header
		int32_t adcTemp, adcPres;																				// Declare the raw temperature and pressure variables
		switch(header)
		{
			case FIFO_SENSOR_PRESS:																				// Header indicates that temperature and pressure data follows
				adcTemp = (int32_t)data[3] << 16 | (int32_t)data[2] << 8 | (int32_t)data[1];  // Copy the temperature and pressure data into the adc variables
				temperature[measCount] = bmp388_compensate_temp((float)adcTemp);   					// Temperature compensation (function from BMP388 datasheet)
				adcPres = (int32_t)data[6] << 16 | (int32_t)data[5] << 8 | (int32_t)data[4];
				pressure[measCount] = bmp388_compensate_press((float)adcPres, temperature[measCount]); 	// Pressure compensation (function from BMP388 datasheet)
				pressure[measCount] /= 100.0f;
				if (alt_enable)																							// Check if altitude measurements have been enabled
				{
					altitude[measCount] = ((float)powf(sea_level_pressure / pressure[measCount], 0.190223f) - 1.0f) * 	// Calculate the altitude in metres 
						(temperature[measCount]  + 273.15f) / 0.0065f; 
				}
				count += 6;																									// Increment the byte count by the size of the data payload
				measCount++;																								// Increment the measurement count
				break;
			case FIFO_SENSOR_TEMP:																				// Header indicates that temperature data follows
				adcTemp = (int32_t)data[3] << 16 | (int32_t)data[2] << 8 | (int32_t)data[1];  // Copy the temperature and pressure data into the adc variables
				temperature[measCount] = bmp388_compensate_temp((float)adcTemp);   					// Temperature compensation (function from BMP388 datasheet)
				count += 3;																									// Increment the byte count by the size of the data payload
				measCount++;																								// Increment the measurement count
				break;
			case FIFO_SENSOR_TIME:																				// Header indicates that sensor time follows
				// Sensor time isn't actually stored in the FIFO, but is appended once the FIFO is read
				sensorTime = (uint32_t)data[3] << 16 | (uint32_t)data[1] << 8 | (uint32_t)data[1];	// Read the sensor time
				count += 3;																									// Increment the byte count by the size of the data payload
				break;
			case FIFO_CONFIG_CHANGE:																			// Header indicates that configuration change or FIFO empty data follows
			case FIFO_EMPTY:
				count++;																										// Increment the byte count
				break;
			case FIFO_CONFIG_ERROR:
				configError = true;																					// Set the configuration error flag
				count++;																										// Increment the byte count
				break;
			default:
				break;
		}
	}
	return configError ? CONFIG_ERROR : DATA_READY;										// Indicate that the measurements are ready, (or if a config error occured)
}

void BMP388Sensor::enableFIFOInterrupt(OutputDrive outputDrive, 			// Enable the BMP388's FIFO interrupts on the INT pin
																		 ActiveLevel activeLevel,
																		 LatchConfig latchConfig)
{
	int_ctrl.reg = latchConfig << 2 | activeLevel << 1 | outputDrive;
	int_ctrl.bit.fwtm_en = 1;
	int_ctrl.bit.ffull_en = 1;
  uint8_t cmd[2] = { BMP388_INT_CTRL, int_ctrl.reg };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 2, ticks_to_wait);
	//writeByte(BMP388_INT_CTRL, int_ctrl.reg);																 
}

void BMP388Sensor::disableFIFOInterrupt()															// Disable the FIFO interrupts on the INT pin
{
	int_ctrl.bit.fwtm_en = 0;
	int_ctrl.bit.ffull_en = 0;
  uint8_t cmd[2] = { BMP388_INT_CTRL, int_ctrl.reg };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 2, ticks_to_wait);
	//writeByte(BMP388_INT_CTRL, int_ctrl.reg);
}

void BMP388Sensor::flushFIFO()																				// Flush the FIFO
{
  uint8_t cmd[2] = { BMP388_CMD, FIFO_FLUSH };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 2, ticks_to_wait);
	//writeByte(BMP388_CMD, FIFO_FLUSH);
}

uint32_t BMP388Sensor::getSensorTime()																// Get the sensor time
{
	uint32_t sensorTime;
  uint8_t cmd[1] = { BMP388_SENSORTIME_0 };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 1, ticks_to_wait);
  i2c_master_read_from_device(i2c_port, i2c_addr, (uint8_t *)&sensorTime, sizeof(sensorTime - 1), I2C_NO_TIMEOUT);
	//readBytes(BMP388_SENSORTIME_0, (uint8_t*)&sensorTime, sizeof(sensorTime - 1));
	return sensorTime & 0x00FFFFFF;
}

void BMP388Sensor::enableI2CWatchdog()																// Enable the I2C watchdog
{
	if_conf.bit.i2c_wdt_en = 1;
  uint8_t cmd[2] = { BMP388_IF_CONFIG, if_conf.reg };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 2, ticks_to_wait);
	//writeByte(BMP388_IF_CONFIG, if_conf.reg);
}

void BMP388Sensor::disableI2CWatchdog()																// Disable the I2C watchdog
{
	if_conf.bit.i2c_wdt_en = 0;
  uint8_t cmd[2] = { BMP388_IF_CONFIG, if_conf.reg };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 2, ticks_to_wait);
	//writeByte(BMP388_IF_CONFIG, if_conf.reg);
}

void BMP388Sensor::setI2CWatchdogTimout(WatchdogTimout watchdogTimeout)		// Set the I2C watchdog time-out: 1.25ms or 40ms
{
	if_conf.bit.i2c_wdt_sel = watchdogTimeout;
  uint8_t cmd[2] = { BMP388_IF_CONFIG, if_conf.reg };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 2, ticks_to_wait);
	//writeByte(BMP388_IF_CONFIG, if_conf.reg);
}

uint8_t BMP388Sensor::getErrorReg()																		// Read the error register
{
  return 0; // TODO: Implement
	//return readByte(BMP388_ERR_REG);
}

uint8_t BMP388Sensor::getStatusReg()																	// Read the status register
{
  return 0; // TODO: Implement
	//return readByte(BMP388_STATUS);
}

////////////////////////////////////////////////////////////////////////////////
// BMP388Sensor Private Member Functions
////////////////////////////////////////////////////////////////////////////////

void BMP388Sensor::setMode(Mode mode)																	// Set the BMP388's mode in the power control register
{
	pwr_ctrl.bit.mode = mode;
  uint8_t cmd[2] = { BMP388_PWR_CTRL, pwr_ctrl.reg };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 2, ticks_to_wait);
  if(err != ESP_OK)
    ESP_LOGE(TAG_BMPSENSOR, "Mode change failed (%s)", esp_err_to_name(err));
	//writeByte(BMP388_PWR_CTRL, pwr_ctrl.reg);
}

void BMP388Sensor::setOversamplingRegister(Oversampling presOversampling,  // Set the BMP388 oversampling register 
																				 Oversampling tempOversampling)
{
	osr.reg = tempOversampling << 3 | presOversampling;
  uint8_t cmd[2] = { BMP388_OSR, osr.reg };
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 2, ticks_to_wait);
	//writeByte(BMP388_OSR, osr.reg);                              
}

uint8_t BMP388Sensor::dataReady()																			// Check if measurement data is ready
{		
  uint8_t cmd[1] = { BMP388_INT_STATUS };
	if (pwr_ctrl.bit.mode == SLEEP_MODE)															// If we're in SLEEP_MODE return immediately
	{
		return 0;
	}
  esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd, 1, ticks_to_wait);
  i2c_master_read_from_device(i2c_port, i2c_addr, (uint8_t *)&int_status.reg, 1, ticks_to_wait);
	//int_status.reg = readByte(BMP388_INT_STATUS);											// Read the interrupt status register
	if (int_status.bit.drdy)																					// Check if the data ready flag has been cleared
	{
		if (pwr_ctrl.bit.mode == FORCED_MODE)					 									// If we're in FORCED_MODE switch back to SLEEP_MODE
		{		
			pwr_ctrl.bit.mode = SLEEP_MODE;	
		}
		return 1;																												// The measurement is ready
	}
	return 0;																													// The measurement is still pending
}

uint8_t BMP388Sensor::fifoReady()																			// Check if the FIFO data is ready
{		
	return dataReady() && int_status.bit.fwm_int ? 1 : 0;							// Return 1 if a FIFO measurement is ready, otherwise 0
}

////////////////////////////////////////////////////////////////////////////////
// Bosch BMP388Sensor (Private) Member Functions
////////////////////////////////////////////////////////////////////////////////

void BMP388Sensor::setI2CAddress(uint8_t addr)
{
  i2c_addr = addr;
}

float BMP388Sensor::bmp388_compensate_temp(float uncomp_temp)
{
	float partial_data1 = uncomp_temp - floatParams.param_T1;
	float partial_data2 = partial_data1 * floatParams.param_T2;
	return (float)(partial_data2 + partial_data1 * partial_data1 * floatParams.param_T3);
}

float BMP388Sensor::bmp388_compensate_press(float uncomp_press, float t_lin)
{
	float partial_data1 = floatParams.param_P6 * t_lin;
	float partial_data2 = floatParams.param_P7 * t_lin * t_lin;
	float partial_data3 = floatParams.param_P8 * t_lin * t_lin * t_lin;
	float partial_out1 = floatParams.param_P5 + partial_data1 + partial_data2 + partial_data3;
	partial_data1 = floatParams.param_P2 * t_lin;
	partial_data2 = floatParams.param_P3 * t_lin * t_lin;
	partial_data3 = floatParams.param_P4 * t_lin * t_lin * t_lin;
	float partial_out2 = uncomp_press * (floatParams.param_P1 +
		partial_data1 + partial_data2 + partial_data3);
	partial_data1 = uncomp_press * uncomp_press;
	partial_data2 = floatParams.param_P9 + floatParams.param_P10 * t_lin;
	partial_data3 = partial_data1 * partial_data2;
	float partial_data4 = partial_data3 + uncomp_press * uncomp_press * uncomp_press * floatParams.param_P11;
	return partial_out1 + partial_out2 + partial_data4;
}
