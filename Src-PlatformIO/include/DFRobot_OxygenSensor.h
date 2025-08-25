/*!
 * @file DFRobot_OxygenSensor.h
 * @brief Define basic struct of DFRobot_OxygenSensor class
 * @details This is an electrochemical oxygen sensor, I2C address can be changed by a dip switch, and the oxygen concentration can be obtained through I2C.
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author ZhixinLiu(zhixin.liu@dfrobot.com)
 * @version V1.0.1
 * @date 2023-08-02
 * @url https://github.com/DFRobot/DFRobot_OxygenSensor
 */

// Modified by L. Peltonen, 2/2024
// From wire to direct i2c

#ifndef __DFRobot_OxygenSensor_H__
#define __DFRobot_OxygenSensor_H__

//#include <Arduino.h>

#define O2SENSOR_ADDRESS_0   0x70
#define O2SENSOR_ADDRESS_1   0x71
#define O2SENSOR_ADDRESS_2   0x72
#define O2SENSOR_ADDRESS_3   0x73  ///< iic slave Address select
#define O2SENSOR_OCOUNT      100   ///< oxygen Count Value
#define OXYGEN_DATA_REGISTER 0x03   ///< register for oxygen data
#define USER_SET_REGISTER    0x08   ///< register for users to configure key value manually
#define AUTUAL_SET_REGISTER  0x09   ///< register that automatically configure key value
#define GET_KEY_REGISTER     0x0A   ///< register for obtaining key value

class O2Sensor
{
public:
  /**
   * @fn O2Sensor
   * @brief Constructor
   * @param port i2c port, default 0
   * @param addr i2c device address
   * @n     Default to use i2c address of 0x73 without passing parameters
   * @return None
   */
  O2Sensor(uint8_t addr = O2SENSOR_ADDRESS_3, i2c_port_t port = I2C_NUM_0);
  ~O2Sensor();
  /**
   * @fn begin
   * @brief Check if sensor responds, I2c must already be initialized
   * @return ESP_OK if everything ok, otherwise esp error code
   */
  esp_err_t begin();

  /**
   * @fn calibrate
   * @brief Calibrate oxygen sensor
   * @param vol oxygen concentration unit vol
   * @param mv calibrated voltage unit mv, (keep as zero?)
   * @return ESP_OK if communication ok, otherwise esp error code
   */
  esp_err_t calibrate(float vol, float mv = 0);

  /**
   * @fn triggerSampling
   * @brief Trigger oxygen sensor sampling
   * @return ESP_OK if communication ok, otherwise esp error code
   */
  esp_err_t triggerSampling();

  /**
   * @fn readOxygenValue
   * @brief Read the oxygen value > 100 ms after triggering sampling
   * @param value Pointer to float where value will be stored
   * @return ESP_OK if communication ok, otherwise esp error code
   */
  esp_err_t readOxygenValue(float &value);

  /**
   * @fn readAverageOxygenValue
   * @brief Read the oxygen value > 100 ms after triggering sampling, average over multiple samples
   * @param value Pointer to float where value will be stored
   * @return ESP_OK if communication ok, otherwise esp error code
   */
  esp_err_t readAverageOxygenValue(float &value, uint8_t n_averages);

  /**
   * @fn getOxygenData
   * @brief Get oxygen concentration
   * @param collectNum The number of data to be smoothed
   * @n     For example, upload 20 and take the average value of the 20 data, then return the concentration data
   * @return Oxygen concentration, unit
   */  
  //float getOxygenData(uint8_t collectNum);
  
private:
  esp_err_t readFlash();
  uint8_t _addr;
  i2c_port_t _port;
  float _Key = 0.0;                          ///< oxygen key value
  float oxygenData[O2SENSOR_OCOUNT] = {0.00};        // Array for averaging
  uint8_t n_data_received;
  float getAverageNum(float bArray[], uint8_t len);
};
#endif