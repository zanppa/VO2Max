/*!
 * @file DFRobot_OxygenSensor.cpp
 * @brief Define the basic struct of DFRobot_OxygenSensor class, the implementation of basic method
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author ZhixinLiu(zhixin.liu@dfrobot.com)
 * @version V1.0.1
 * @date 2022-08-02
 * @url https://github.com/DFRobot/DFRobot_OxygenSensor
 */

// Modified by L. Peltonen, 2/2024
// - Rename class ot O2Sensor
// - From wire to direct i2c
// - Return esp errors

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esp_err.h"
#include "driver/i2c.h"
#include "esp_log.h"

#include "DFRobot_OxygenSensor.h"

static const TickType_t ticks_to_wait = pdMS_TO_TICKS(2); // Default timeout
static const TickType_t ticks_begin = pdMS_TO_TICKS(100);
const char *TAG_O2SENSOR = "O2sensor";

O2Sensor::O2Sensor(uint8_t addr, i2c_port_t port) : _addr{addr}, _port{port} { n_data_received = 0; }

O2Sensor::~O2Sensor() { }

esp_err_t O2Sensor::begin() 
{
  uint8_t cmd[1] = {OXYGEN_DATA_REGISTER};
  esp_err_t err = i2c_master_write_to_device(_port, _addr, cmd, 1, ticks_begin);

  if(err != ESP_OK) {
    ESP_LOGE(TAG_O2SENSOR, "O2Sensor::begin failed: %s", esp_err_to_name(err));
    return err;
  } else {
    ESP_LOGI(TAG_O2SENSOR, "O2Sensor initialized at addr 0x%x", _addr);
  }

  err = readFlash();  // Read calibration value from sensor memory

  return err;
}

esp_err_t O2Sensor::readFlash()
{
  uint8_t value = 0;
  uint8_t cmd[1] = {GET_KEY_REGISTER};
  esp_err_t err = i2c_master_write_to_device(_port, _addr, cmd, 1, ticks_to_wait);
  if(err != ESP_OK) return err;

  vTaskDelay(pdMS_TO_TICKS(50));  // Wait 50 ms

  err = i2c_master_read_from_device(_port, _addr, &value, 1, ticks_to_wait);
  if(err != ESP_OK) {
    ESP_LOGE(TAG_O2SENSOR, "O2Sensor::readFlash failed: %s", esp_err_to_name(err));
    return err;
  }

  if(value == 0){
    this->_Key = 20.9 / 120.0;
  }else{
    this->_Key = (float)value / 1000.0;
  }

  return ESP_OK;
}


esp_err_t O2Sensor::calibrate(float vol, float mv)
{
  uint8_t cmd[2] = {0, 0};

  uint8_t keyValue = vol * 10;
  if(mv < 0.000001 && mv > (-0.000001) ) {
    cmd[0] = USER_SET_REGISTER;
  }else {
    keyValue = (vol / mv) * 1000;
    cmd[0] = AUTUAL_SET_REGISTER;
  }
  cmd[1] = keyValue;
  esp_err_t err = i2c_master_write_to_device(_port, _addr, cmd, 2, ticks_to_wait);

  if(err != ESP_OK)
    ESP_LOGE(TAG_O2SENSOR, "O2Sensor::calibrate failed: %s", esp_err_to_name(err));

  return err;
}

esp_err_t O2Sensor::triggerSampling()
{
  uint8_t cmd[1] = { OXYGEN_DATA_REGISTER };
  esp_err_t err = i2c_master_write_to_device(_port, _addr, cmd, 1, ticks_to_wait);
  return err;
}

esp_err_t O2Sensor::readOxygenValue(float &value)
{
  uint8_t rxbuf[3] = {0};

  // Read 3 bytes from memory
  esp_err_t err = i2c_master_read_from_device(_port, _addr, rxbuf, 3, ticks_to_wait);
  if(err == ESP_OK)
    value = _Key * ( ((float)rxbuf[0]) + ((float)rxbuf[1] / 10.0) + ((float)rxbuf[2] / 100.0) );

  return err;
}

esp_err_t O2Sensor::readAverageOxygenValue(float &value, uint8_t n_averages)
{
  uint8_t rxbuf[3] = {0};

  if(n_averages >= O2SENSOR_OCOUNT) n_averages = O2SENSOR_OCOUNT-1;

  // Shuffle old values forwards
  for(uint8_t i = n_averages; i > 0; i--) oxygenData[i] = oxygenData[i-1];

  // Read 3 bytes from memory
  esp_err_t err = i2c_master_read_from_device(_port, _addr, rxbuf, 3, ticks_to_wait);
  oxygenData[0] = _Key * ( ((float)rxbuf[0]) + ((float)rxbuf[1] / 10.0) + ((float)rxbuf[2] / 100.0) );

  if(n_data_received < n_averages) n_data_received++;

  value = getAverageNum(oxygenData,n_data_received);

  return err;
}


/*
float O2Sensor::getOxygenData(uint8_t collectNum)
{
  uint8_t rxbuf[10]={0}, k = 0;
  static uint8_t i = 0, j = 0;
  readFlash();
  if(collectNum > 0){
    for(j = collectNum - 1;  j > 0; j--) {  oxygenData[j] = oxygenData[j-1]; }
    _pWire->beginTransmission(_addr);
    _pWire->write(OXYGEN_DATA_REGISTER);
    _pWire->endTransmission();
    delay(100);
    _pWire->requestFrom(_addr, (uint8_t)3);
      while (_pWire->available()){
        rxbuf[k++] = _pWire->read();
      }
    oxygenData[0] = ((_Key) * (((float)rxbuf[0]) + ((float)rxbuf[1] / 10.0) + ((float)rxbuf[2] / 100.0)));
    if(i < collectNum) i++;
    return getAverageNum(oxygenData, i);
  }else {
    return -1.0;
  }
}
*/

float O2Sensor::getAverageNum(float bArray[], uint8_t len)
{
  uint8_t i = 0;
  float bTemp = 0;    // Should be enough to just use float here, no need for double I think
  for(i = 0; i < len; i++) {
    bTemp += bArray[i];
  }
  return bTemp / (float)len;
}