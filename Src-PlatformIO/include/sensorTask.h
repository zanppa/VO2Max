/*
 * Copyright (C) 2024 Lauri Peltonen
 * GPL V3
 */

#ifndef __SENSORTASK_H__
#define __SENSORTASK_H__

#include <freertos/FreeRTOS.h>

#define I2C_PORT      (I2C_NUM_0)
#define PIN_SDA       21
#define PIN_SCL       22

// Enable this if temperature measurement from the differential pressure sensor
// should be used as exhale temperature instead of default 35 C.
// The sensor seems to read very low temperatures, around 24 C so enabling this probably
// adds more error than if constant 35 C was assumed
// #define SENSOR_UPDATE_EXHALE_TEMP

// Event group bits to indicate sensor status etc.
#define SENSOR_HAS_FLOW     0x0001
#define SENSOR_HAS_O2       0x0002
#define SENSOR_HAS_CO2      0x0004
#define SENSOR_HAS_PRESSURE 0x0008
#define SENSOR_VO2_VALID    0x0020      // Cleared wherever the data is read
#define SENSOR_VCO2_VALID   0x0040      // Cleared wherever the data is used
#define SENSOR_INIT_DONE    0x0080      // Sensor initialization is done



// Events for example to send data over wifi
#define SENSOR_EVENT_FLOW   0x0001      // Triggered on every flow measurement event
#define SENSOR_EVENT_CALC   0x0002      // Triggered on every calculation step
#define SENSOR_EVENT_AVE    0x0004      // Triggered when averaging time has passed
#define SENSOR_EVENT_VOL    0x0008      // Triggered after volume calc step, even if nothing was calculated
#define SENSOR_EVENT_INIT   0x1000      // Set initial oxygen level (triggered from outside)
#define SENSOR_EVENT_RESET  0x2000      // Reset calculation values (vo2Max and such)
#define SENSOR_EVENT_O2_CAL 0x4000      // Request to calibrate O2 sensor
#define SENSOR_EVENT_RECONF 0x8000      // Request to copy settings from global settings

#define SENSOR_EXTRA_HR     0x01        // Heart rate
#define SENSOR_EXTRA_RR     0x02        // Heart rate R-R variation

typedef struct _sensorExtraData_t {
  uint8_t type;
  float value;
} sensorExtraData_t;

typedef struct _sensorData_t {
  float vo2;      // Latest volume of o2 consumed in one minute [ml/min/kg]
  float vo2Max;   // Abs. maximum of previous [ml/min/kg]
  float ve;       // Latest exhaust air volume (single breath) [l]
  float veMax;    // Maximum of single breath exhaust volume [l]
  float veMean;   // One minute average of exhaust air volume (minute ventilation) [l/min/kg]
  float vco2;     // Latest volume of co2 consumed in one minute [ml/min/kg]??
  float vco2Max;  // Abs. max of previous [ml/min/kg]??
  float rq;       // Respiratory quotient, Vco2/Vo2 (also respiratory exchange ratio, RER)
  float resp_rate;  // Respiratory rate, breaths per minute
  float ambient_pressure; // Ambient air pressure [hPa]
  float ambient_temperature;  // Ambient air temperature [degC]
  float exhale_temperature;   // Exhale temperature from pressure sensor
  float o2;       // O2 sensor reading (raw value) [%]
  float co2;      // CO2 sensor reading (raw value) [%]
  float flow_value;   // Instant value from the flow sensor (pressure) [Pa]

// Extra sensors (outside the sensorTask)
  float hr;           // Heart rate
  float hr_max;       // Maximum of heart rate
  float rr;           // Heart rate r-r variation

// Debug variables
  uint16_t errors;    // Error counter
} sensorData_t;


#define STORE_BUFFER_SIZE   1024      // How many data samples to store

// Structure of data to buffer
typedef struct _storeData_t {
  float vo2;    // Vo2, ml/min/kg, mean value
  float ve;     // Ve, l/min, mean value
  float vco2;   // Vco2, ml/min/kg, mean value
  float resp_rate;  // Breaths per minute 1/min
  float hr;     // Heart rate, 1/min, latest value
  float temperature;   // Ambient temperature [degC]
  float pressure; // Ambient pressure [hPa]
  float o2;     // Raw O2 sensor reading [%]
} storeData_t;




void sensorTask(void *params);
uint16_t sensorGetStatus();

bool sensorGetData(sensorData_t *copy_to);

void sensorClearEvent(uint32_t event);
bool sensorWaitEvent(uint32_t event, TickType_t timeout);

bool sensorAddExtraValue(uint8_t type, float value, TickType_t timeout);

void sensorSetInitial(void);
void sensorResetCalculation(void);
void sensorO2Calibrate(void);
void sensorSetConfiguration(void);

bool sensorHasErrors(void);

const storeData_t *getStorageBuffer(void);
unsigned int getStorageBufferPosition(void);

#endif