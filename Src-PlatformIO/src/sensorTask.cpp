/*
 * Main sensor task for VO2 sensor
 * Reads flow with pressure sensor, oxygen concentration
 * and ambient pressure & temperature
 * and handles all integrations and calculations
 *
 *
 * Copyright (C) 2024 Lauri Peltonen
 * GPL V3
 *
 * Volume compensation equations from
 * https://sites.uni.edu/dolgener/Instrumentation/Repeating%20material/VO2%20Computation.pdf
 *
 * Venturi calculation for flow rate from https://en.wikipedia.org/wiki/Venturi_effect
 * Air density calculation from https://en.wikipedia.org/wiki/Density_of_air
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include "esp_log.h"

#include <Arduino.h> // DEBUG

#include <math.h>

#include "config.h"

#include "sensorTask.h"

#include "sdpsensor.h"
#include "bmp388sensor.h"
#include "DFRobot_OxygenSensor.h"

const char *TAG_SENSOR = "sensorTask";

#define I2C_DEFAULT_TIMEOUT   (320000)  // time = this * APB 80 MHz clock. 80 000 = 1 ms

esp_err_t initialize_I2C(i2c_port_t i2c_port=I2C_PORT, int pinSDA=21, int pinSCL=22);
esp_err_t reset_I2C();

// Sensor definitions
SDPSensor flow_sensor(0x26, I2C_PORT);    // SDP801-500Pa has address of 0x26
BMP388Sensor pressure_sensor(0x77, I2C_PORT); // BMP388 atmospheric pressure sensor (address selected 0x77, other setting is 0x76)
O2Sensor oxygen_sensor(0x73, I2C_PORT);   // DFRobot Sen0322 oxygen sensor (my was default 0x73 with both address pins "low")


// TODO: Move to config?
#define READINTERVAL    10    // ms, how often we run the sensor reading task
#define FLOWINTERVAL    (2*READINTERVAL)  // ms, how often we read & integrate the flow meter, should be below 25 ms
#define OXYGEN_INTERVAL 4     // After the oxygen sensor is triggered, then next Nth time it is read, because there needs to be 100 ms between trigger & read

// Time to integrate the oxygen consumption volume (1 minute)
static int INTEGRATION_TIME = 15000;  // Update every 15 seconds by default
#define NORMALIZATION_TIME  60000     // ms, normalize to this value, e.g. normalize to 60 seconds (e.g. veMean = liters / 1 minute)
static unsigned int STORE_RATE = 0;   // How often store the values to buffer, default=every integration step

static uint16_t errorCounter = 0;

// Struct where sensor data is be copied on command
sensorData_t sensorData;

// Structure where data is buffered
storeData_t bufferData[STORE_BUFFER_SIZE] = {0};
uint16_t bufferPosition = 0;

typedef enum {
  IDLE = 0,
  READ_FLOW,      // Read & integrate flow in the venturi tube
  READ_O2,        // Read exhaust O2 sensor
  READ_CO2,       // Read exhaust CO2 sensor
  READ_PRESSURE,  // Read ambient pressure (& temperature)
  CALC_AIRDENSITY,  // Calculate air density
  CALC_VOLUMES,      // Calculate volume
} sensorSequence_t;

// Here we define in what order the sensors are read
// First item is read between all other items also
static sensorSequence_t sequence[] = {
  READ_FLOW,          // At 400 kHz this takes currently 240 us
  READ_O2,
  READ_CO2,
  READ_PRESSURE,      // At 100 kHz currently takes 940 us. 400 kHz --> 310 us (calculation takes almost no time)
  CALC_AIRDENSITY,
  CALC_VOLUMES,
};

static const uint8_t sequenceSteps = sizeof(sequence) / sizeof(sensorSequence_t);

static const float calculationRate = ((2 * sequenceSteps) - 2) * READINTERVAL;  // How often (in ms) we calculate everything

// Previous pressure values used to detect when breathing out
static float prevPressure0 = 0.0;
static float prevPressure1 = 0.0;
static float prevPressure2 = 0.0;
static float currentPressure = 0.0;         // Current (latest) pressure reading
static const float pressureThreshold = 1.0; // Threshold when breathing out (Pa), to start or stop integration. Keep low for best accuracy.
static float integratedPressure = 0.0;      // Integrated value of pressure for one breath
static float integrationTotal = 0.0;        // Total integration over the INTEGRATION_TIME
static uint32_t integrationTime = 0;        // How many times we integrated the pressure --> time
static unsigned int storeIntervalTime = 0;  // Interval when to store values to buffer
static bool integratePressure = false;      // Whether we should integrate the pressure value or not
static bool newBreathData = false;          // Is there a new single breath analyzed
static uint32_t breathIntervalCount = 0;    // Interval between two breaths, on-going counter
static uint32_t lastBreathInterval = 0;     // Latest interval between two breaths (ms)

// The volumetric flow volume is, according to https://en.wikipedia.org/wiki/Venturi_effect
// Q = A1 * sqrt( (2/rho) * dp / ( (A1/A2)^2 - 1)) )
//   = A1 * sqrt(2/rho) * sqrt(dp / ( (A1/A2)^2 - 1) )
//   = A1 * sqrt(2/rho) * sqrt(1  / ( (A1/A2)^2 - 1)) * sqrt(dp)
//   = constant * sqrt(2/rho) * sqrt(dp)
// So here we can calculate the constant first, and then integrate the sqrt of the pressure differential
// and if we have an ambient pressure sensor, then the sqrt(2/rho) may change also

// Constants for the venturi tube calculation
static const float venturiDiameter1 = 0.026;  // meters
static const float venturiDiameter2 = 0.019;  // meters, smaller of the two diameters
static const float venturiArea1 = 3.14159 * venturiDiameter1 * venturiDiameter1 * 0.25; // pre-calc
static const float venturiArea2 = 3.14159 * venturiDiameter2 * venturiDiameter2 * 0.25;// pre-calc
static const float venturiConstant = venturiArea1 / sqrt((venturiArea1 / venturiArea2) * (venturiArea1 / venturiArea2) - 1.0 );
//static const float venturiConstant = venturiArea2 / sqrt(1.0 - ((venturiDiameter2 / venturiDiameter1) * (venturiDiameter2 / venturiDiameter1)) );
static float flowCorrectionFactor = 1.0;    // Correction factor for flow measurement


// This is for exhaling air volume calculation --> actual rho of exhale air
static float inverseRhoCoeff = sqrt(2.0 / 1.123);   // 1.123 kg/m^3 is the default air density at 35°C and 95 % humidity

// Coefficients of moist air water vapor pressure in hPA (as the pressure sensor returns result in hPA)
// for measurement temperatures from 20C to 40C (endpoints included)
static float moistVaporPressure[21] = {23.33, 24.93, 26.40, 28.13, 29.86, 31.73, 33.60, 35.60, 37.86, 40.00, 42.40, 44.93, 47.60, 50.26, 53.20, 56.26, 59.46, 62.79, 66.26, 69.86, 73.73};

static float VATPStoSTDP = 1.0; // Coefficient to convert ATPS volume (measurement conditions) to STDP (standard conditions)

static float exhale_temperature = 35.0; // Default exhale temperature assumed to be 35C

// Ambient air temperature (degC) and pressure (hPa)
static volatile float ambient_pressure = 0.0;
static volatile float ambient_temperature = 0.0;

// O2 concentration
float initialO2 = 20.93;     // Initial O2 level to be used in VO2 measurement, about constant 20.93 % in atmosphere
static float currentO2 = initialO2;      // Latest O2 measurement result
static float depletedO2;    // How much O2 was depleted from exhaust air

static bool useCO2Sensor = false;    // Use or do not use CO2 sensor if it is available, default is not
float initialCO2 = 0.03;    // Initial CO2 level, about constant 0.03 % in atmosphere
static float currentCO2 = initialCO2;   // Latest CO2 measurement result
static float producedCO2 = 0.0;   // How much CO2 was produced in exhalation (%)

static float lastBreathVolume = 0.0;  // liters
static float veMax = 0.0; // liters
static float veMean = 0.0;  // liters/minute
static bool newVolumeData = false;

static float lastVolumeO2 = 0.0; // in liters
static float averageO2level = 0.0;
static float averageCO2level = 0.0;
static uint32_t averagingTime = 0;

static float vo2 = 0.0;    // Integral over predefined time (e.g. 15s or minute), liters/minute
static float vo2Max = 0.0;  // Maximum of previous, liters/minute

static float weight;      // User's weight

static float vco2 = 0.0;    // Integral over predefined time (e.g. 15s or minute), liters/minute
static float vco2Max = 0.0; // Maximum of previous, liters/minute


// Variables for controlling the task flow
static TickType_t xLastWakeTime;
static const TickType_t xFrequency = pdMS_TO_TICKS(READINTERVAL);
static BaseType_t xWasDelayed;

// Current step in the sequence that we're taking
static uint8_t action = 0;
bool read_first = false;

// Event group holding the status bits
EventGroupHandle_t sensorStatus = nullptr;

// Evet group for triggering events on other tasks
EventGroupHandle_t sensorEvent = nullptr;

// Queue to handle the latest data after every sensor task step
QueueHandle_t sensorQueue = nullptr;

// Queue to update extra data to the structure
static QueueHandle_t sensorExtraQueue = nullptr;
static sensorExtraData_t extraDataBuf;

// Counter for low-pass filter signals, to allow stabilisation before
// the filter is taken into use
static uint8_t initialize_counter = 2000 / calculationRate;  // Roughly 2 seconds
static const float low_pass_gain = calculationRate / (3000 + calculationRate);    // 3000 ms time constant

// Helper function to calculate simple exponentially weighted average low-pass
// gain has to be calculated depending on the time step (dt)
// so that gain = dt / (tau + dt)
float inline low_pass_filter(float new_value, float old_value, float gain)
{
  return gain * new_value + (1.0 - gain) * old_value;
}

// Helper function to read the airflow (pressure) value
// and integrate the result
void inline read_flow(void)
{
  float new_temperature;

  // Move the previous pressure values around
  prevPressure0 = prevPressure1;
  prevPressure1 = prevPressure2;
  prevPressure2 = currentPressure;


  integrationTime += FLOWINTERVAL;      // For total integration (average over 1 minute for example)
  breathIntervalCount += FLOWINTERVAL;  // Measure the time between two exhales

  // Read the sensor
  if(flow_sensor.readDiffPressureTemperature(currentPressure, new_temperature) != ESP_OK)
  {
    if(errorCounter < 0xFFFF)
      errorCounter++;
      return;
  }

#ifdef SENSOR_UPDATE_EXHALE_TEMP
  // Low pass filter the temperature
  if(!initialize_counter) {
    exhale_temperature = low_pass_filter(new_temperature, exhale_temperature, 0.0066);
  } else {
    exhale_temperature = new_temperature;
  }
#endif

  if(currentPressure < 0.0) currentPressure = 0.0;

  if(integratePressure)
  {
    // We are currently exhaling

    // Integrate pressure value
    integratedPressure += sqrt(currentPressure);
    integrationTotal += sqrt(currentPressure);

    // Detect when to stop --> 3 latest ones are below threshold
    if(currentPressure <= pressureThreshold && prevPressure2 <= pressureThreshold && prevPressure1 <= pressureThreshold)
    {
      newBreathData = true; // Now we can do the exhaust volume calculation
      integratePressure = false;  // Stop integration until next breath out
    }
  }

  else if(currentPressure >= pressureThreshold && prevPressure2 >= pressureThreshold && prevPressure1 >= pressureThreshold)
  {
    // We were not integrating but now last samples were above threshold --> start integration
    integratedPressure = sqrt(currentPressure) + sqrt(prevPressure2) + sqrt(prevPressure1) + sqrt(prevPressure0);
    integrationTotal += integratedPressure;

    lastBreathInterval = breathIntervalCount; // Store the latest time
    breathIntervalCount = 0;  // Start counting between two exhales here again

    integratePressure = true;
    newBreathData = false;
  }

  // TODO: Needs a timeout of some kind for integration?

  xEventGroupSetBits(sensorEvent, SENSOR_EVENT_FLOW);
}


// Read ambient air pressure and temperature
void inline read_pressure()
{
  float new_temperature, new_pressure;
  if(!pressure_sensor.getTempPres(new_temperature, new_pressure, true))
  {
    if(errorCounter < 0xFFFF)
      errorCounter++;
      return;
  }
  if(!initialize_counter) {
    ambient_pressure = low_pass_filter(new_pressure, ambient_pressure, low_pass_gain);
    ambient_temperature = low_pass_filter(new_temperature, ambient_temperature, low_pass_gain);
  } else {
    ambient_pressure = new_pressure;
    ambient_temperature = new_temperature;
  }
}


static uint8_t o2_current_interval = 0;

// Read O2 sensor
// This is handled in two parts due to not knowing how the sensor works
// but this method seems to work properly.
// 1st write the register address to be read (called as "trigger")
// 2nd after ~100 ms or so, read the values from the register
void read_o2()
{
  if(!o2_current_interval) {
    oxygen_sensor.triggerSampling();
    o2_current_interval = OXYGEN_INTERVAL;
  } else {
    if(o2_current_interval == 1) {
      oxygen_sensor.readOxygenValue(currentO2);

      // Compensate for sensor drift, basically this should never happen...
      if(currentO2 > initialO2) initialO2 = currentO2;  // TODO: Not sure if this makes sense here
    }
  }
  o2_current_interval--;
}

// Read CO2 sensor
// Currently no sensor is supported so this would need to be implemented later
// TODO: Implement CO2 sensor reading!
void read_co2()
{
  currentCO2 = 0.03;    // TODO: Here we should read the sensor
  producedCO2 = currentCO2 - initialCO2;
}

// Calculate air densities for accurate calculation of volumes
// if we have ambient pressure and temperature sensor
bool inline calculate_airdensity()
{
  float temperature_kelvin = exhale_temperature + 273.15;

  // Volume compensation from moist body temperature air to standard dry, 0C sea-level air
  int8_t temp = (int)exhale_temperature;
  temp = (temp < 20) ? 20 : temp;
  temp = (temp > 40) ? 40 : temp;
  temp -= 20; // Index to the water vapor array

  VATPStoSTDP = 273.15 / (temperature_kelvin); // Temperature compensation
  VATPStoSTDP *= (ambient_pressure - moistVaporPressure[temp]) / 1013.25; // Pressure compensation, 1013.25 hPA is standard sea-level pressure

  // Air density calculation based on measurement conditions (ambient pressure, exhale temperature, 100 % humid air)
  // rho_humid = (pd*Md + pV * Mv) / (R*T)
  // Where pd = partial pressure of dry air (external pressure), Md = Molar mass of dry air, pv=partial pressure of 100 % humid air,
  // Mv = molar mass of water vapor, R = universal gas constant, T = temperature in Kelvin
  // Pressure is given in hPA instead of PA, so Md and Mv are scaled by 100 here!
  float rho = (ambient_pressure * 2.89652 + moistVaporPressure[temp] * 1.8016) / (8.31446 * temperature_kelvin);

  // Sanity check...
  if(rho < 0.6) rho = 0.6;
  else if(rho > 1.6) rho = 1.6;

  inverseRhoCoeff = sqrt(2.0 / rho);

  return true;
}

// Function to calculate the volume of air from the integrated venturi pressure differentials
// also volume of o2 and co2 from latest breath pulse
// Returns false if integration was still running (or no new data)
bool calculate_volumes()
{
  // Calculate amount of N2 expired
  float expiredN2;

  if(!newBreathData) {
    newVolumeData = false;
  }
  else
  {
    // Calculate the volume of air
    lastBreathVolume = flowCorrectionFactor * venturiConstant * inverseRhoCoeff * integratedPressure;
    lastBreathVolume *= (float)FLOWINTERVAL; // Interval in ms but not dividing by 1000 as we need to multiple by 1000 to get from m^3 to liters
    lastBreathVolume *= VATPStoSTDP;  // Convert volume to STDP conditions, in liters
    if(lastBreathVolume > veMax) veMax = lastBreathVolume;

    // Calculate amount of O2 removed from breating air
    // Either directly from O2 sensor, or if we have CO2 sensor, then scale also
    // from exhale volume to inhale volume with N2
    if(useCO2Sensor) {
      expiredN2 = (100.0f - currentO2 - currentCO2) * 0.01;
      lastVolumeO2 = lastBreathVolume * ((expiredN2 * 0.265) - (currentO2 * 0.01)); // Inhale volume of O2
    } else {
      depletedO2 = (initialO2 - currentO2) * 0.01;    // Scale from percents
      lastVolumeO2 = lastBreathVolume * depletedO2; // in liters, exhale volume of O2
    }

    averageO2level += currentO2;     // Average O2 level during the calculation period
    averageCO2level += currentCO2;   // Average CO2 level
    averagingTime++;

    newVolumeData = true;
    newBreathData = false;
    
    xEventGroupSetBits(sensorEvent, SENSOR_EVENT_CALC);
  }

  // Calculate average values (like average ve/minute, vo2, vco2...)
  if(integrationTime > INTEGRATION_TIME)
  {
    // Calculate average breathing volume (l / minute)
    veMean = flowCorrectionFactor * venturiConstant * inverseRhoCoeff * integrationTotal;
    veMean *= (float)FLOWINTERVAL;  // Interval in ms but not dividing by 1000 as we need to multiple by 1000 to get from m^3 to liters
    veMean *= (float)NORMALIZATION_TIME / (float)integrationTime;
    veMean *= VATPStoSTDP;  // Convert volume to STDP condition, l/minute

    // Calculate average oxygen consumption
    if(averagingTime) { // Avoid divide by zero
      averageO2level /= (float)averagingTime;
      averageCO2level /= (float)averagingTime;
      averageO2level *= 0.01;  // From %
      averageCO2level *= 0.01; // From %

      if(useCO2Sensor) {
        expiredN2 = (1.0f - averageO2level - averageCO2level);
        vo2 = veMean * ((expiredN2 * 0.265) - averageO2level); // Inhale volume of O2
        vco2 = veMean * (averageCO2level - initialCO2);
      } else {
        vo2 = veMean * averageO2level;     // veMean is already normalized to volume in 1 minute, result is l/minute
      }

      if(vo2 > vo2Max) vo2Max = vo2;
      if(vco2 > vco2Max) vco2Max = vco2;
    }
    else {
      // No breathing during the integration time at all
      // so averaging periods is zero --> values should be zeroed
      vo2 = 0.0;
      vco2 = 0.0;
      lastBreathInterval = 0;
    }

    // Check when to store values to buffer and then store them
    storeIntervalTime++;
    if(storeIntervalTime > STORE_RATE) {
      bufferData[bufferPosition].vo2 = vo2;
      bufferData[bufferPosition].ve = veMean;
      bufferData[bufferPosition].vco2 = vco2;
      bufferData[bufferPosition].resp_rate = lastBreathInterval ? (60000.0 / (float)lastBreathInterval) : 0;
      bufferData[bufferPosition].hr = sensorData.hr;  // If available
      bufferData[bufferPosition].temperature = sensorData.ambient_temperature;
      bufferData[bufferPosition].pressure = sensorData.ambient_pressure;
      bufferData[bufferPosition].o2 = sensorData.o2;

      bufferPosition++;
      if(bufferPosition >= STORE_BUFFER_SIZE) bufferPosition = 0;
      storeIntervalTime = 0;
    }

    // Ready for next round
    integrationTime = 0;
    integrationTotal = 0.0;
    averagingTime = 0;
    averageO2level = 0.0;
    averageCO2level = 0.0;

    xEventGroupSetBits(sensorEvent, SENSOR_EVENT_AVE);
  }

  xEventGroupSetBits(sensorEvent, SENSOR_EVENT_VOL);

  return true;
}


// Copy sensor data to the data struct for other
// tasks to use
void copy_sensor_data()
{
  sensorData.vo2 = 1000.0 * vo2 / weight;  // ml / kg / min
  sensorData.vo2Max = 1000.0 * vo2Max / weight;    // ml / kg / min
  sensorData.ve = lastBreathVolume;    // liters
  sensorData.veMax = veMax;    // liters / min
  sensorData.veMean = veMean;  // liters / min
  sensorData.vco2 = 1000.0 * vco2 / weight;   // ml / kg / min
  sensorData.vco2Max = 1000.0 * vco2Max / weight; // ml / kg / min
  sensorData.rq = vco2 / vo2;
  
  if(lastBreathInterval != 0)
    sensorData.resp_rate = 60000.0 / (float)lastBreathInterval;   // Breaths per minute
  else
    sensorData.resp_rate = 0;

  sensorData.ambient_temperature = ambient_temperature;
  sensorData.ambient_pressure = ambient_pressure;
  sensorData.exhale_temperature = exhale_temperature;
  sensorData.flow_value = currentPressure;  // Pa
  sensorData.o2 = currentO2;    // %
  sensorData.errors = errorCounter;
}

// Reset all calulation values, interrupt ongoing integrations etc etc
void resetCalculations(void)
{
  prevPressure0 = 0.0;
  prevPressure1 = 0.0;
  prevPressure2 = 0.0;
  currentPressure = 0.0;

  integrationTotal = 0.0;
  integrationTime = 0;
  integratePressure = false;
  newBreathData = false;
  breathIntervalCount = 0;
  lastBreathInterval = 0;
  lastBreathVolume = 0.0;

  veMax = 0.0;
  veMean = 0.0;
  newVolumeData = false;

  lastVolumeO2 = 0.0;
  averageO2level = 0.0;
  averageCO2level = 0.0;
  averagingTime = 0;

  vo2 = 0.0;
  vo2Max = 0.0;

  vco2 = 0.0;
  vco2Max = 0.0;

  bufferPosition = 0;
  memset(bufferData, 0, sizeof(bufferData));
}


// This is the main task that handles reading the sensors
// and calculating all critical parameters.
// This should be the highest priority task that does not get
// pre-empted, to guarantee that sensors are read at correct
// intervals. The taks may yield though, for example when waiting
// for I2C transmissions to complete
void sensorTask(void *params)
{
  sensorStatus = xEventGroupCreate();
  configASSERT(sensorStatus); // DEBUG

  sensorEvent = xEventGroupCreate();
  configASSERT(sensorEvent); // DEBUG

  sensorQueue = xQueueCreate(1, sizeof(sensorData_t));
  configASSERT(sensorQueue);

  // Make sure there is something in the queue
  copy_sensor_data();
  xQueueOverwrite(sensorQueue, &sensorData);

  // Initialize the extra sensor queue
  sensorExtraQueue = xQueueCreate(10, sizeof(sensorExtraData_t));
  configASSERT(sensorExtraQueue);

  // Initialize I2C bus
  if(initialize_I2C(I2C_PORT, PIN_SDA, PIN_SCL) == ESP_OK) {

    // Initialize flow sensor
    ESP_LOGI(TAG_SENSOR, "Init: Starting flow sensor initialization");
    flow_sensor.reset();
    if(flow_sensor.begin() == ESP_OK)
    {
      // Start continuous measurement
      flow_sensor.startContinuous();
      xEventGroupSetBits(sensorStatus, SENSOR_HAS_FLOW);
      ESP_LOGI(TAG_SENSOR, "Init: Flow sensor initialized");
    } else {
      ESP_LOGE(TAG_SENSOR, "Init: Flow sensor init failed");
    }

    // Initialize ambient pressure (and temperature) sensor
    ESP_LOGI(TAG_SENSOR, "Init: Starting temperature/pressure sensor initialization");
    if(pressure_sensor.begin(SLEEP_MODE, OVERSAMPLING_X4, OVERSAMPLING_SKIP, IIR_FILTER_4, TIME_STANDBY_40MS) == ESP_OK)
    {
      pressure_sensor.startNormalConversion();
      xEventGroupSetBits(sensorStatus, SENSOR_HAS_PRESSURE);
      ESP_LOGI(TAG_SENSOR, "Init: Pressure sensor initialized");
    } else {
      ESP_LOGE(TAG_SENSOR, "Init: Pressure sensor init failed");
    }


    // Initialize oxygen sensor
    ESP_LOGI(TAG_SENSOR, "Init: Starting oxygen sensor initialization");
    if(oxygen_sensor.begin() == ESP_OK)
    {
      xEventGroupSetBits(sensorStatus, SENSOR_HAS_O2);
      ESP_LOGI(TAG_SENSOR, "Init: Oxygen sensor initialized");
    } else {
      ESP_LOGE(TAG_SENSOR, "Init: Oxygen sensor init failed");
    }

    // TODO: Add CO2 sensor initialization
    ESP_LOGW(TAG_SENSOR, "Init: CO2 sensor not supported yet");
  }
  else
  {
    // Handle I2C initialization failure somehow here?
  }


  // Make sure the storage buffer is reset with zeroes
  bufferPosition = 0;
  memset(bufferData, 0, sizeof(bufferData));

  // Initialize any variables needed from global settings
  INTEGRATION_TIME = global_settings.integrationTime;
  STORE_RATE = global_settings.storeDataRate;
  flowCorrectionFactor = global_settings.flowCorrectionFactor;
  weight = global_settings.userWeight;
  if(xEventGroupGetBits(sensorStatus) & SENSOR_HAS_CO2)
    useCO2Sensor = global_settings.co2sensor_enable;  // Only allow enabling if we have the sensor


  // Signal that we have initialized all sensors --> can start other tasks
  xEventGroupSetBits(sensorStatus, SENSOR_INIT_DONE);

  xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
    // Wait until next time to read/calculate things
    // this one takes into account the time used in the previous step
    xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xFrequency);

    digitalWrite(27, true); // DEBUG: For timing purposes

    // Either we repeat the first action in the sequence
    // or one of the others
    switch(sequence[read_first ? 0 : action]) {
    case READ_FLOW:
      if(xEventGroupGetBits(sensorStatus) & SENSOR_HAS_FLOW)
        read_flow();
      break;
    case READ_O2:
      if(xEventGroupGetBits(sensorStatus) & SENSOR_HAS_O2)
        read_o2();
      break;
    case READ_CO2:
      if(xEventGroupGetBits(sensorStatus) & SENSOR_HAS_CO2)
        read_co2();
      break;
    case READ_PRESSURE:
      if(xEventGroupGetBits(sensorStatus) & SENSOR_HAS_PRESSURE)
        read_pressure();
      break;
    case CALC_AIRDENSITY:
      if(xEventGroupGetBits(sensorStatus) & SENSOR_HAS_PRESSURE)
        calculate_airdensity();
      break;
    case CALC_VOLUMES:
      calculate_volumes();
      break;
    default:
      break;
    };

    // Check if we need to (re)set initial O2 & CO2 level
    if(xEventGroupGetBits(sensorEvent) & SENSOR_EVENT_INIT) {
      initialO2 = currentO2;
      initialCO2 = currentCO2;
      xEventGroupClearBits(sensorEvent, SENSOR_EVENT_INIT);
    }

    // Check if reset of calculations was requested
    if(xEventGroupGetBits(sensorEvent) & SENSOR_EVENT_RESET) {
      resetCalculations();
      xEventGroupClearBits(sensorEvent, SENSOR_EVENT_RESET);
    }

    // Check if O2 sensor calibration was requested
    if(xEventGroupGetBits(sensorEvent) & SENSOR_EVENT_O2_CAL) {
      oxygen_sensor.calibrate(20.9, 0);   // Calibrate in air --> 20.9 % O2 concentration. MV must be set to 0 according to example code
      xEventGroupClearBits(sensorEvent, SENSOR_EVENT_O2_CAL);
    }

    // Check if reconfiguration based on global config is requested
    if(xEventGroupGetBits(sensorEvent) & SENSOR_EVENT_RECONF) {
      flowCorrectionFactor = global_settings.flowCorrectionFactor;
      weight = global_settings.userWeight;
      INTEGRATION_TIME = global_settings.integrationTime;

      if(xEventGroupGetBits(sensorStatus) & SENSOR_HAS_CO2)
        useCO2Sensor = global_settings.co2sensor_enable;  // Only allow enabling if we have the sensor
      else
        useCO2Sensor = false;   // Do not try to use the sensor if we don't have one...

      xEventGroupClearBits(sensorEvent, SENSOR_EVENT_RECONF);
    }


    // If there is extra values in queue, add them to structure
    while(xQueueReceive(sensorExtraQueue, &extraDataBuf, 0) == pdTRUE)
    {
      switch(extraDataBuf.type) {
      case SENSOR_EXTRA_HR:
        sensorData.hr = extraDataBuf.value;
        if(extraDataBuf.value > sensorData.hr_max) sensorData.hr_max = extraDataBuf.value;
        break;
      case SENSOR_EXTRA_RR:
        sensorData.rr = extraDataBuf.value;
        break;
      default: break;
      };
    }

    // Update data in the queue
    copy_sensor_data();
    xQueueOverwrite(sensorQueue, &sensorData);

    // Repeat the 1st item in the sequence after every other step
    read_first = !read_first;
    if(!read_first || action == 0 || action == (sequenceSteps-1)) // To make sure we don't repeat the 1st item
    {
      action++;
      if(action >= sequenceSteps) {
        action = 0;
        if(initialize_counter) initialize_counter--;  // Decrease initialization counter on every full loop
      }

      read_first = false;
    }

    // Reset the I2C bus if there is enough errors
    if(errorCounter & 0b111) {  // Timeout seems to be hard-coded to 1 s so this should reset every ~8 s
      //reset_I2C();
      errorCounter++;
    }
    
    digitalWrite(27, false); // DEBUG: For timing purposes
  }
}


// Copy latest sensor data to a new struct
// There should always be data available so no blocking
// Use sensorWaitEvent to wait until certain data fields are updated
bool sensorGetData(sensorData_t *copy_to)
{
  return xQueuePeek(sensorQueue, copy_to, 0) == pdTRUE;
}

// Clear event, useful before waiting for one
void sensorClearEvent(uint32_t event)
{
  xEventGroupClearBits(sensorEvent, event);
}

// Wait for event to trigger and then clear it
// returns true if event happened, false if timeout occurred first
bool sensorWaitEvent(uint32_t event, TickType_t timeout)
{
  if(!sensorEvent) {  // Fix a race condition where main may wait for initialization before the event groups is created
    vTaskDelay(timeout);
    return false;
  }
  return (xEventGroupWaitBits(sensorEvent, event, pdTRUE, pdFALSE, timeout) & event) ? true : false;
}


// Add data from external extra sensors to the structure, return true if success
// false if queue was full. Wait for timeout if so desired
bool sensorAddExtraValue(uint8_t type, float value, TickType_t timeout)
{
  sensorExtraData_t data = {type, value};
  return xQueueSendToBack(sensorExtraQueue, (void *)&data, timeout) == pdTRUE;
}

// Get status bits
uint16_t sensorGetStatus()
{
  return xEventGroupGetBits(sensorStatus);
}

// Set initial values, especially initial O2 %
void sensorSetInitial(void)
{
  xEventGroupSetBits(sensorEvent, SENSOR_EVENT_INIT);
}

// Request reset of calculations, like vo2max and such
void sensorResetCalculation(void)
{
  xEventGroupSetBits(sensorEvent, SENSOR_EVENT_RESET);
}

// Request calibration of O2 sensor in 20.9 % oxygen
void sensorO2Calibrate(void)
{
  xEventGroupSetBits(sensorEvent, SENSOR_EVENT_O2_CAL);
}

bool sensorHasErrors(void)
{
  return errorCounter > 0;
}

void sensorSetConfiguration(void)
{
  xEventGroupSetBits(sensorEvent, SENSOR_EVENT_RECONF);
}

// Get (constant) pointer to storage buffer
const storeData_t *getStorageBuffer(void)
{
  return bufferData;
}

// Get current (next) write position in the buffer
unsigned int getStorageBufferPosition(void)
{
  return bufferPosition;
}



// Initialize I2C communication channel
esp_err_t initialize_I2C(i2c_port_t i2c_port, int pinSDA, int pinSCL)
{
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
    conf.master.clk_speed = 400000;  /*!< I2C master clock frequency */

    esp_err_t err;
    err = i2c_param_config(i2c_port, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SENSOR, "Could not configure the I2C: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(i2c_port, conf.mode,
            i2c_master_rx_buf_disable, i2c_master_tx_buf_disable,
            intr_flag_disable);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SENSOR, "Failed to initialize the I2C: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_set_timeout(i2c_port, I2C_DEFAULT_TIMEOUT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SENSOR, "Failed to set timeout: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG_SENSOR, "I2C%d initialized", i2c_port);

    return err;
}

esp_err_t reset_I2C()
{
  return flow_sensor.reset();
}