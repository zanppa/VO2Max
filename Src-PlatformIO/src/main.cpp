/*
 * VO2 sensor for measuring VO2 and VO2max during exercise
 *
 * Copyright (C) 2024, 2025 Lauri Peltonen
 * GPL V3
 *
 * Based heavily on IHewitt's VO2max project
 * https://github.com/ihewitt/VO2max
 *
 *
 * Main parts:
 *   Lilygo T-display (ESP32, 1.14" display, bluetooth, wifi)
 *   DFRobot SEN0322 I2C oxygen sensor
 *   Sensirion SDP801 500 Pa I2C differential pressure sensor
 *   BMP388 ambient pressure sensor (DFRobot Gravity BMP388)
 *
 *
 * PlatformIO board settings:
 *
 * Board: Lilygo-t-display
 * Upload Speed: 921600
 * CPU Frequency: 240Mhz (WiFi/BT)
 * Flash Frequency: 80Mhz
 * Flash Mode: QIO
 * Partition Scheme: Default.csv [4MB with spiffs (1.2MB APP/1.5 SPIFFS)]
 * Core Debug Level: None --> For debugging can set also different values, for release select None
 * PSRAM: No
 *
 * Remember to modify the TFT_eSPI configuration file to select T-Display!
 * Select this one line in "user_setup_select.h"
 * #include <User_Setups/Setup25_TTGO_T_Display.h>    // Setup file for ESP32 and TTGO T-Display ST7789V SPI bus TFT
 * and don't forget to comment out the custom user setup
 * //#include <User_Setup.h>           // Default setup is root library folder
 * 
 * 
 * With NimBLE-Arduino the code just barely fits into default partition.
 * In case of problems (e.g. if program does not fit with debug level = info), one can use
 *    huge_app.csv / Huge APP (3 MB No OTA / 1 MB SPIFFS)
 * partition scheme to make the program fit.
 *
 */

/*
 * External boards requirements
 *  - esp32 by Espressif (3.0.2 --> 3.0.5)
 *
 * External library requirements
 * - tft_espi (version 2.5.43)
 * - NimBLE_Arduino (version 1.4.1)
 */


#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

//#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/gpio.h"

#include <TFT_eSPI.h>

#include "EEPROM.h"

#include "config.h"
#include "sensorTask.h"
#include "buttonsTask.h"
#include "wifiTask.h"
#include "status.h"
#include "menu.h"
#include "BLE_HRBelt.h"


#define VO2_PIN_DEBUG   27
#define VO2_PIN_ADC_EN  14
#define VO2_PIN_BAT_VOLT  34

#define EEPROM_STORE_SIZE   sizeof(settings_t)
#define EEPROM_STORE_START  2   // first is magic and 2nd is version, so data starts from offset 2


// Default settings
settings_t global_settings = {
  .flowCorrectionFactor = 1.0,
  .userWeight = 80.0,
  .wifiDataRate = 0,
  .integrationTime = 15000,
  .hrsensor_enable = false,
  .co2sensor_enable = false,
  .wifi_enable = true,
  .cheetah_enable = false,
  .HRSensorAddress = {0, 0, 0, 0, 0, 0},
  .wifiStationName = {'V', 'O', '2', 'M', 'A', 'X', '_', 0},
  .wifiPassword = {'4', '3', '2', '1', 'a', 's', 'd', 'f', 0},
};


#define PRIORITY_SENSORTASK   50
#define PRIORITY_WIFITASK     20
#define PRIORITY_BLETASK      15
#define PRIORITY_BUTTONTASK   10
#define PRIORITY_GUITASK      5

// Display
TFT_eSPI tft;

static const char *TAG_VO2 = "VO2MAX";

// Handles to other tasks
TaskHandle_t task_sensorTask;
TaskHandle_t task_buttonTask;
TaskHandle_t task_wifiTask;
//TaskHandle_t task_guiTask;
TaskHandle_t task_BLETask;
TimerHandle_t timer_clock;

volatile uint32_t seconds_from_start = 0;

void debugPrintConfig(void)
{
  Serial.println("Configuration:");
  Serial.print(" Correction factor: ");
  Serial.println(global_settings.flowCorrectionFactor);
  Serial.print(" Weight: ");
  Serial.println(global_settings.userWeight);
  Serial.print(" Wifi rate: ");
  Serial.println(global_settings.wifiDataRate);
  Serial.print(" Int time: ");
  Serial.println(global_settings.integrationTime);
  Serial.print(" Store rate: ");
  Serial.println(global_settings.storeDataRate);
  Serial.print(" HR Sensor: ");
  Serial.println(global_settings.hrsensor_enable);
  Serial.print(" CO2 sensor: ");
  Serial.println(global_settings.co2sensor_enable);
  Serial.print(" Wifi: ");
  Serial.println(global_settings.wifi_enable);
  Serial.print(" Golden Cheetah: ");
  Serial.println(global_settings.cheetah_enable);
  Serial.print(" HR Address: ");
  for(int i=0;i<6;i++)
    Serial.print(global_settings.HRSensorAddress[i], HEX);
  Serial.println();
}

// Keep track of running time in nice units
void clockTimer(TimerHandle_t xTimer)
{
  seconds_from_start++;
}

float battery_voltage = 0.0;

float readBatteryVoltage()
{
  digitalWrite(VO2_PIN_ADC_EN, HIGH);
  vTaskDelay(pdMS_TO_TICKS(1));
  float measurement = (float) analogRead(VO2_PIN_BAT_VOLT);
  measurement = (measurement / 4095.0) * 7.26;
  digitalWrite(VO2_PIN_ADC_EN, LOW);
  battery_voltage = measurement;
  return measurement;
}

// Load settings structure from EEPROM
bool loadSettings() {
  uint8_t b;
  // Check magic and version to make sure EEPROM contents are compatible
  if((b = EEPROM.read(0)) != SETTINGS_MAGIC) {
    ESP_LOGE(TAG_VO2, "EEPROM: Magic does not match (%x != %x)", b, SETTINGS_MAGIC);
    return false;
  }
  if((b = EEPROM.read(1)) != SETTINGS_VERSION) {
    ESP_LOGE(TAG_VO2, "EEPROM: Version does not match (%d != %d)", b, SETTINGS_VERSION);
    return false;
  }

  // Read rest of the settings
  for(int i=0; i<EEPROM_STORE_SIZE; i++)
      ((byte *)&global_settings)[i] = EEPROM.read(i + EEPROM_STORE_START);

  ESP_LOGI(TAG_VO2, "EEPROM: Settings restored");

  return true;
}

// Store current settings struct to EEPROM
void storeSettings() {
  bool changed = false;

  // Check magic and version
  if(EEPROM.read(0) != SETTINGS_MAGIC) {
    EEPROM.write(0, SETTINGS_MAGIC);
    changed = true;
  }

  if(EEPROM.read(1) != SETTINGS_VERSION) {
    EEPROM.write(1, SETTINGS_VERSION);
    changed = true;
  }

  for(int i=0; i<EEPROM_STORE_SIZE; i++) {
    uint8_t b = EEPROM.read(i + EEPROM_STORE_START);
    uint8_t sb = ((byte *)&global_settings)[i];
    if (b != sb) {
      EEPROM.write(i + EEPROM_STORE_START, sb);
      changed = true;
    }
  }
  if(changed) {
    EEPROM.commit();
    ESP_LOGI(TAG_VO2, "EEPROM: Commit");
  }

  ESP_LOGI(TAG_VO2, "EEPROM: Settings stored");
}


void setup() {
  // Initialize serial so we get debug messages etc.
  Serial.begin(115200);

  // Pre-initialize button I/O pins so we can control the boot-up if necessary
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);

  configASSERT(EEPROM_STORE_SIZE+2 <= 512); // Sanity check

  // Initialize EEPROM, also add 2 bytes for version and magic in addition to settings data
  EEPROM.begin(EEPROM_STORE_SIZE + 2);

  // Debug print default config
  debugPrintConfig();

  // If upper button is held down during boot, reset to default settings,
  // i.e. do not read from EEPROM
  if(digitalRead(BUTTON_2_PIN)) // Returns true if button is not pressed
    loadSettings();
  else 
    ESP_LOGI(TAG_VO2, "Setup: Reverting to default settings");

  // Debug print loaded config
  debugPrintConfig();

  // Initialize the display
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  //tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  //tft.setCursor(24, 48, 7);
  //tft.print("VO2Max");

  pinMode(VO2_PIN_DEBUG, OUTPUT); // DEBUG output pin

  // Setup ADC  for battery voltage measurement
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t)ADC_UNIT_1, (adc_atten_t)ADC_ATTEN_DB_2_5, (adc_bits_width_t)ADC_WIDTH_BIT_12, 1100, &adc_chars);
  pinMode(VO2_PIN_ADC_EN, OUTPUT);

  // Initialize status screens
  initScreens();

  // Initialize button interrupt, one way to handle buttons
  buttonInit();
  attachInterrupt(BUTTON_1_PIN, buttonInterrupt, CHANGE);
  attachInterrupt(BUTTON_2_PIN, buttonInterrupt, CHANGE);

  // Create other tasks
  xTaskCreate(sensorTask, "sensor", 4096, nullptr, PRIORITY_SENSORTASK, &task_sensorTask);
  sensorWaitEvent(SENSOR_INIT_DONE, pdMS_TO_TICKS(1000));

  //xTaskCreate(buttonTask, "buttons", 1024, nullptr, PRIORITY_BUTTONTASK, &task_buttonTask); // Alternative way to handle buttons
  xTaskCreate(wifiTask, "wifi", 4096, nullptr, PRIORITY_WIFITASK, &task_wifiTask);
  xTaskCreate(BLETask, "BLE", 4096, nullptr, PRIORITY_BLETASK, &task_BLETask);

  timer_clock = xTimerCreate("clock", pdMS_TO_TICKS(1000), pdTRUE, nullptr, clockTimer);
  xTimerStart(timer_clock, 0);

  // Update initial configuration, default or from eeprom
  sensorSetConfiguration();

  // Queue automatic start of other features that may also be disabled
  if(global_settings.hrsensor_enable)
    BLEHRConnect();

  if(global_settings.cheetah_enable)
    BLEGCStart();

  if(global_settings.wifi_enable)
    wifiRequestStart();
}

static sensorData_t temp_daatta;

void loop() {
  uint8_t button = 0;
  uint32_t batteryReadTime = 0;

  // Show the initial warm-up screen first
  statusInitial();

  // Loop to show either status screens or menu
  for(;;)
  {
    // Read battery voltage every now and then
    if(seconds_from_start > batteryReadTime) {
      batteryReadTime += 30;   // Read every 30 seconds
      readBatteryVoltage();
    }

    waitButtonPress(&button, pdMS_TO_TICKS(500)); // Update twice a second
    if(button == BUTTON_LOWER) {
      changeScreen(true);
    }
    else if(button == BUTTON_UPPER) {
      doMenu(nullptr);  // Blocking call to menu

      // Store settings when closing the menu
      storeSettings();

      // Update sensor task configuration if there was changes
      sensorSetConfiguration();

      // Check if relevant settings were changed
      // Start or stop WiFi
      if(global_settings.wifi_enable)
        wifiRequestStart();
      else
        wifiRequestStop();

      // Start or stop BLE HR sensor service
      if(global_settings.hrsensor_enable)
        BLEHRConnect();
      else
        BLEHRDisconnect();

      // Start or stop Golden Cheetah service
      if(global_settings.cheetah_enable)
        BLEGCStart();
      else
        BLEGCStop();
    }

    showScreen(); // Update status screen
  }
}
