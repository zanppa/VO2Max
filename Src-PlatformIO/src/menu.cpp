/*
 * Configuration menu for VO2 sensor
 *
 * Copyright (C) 2024 Lauri Peltonen
 * GPL V3
 */

 
#include <TFT_eSPI.h>

#include "config.h"
#include "menu.h"

#include "files.h"

#include "buttonsTask.h"
#include "sensorTask.h"
#include "wifiTask.h"
#include "BLE_HRBelt.h"

void saveSettings() {}

extern TFT_eSPI tft;

extern volatile uint32_t seconds_from_start;


// Forward declarations
void func_closemenu();
void func_sendstored();
void func_submenu(void *p1, void *p2);
void func_floatnum(void *p1, void *p2);
void func_stringvalue(void *p1, void *p2);
void func_hrconnect();
void func_calibrateO2();
void func_calibrateFlow();
void func_resetValues();
void func_writeToFile();
void func_sendfiles();

void printButtonLabels(const char *upper, const char *lower)
{
  tft.setTextDatum(TR_DATUM);
  tft.drawString(upper, 235, 5, 4);
  tft.drawString(lower, 235, 115, 4);
  tft.setTextDatum(TL_DATUM);
}


class MenuItem {
public:
  MenuItem(const char *label) : label{ label } { }
  virtual void init() { }
  const char *get_label() const { return label; }
  virtual void click() { }
  virtual void draw() { tft.print(label); }
private:
  const char *label;
};

class CheckMenuItem : public MenuItem {
public:
  CheckMenuItem(const char *label, bool *value) : MenuItem{ label }, value{ value } {}
  const bool get_value() const { return *value; }
  virtual void click() override { *value = !(*value); } // Switch when clicked
  virtual void draw() override { MenuItem::draw(); tft.print(" "); tft.print((*value) ? "[Yes]" : "[No ]"); }
private:
  bool *value;
};

class FunctionMenuItem : public MenuItem {
public:
  FunctionMenuItem(const char *label, void (*fn)()) : MenuItem{ label }, fn{ fn } { }
  virtual void draw() override { MenuItem::draw(); tft.print("..."); }
  virtual void click() override { if(fn) (fn)(); }
private:
  void (*fn)();
};

// Call function with 2 pre-defined parameters
class Function2MenuItem : public MenuItem {
public:
  Function2MenuItem(const char *label, void (*fn)(void *p1, void *p2), void *p1, void *p2) : MenuItem{ label }, fn{ fn }, p1{ p1 }, p2{ p2 } { }
  virtual void draw() override { MenuItem::draw(); tft.print("..."); }
  virtual void click() override { if(fn) (fn)(p1, p2); }
private:
  void (*fn)(void *p1, void *p2);
  void *p1;
  void *p2;
};

class SelectMenuItem : public MenuItem {
public:
  SelectMenuItem(const char *label, int *variable, const char **choices, int *values, uint8_t count) : MenuItem{label}, variable{variable}, choices{choices}, values{values}, count{count}, current{0} { }
  virtual void init() override
  {
    for(int i=0;i<count;i++) {  // Set selection to the selected value
      if((*variable) == values[i]) {
        current = i;
        return;
      }
    }
  }
  const bool get_value() const { return *variable; }
  virtual void click() override { current++; if(current>=count) current=0; *variable = values[current]; } // Next item when clicked
  virtual void draw() override { MenuItem::draw(); tft.print(" ["); tft.print(choices[current]); tft.print("]"); }
private:
  int *variable;
  const char **choices;
  int *values;
  uint8_t count;
  uint8_t current;
};


class Menu {
public:
  Menu(MenuItem **items, int n_items, int sel = 0) : items { items }, n_items { n_items }, selected{ sel } {  }
  void init() { for(int i=0; i<n_items; i++) items[i]->init(); }
  void next() { selected++; if(selected >= n_items) selected = 0; }
  void prev() { selected--; if(selected < 0) selected = n_items - 1; }
  void click() { items[selected]->click(); }
  void draw() {
    int d_item = selected - (max_on_screen - 1); // Max 5 items on screen
    d_item = (d_item < 0) ? 0 : d_item;

    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);

    printButtonLabels(">", "v");

    for (int i = 0; i < max_on_screen && d_item < n_items; i++, d_item++) {
        int y = 5 + i * 25;
        int x = 5;

        tft.setCursor(x, y, 4);

        if (d_item == selected)
            tft.setTextColor(TFT_BLUE, TFT_LIGHTGREY);
        else
            tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);

        tft.print(" ");
        items[d_item]->draw();
    }
  }

private:
  MenuItem **items;
  int n_items;
  int selected;
  const int max_on_screen = 5;
};

static const char *wifiDataRateNames[3] = {"All", "Less", "Min"};
static int wifiDataRates[3] = {0, 1, 2};

static const char *intTimeNames[5] = {"5s", "10s", "15s", "30s", "60s"};
static int intTimeValues[5] = {5000, 10000, 15000, 30000, 60000};

static const char *storeRateNames[5] = {"All", "1:2", "1:3", "1:5", "1:10"};
static int storeRateValues[5] = {0, 1, 2, 4, 9};

static floatNumParams_t weightParams = {"Weight", 20.0, 200.0, 0.5};

static uint8_t wifi_string_length = 31;

static const int defaultmenu_count = 18;
static MenuItem *defaultmenu_items[defaultmenu_count] = {
                            new FunctionMenuItem("Done", &func_closemenu),
                            new FunctionMenuItem("Save to file", &func_writeToFile),
                            new FunctionMenuItem("Send memory", &func_sendstored),
                            new FunctionMenuItem("Send file", &func_sendfiles),
                            new Function2MenuItem("Set weight", &func_floatnum, &global_settings.userWeight, (void*)&weightParams),
                            new SelectMenuItem("Interval", &global_settings.integrationTime, intTimeNames, intTimeValues, 5),
                            new SelectMenuItem("Store rate", &global_settings.storeDataRate, storeRateNames, storeRateValues, 5),
                            new FunctionMenuItem("Recalibrate O2", &func_calibrateO2), 
                            new FunctionMenuItem("Calibrate flow", &func_calibrateFlow),
                            new CheckMenuItem("Wifi", &global_settings.wifi_enable),
                            new SelectMenuItem("Wifi send", &global_settings.wifiDataRate, wifiDataRateNames, wifiDataRates, 3),
                            new Function2MenuItem("Wifi name", &func_stringvalue, global_settings.wifiStationName, (void *)&wifi_string_length),
                            new Function2MenuItem("Wifi password", &func_stringvalue, global_settings.wifiPassword, (void *)&wifi_string_length),
                            new CheckMenuItem("CO2 sensor", &global_settings.co2sensor_enable),
                            new CheckMenuItem("G Cheetah", &global_settings.cheetah_enable),
                            new CheckMenuItem("HR sensor", &global_settings.hrsensor_enable),
                            new FunctionMenuItem("HR scan", &func_hrconnect),
                            new FunctionMenuItem("Start over", &func_resetValues)
                            
};


Menu default_menu = Menu(defaultmenu_items, defaultmenu_count, 0);  // Start with 1st item selected (0=Done...)
Menu *current_menu = &default_menu;



// Quit menu
bool quit_menu = false;
void func_closemenu()
{
  saveSettings();
  quit_menu = true;
}

// Request wifi to send stored data
void func_sendstored()
{
  // We abuse the wifi config a bit to send request to send the stored buffer...
  wifiSetConfig(WIFI_SEND_STORED, NULL, pdMS_TO_TICKS(100));
}

// Save memory buffer contents to SPIFFS file(s)
void func_writeToFile()
{
  unsigned int storagePosition = getStorageBufferPosition();
  const storeData_t *sendStorageData = getStorageBuffer();
  bool ret = false;

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);

  tft.setCursor(5, 5, 4);
  tft.print("Writing file... ");

  if(write_log_buffer_to_file((const char *)sendStorageData, STORE_BUFFER_SIZE, sizeof(storeData_t), storagePosition)) {
    tft.print("OK");
    vTaskDelay(pdMS_TO_TICKS(500));
  } else {
    tft.println("FAIL");
    tft.println("Press a button");
    printButtonLabels(" ", ">");
    waitButtonEvent(0, true, pdMS_TO_TICKS(5000));  // Wait until any button pressed or 5 s
  }

}

// Request WiFi task to send data stored in Flash file(s)
void func_sendfiles()
{
  // We abuse the wifi config a bit to send request to send the files...
  wifiSetConfig(WIFI_SEND_FILE, NULL, pdMS_TO_TICKS(100));
}

// Change menu to another menu pointed by parameter p1
void func_submenu(void *p1, void *p2)
{
  if(!p1)
    current_menu = &default_menu;
  else
    current_menu = (Menu *)p1;
  current_menu->init();
}

// Enter an float number. Store to (float *)p1, optional configuration is in p2
void func_floatnum(void *p1, void *p2)
{
  uint8_t buttons = 0;
  int8_t counter = 0;
  bool valueChanged = false;
  float *value = (float *)p1;
  float increment = 0.5;
  TickType_t exitTimer;
  floatNumParams_t *pp2 = (floatNumParams_t *)p2;

  if(!p1) return;

  if(pp2) increment = pp2->step;

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);

  tft.setCursor(5, 110, 2);
  tft.print("Wait 5 seconds to confirm");

  printButtonLabels("+", "-");

  if(pp2 && pp2->label)
    tft.drawString((const char *)pp2->label, 20, 10, 4);
  tft.drawFloat(*value, 1, 48, 48, 7);

  // Wait first that buttons are released
  waitButtonRelease(portMAX_DELAY);

  exitTimer = xTaskGetTickCount();
  while ((xTaskGetTickCount() - exitTimer) < pdMS_TO_TICKS(5000)) {
    waitButtonPress(&buttons, pdMS_TO_TICKS(500));
    buttons = buttonPressStatus();    // This allows us to handle also button holding

    if (buttons & 0x01) { // Button 1 pressed, decrease value
      *value -= increment;
      if(counter > -10) counter -= 1;
      else *value -= 4*increment;   // Fast steps if button is held for long
      valueChanged = true;
    }
    else if (buttons & 0x02) { // Button 2, increase value
      *value += increment;
      if(counter < 10) counter += 1;
      else *value += 4*increment;   // Fast steps if button is held for long
      valueChanged = true;
    }
    else counter = 0;   // If button is released, reset counter

    if(pp2) {
      if(*value < pp2->min) *value = pp2->min;
      if(*value > pp2->max) *value = pp2->max;
    }

    if(valueChanged) {
      tft.fillRect(48, 48, 60, 12, TFT_BLACK);  // Clear old number
      tft.drawFloat(*value, 1, 48, 48, 7);
      valueChanged = false;
      exitTimer = xTaskGetTickCount();
    }

    if(buttons) vTaskDelay(pdMS_TO_TICKS(200)); // Delay if buttons are kept pressed
    //waitButtonRelease(pdMS_TO_TICKS(500));
  }
}


// Find next character, given current one is c
// in a restricted set of characters
static char next_char(char c)
{
  if(c == 57) return 95;   // from 9 to _
  else if(c == 95) return 32;   //From _ to space (null, terminator)
  else if(c == 32) return 97; // space to a
  else if(c == 122) return 65; // z to A
  else if(c == 90) return 48; // Z to 0
  else return c+1;
}

// Enter a string. Stored as char * in p1, maximum length is uint8_t *p2
void func_stringvalue(void *p1, void *p2)
{
  uint8_t buttons = 0;
  bool valueChanged = true;
  char *value = (char *)p1;
  uint8_t maxlen = *((uint8_t *)p2);
  uint8_t curpos = 0;
  TickType_t exitTimer;

  if(!p1) return;

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);

  tft.setCursor(5, 5, 4);
  tft.print("Enter string");

  printButtonLabels(">", "v");

  tft.setCursor(5, 110, 2);
  tft.print("Wait 5 seconds to confirm");

  // Wait first that buttons are released
  waitButtonRelease(portMAX_DELAY);

  exitTimer = xTaskGetTickCount();
  while ((xTaskGetTickCount() - exitTimer) < pdMS_TO_TICKS(5000)) {
    if(valueChanged) {
      tft.fillRect(0, 48, 240, 30, TFT_BLACK);  // Clear old string

      // Draw the string
      tft.setCursor(5, 48, 4);

      // Show maximum of N characters
      uint8_t idx = (curpos>=STRING_SHOW_CHARS) ? curpos-(STRING_SHOW_CHARS-1) : 0;
      idx = (idx+STRING_SHOW_CHARS) >= maxlen ? (maxlen>STRING_SHOW_CHARS ? maxlen - STRING_SHOW_CHARS : 0) : idx;
      
      for(uint8_t i=0;i<STRING_SHOW_CHARS; i++) {
        if(idx+i >= maxlen) break;

        if(i+idx == curpos) {
          tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
        } else {
          tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
        }
        if(value[idx+i])
          tft.print(value[idx+i]);
        else
          tft.print(" "); // Null character --> empty space
      }

      if(idx+STRING_SHOW_CHARS < maxlen) {
        tft.setCursor(225, 48, 4);
        tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
        tft.print(">");
      }
      
      valueChanged = false;
      exitTimer = xTaskGetTickCount();
    }

    waitButtonPress(&buttons, pdMS_TO_TICKS(500));
    buttons = buttonPressStatus();    // This allows us to handle also button holding

    if (buttons & 0x01) { // Button 1 (low) pressed, change current character
      //ESP_LOGI("string", "Current %c next %c", value[curpos], next_char(value[curpos]));
      value[curpos] = next_char(value[curpos]);
      valueChanged = true;
    }
    else if (buttons & 0x02) { // Button 2, go to next character in string
      if(curpos < maxlen) curpos++;
      else break; // End of string, consider as quit
      valueChanged = true;
    }

    if(buttons) vTaskDelay(pdMS_TO_TICKS(200)); // Delay if buttons are kept pressed
  }

  // Finally convert any "space" (decimal 32) in the string to null
  for(uint8_t idx=0; idx<maxlen; idx++)
    if(value[idx] == 32) value[idx] = 0;
}


static sensorData_t sensor_data_buf;

// Calibrate the initial O2 level at 20.9 % outdoor level
void func_calibrateO2()
{
  uint8_t state = 0;
  uint8_t oldstate = 1;
  uint8_t buttons;
  bool quit = false;

  while(!quit) {
    if(oldstate != state || state == 2) {
      tft.fillScreen(TFT_BLACK);
      tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);

      printButtonLabels("X", ">");

      oldstate = state;

      switch(state) {
      case 0:
        tft.setCursor(0, 0, 4);
        tft.println("Vent the sensor.");
        tft.println("Wait 3 minutes");
        tft.println("for sensor to");
        tft.println("stabilize...");
        break;
      case 1:
        //tft.setCursor(0, 10, 4);
        //tft.println("Calibrating O2");
        //tft.println("sensor to 20.9 %...");
        break;
      case 2:
        tft.setCursor(0, 10, 4);
        tft.println("Calibration done!");
        tft.println("Current O2 reading");
        sensorGetData(&sensor_data_buf);
        tft.drawFloat(sensor_data_buf.o2, 1, 20, 60, 7);
        break;
      default:
        quit = true;
        break;
      };
    }
    if(waitButtonPress(&buttons, pdMS_TO_TICKS(1000)))
    {
      // Buttons were pressed, react immediately on press
      if(buttons == BUTTON_UPPER) {  // Cancel button
        quit = true;
      } else if(buttons == BUTTON_LOWER) { // Next button
        state++;
      } else waitButtonRelease(portMAX_DELAY);  // Buttons were kept pressed --> wait until released

      // State 1 is a "meta state" where we send the calibration signal
      if(state == 1) {
        sensorO2Calibrate();
        vTaskDelay(pdMS_TO_TICKS(500));
        sensorSetInitial();   // Set the calibrated value as the initial O2 value also
        state++;
      }
    }
  }
}

static floatNumParams_t calFlowParams = {"Cal. volume L", 0.5, 5.0, 0.1};
static float calFlowVolume = 3.0;

// Calibrate flow sensor
void func_calibrateFlow()
{
  uint8_t button;
  float factor = 1.0;

  // First query the flow volume
  func_floatnum(&calFlowVolume, (void *)&calFlowParams);

  // Show information
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setCursor(0, 40, 4);
  tft.println("Empty the");
  tft.println("syringe through");
  tft.println("the mask.");

  sensorClearEvent(SENSOR_EVENT_CALC);
  
  // Then wait until flow calculation updates, max 30 seconds
  if(sensorWaitEvent(SENSOR_EVENT_CALC, pdMS_TO_TICKS(30000))) {
    // Get the new data
    sensorGetData(&sensor_data_buf);
    if(sensor_data_buf.ve)  // Only update if we got non-zero value
      factor = calFlowVolume / sensor_data_buf.ve;

    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.setCursor(0, 40, 4);
    tft.println("OK!");
    tft.println("New factor is");
    tft.printf("%04.2f", factor);

    printButtonLabels("", "OK");
  } else {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.setCursor(0, 40, 4);
    tft.println("Error detecting");
    tft.println("Flow. Please try");
    tft.println("again.");

    factor = global_settings.flowCorrectionFactor;  // Make sure we don't accidentally update...
  }

  //tft.setCursor(140, 5, 4);
  //tft.print("Cancel");
  printButtonLabels("Cancel", "");

  waitButtonPress(&button, portMAX_DELAY);
  waitButtonRelease(portMAX_DELAY);

  if(button == BUTTON_LOWER)
    global_settings.flowCorrectionFactor = factor;  // TODO: Maybe works directly from here?
}


// Scan & connect to BLE HR belt if found
void func_hrconnect(void)
{
  bool skip = false;
  uint8_t but = 0;
  uint8_t spinner = 0;

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);

  printButtonLabels("Cancel", "");

  tft.setCursor(20, 40, 4);
  tft.print("Scanning...");

  BLEHRStartScan();
  //Serial.println(BLEIsScanning());

  do {
    if(waitButtonPress(&but, pdMS_TO_TICKS(200)) && but == BUTTON_UPPER)
    {
      skip = true;
      break;
    }

    // Let's see if a simple spinner works to show that something is going on...
    tft.setCursor(20, 60, 4);
    tft.print("[");
    for(uint8_t i=0;i<8;i++) {
      if((spinner < 8 && i>spinner) || (spinner >=8 && i < (spinner-8))) tft.print("-");
      else tft.print("X");
    }
    tft.print("]");
    spinner = (spinner + 1) & 0x0F;

  } while(BLEHRIsScanning());

  tft.fillScreen(TFT_BLACK);

  // Cancel was pressed, disconnect even if connection would have been
  // succesfull, or skip the connection after scan
  // Scan will finish in the background anyways...
  if(skip) {
    BLEHRDisconnect();
    return;
  }

  tft.setCursor(10, 30, 4);
  if(BLEHRIsConnected()) {
    // Print name/address of the connected sensor
    tft.println("Connected to");
    tft.print(BLEGetHRName());
  } else {
    // No suitable device found
    tft.println("No suitable");
    tft.println("devices found");
  }

  //tft.setCursor(200, 5, 4);
  //tft.print("OK");
  printButtonLabels("", "OK");
  waitButtonPress(&but, portMAX_DELAY);
}

// Reset values & timer
void func_resetValues(void)
{
  uint8_t buttons;

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setCursor(0, 40, 4);

  tft.println("Reset all values");
  tft.println("and start again?");

  printButtonLabels("Cancel", "OK");

  waitButtonRelease(portMAX_DELAY);
  waitButtonPress(&buttons, portMAX_DELAY);

  if(buttons == BUTTON_LOWER) {
    // Reset
    sensorResetCalculation();
    seconds_from_start = 0;
  }
}

void doMenu(void *params) {
  uint8_t buttons = 0;
  
//    loadSettings();
  quit_menu = false;

  tft.setRotation(1);
  tft.setTextDatum(TL_DATUM);

  //waitButtonRelease();

  current_menu->init();

  // Display
  current_menu->draw();

  while(!quit_menu) {
    // Wait for button press
    waitButtonPress(&buttons, portMAX_DELAY);

    if (buttons == BUTTON_UPPER) // Button 2 clicked
      current_menu->click();
    else if (buttons == BUTTON_LOWER) // Button 1 clicked
      current_menu->next();

    if(buttons) // Do not re-draw on button release, only if it was pressed
      current_menu->draw();

  }
  tft.fillScreen(TFT_BLACK);
}

