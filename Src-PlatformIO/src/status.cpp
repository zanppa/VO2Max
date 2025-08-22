/*
 * Status screens for VO2 sensor
 * Shows real time information about the sensor
 *
 * Copyright (C) 2024 Lauri Peltonen
 * GPL V3
 */

#include <TFT_eSPI.h>
#include <vector>

#include "config.h"
#include "status.h"

#include "sensorTask.h"
#include "wifiTask.h"
#include "buttonsTask.h"
#include "BLE_HRBelt.h"

// Display handler
extern TFT_eSPI tft;


extern float battery_voltage;

static sensorData_t sensor_data_buf;
static bool hr_blink = false;


// Forward declarations
void statusInitial();
void drawStatusBar();


// Simple numerical value gauge. x, y are top center of text
class Gauge {
public:
  Gauge(int x, int y, volatile float *var, int s=4) : x{x}, y{y}, var{var}, size{s} { }
  virtual void draw() { tft.drawFloat(*var, 1, x, y, size); }
protected:
  int x, y;
  volatile float *var;
  int size;
};

// Simple label, no variable
class LabelGauge : public Gauge {
public:
  LabelGauge(int x, int y, const char *lbl, int fs=4) : Gauge{x, y, nullptr, fs}, label{lbl}, size{fs} { }
  void draw() override { tft.drawString(label, x, y, size); }
private:
  int size;
  const char *label;
};

// Simple label with variable (boolean) to toggle color
class ColorLabelGauge : public Gauge {
public:
  ColorLabelGauge(int x, int y, const char *lbl, bool *val, uint16_t off=TFT_RED, uint16_t on=TFT_GREEN, int fs=4) : Gauge{x, y, nullptr, fs}, label{lbl}, size{fs}, toggle{val}, off_color{off}, on_color{on} { }
  void draw() override { if(toggle && *toggle) tft.setTextColor(on_color, TFT_BLACK); else tft.setTextColor(off_color, TFT_BLACK); tft.drawString(label, x, y, size); tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK); }
private:
  int size;
  const char *label;
  bool *toggle;
  uint16_t on_color;
  uint16_t off_color;
};

// Arc gauge. x, y are center point
class ArcGauge : public Gauge {
public:
  ArcGauge(int x, int y, volatile float *var, int fs=4, float min=0, float max=1.0, int r=22) : Gauge{x, y-2, var, fs}, min{min}, max{max-min}, r{r} { }
  virtual void draw() override {
    float amax = ((*var) < min) ? min : (*var);
    amax = 300.0 * (amax/max);
    amax = ((amax > 300) ? 300 : amax) + 30;
    tft.drawArc(x, y, r, r-4, 30, (int)amax, TFT_GREEN, TFT_BLACK, true);
    tft.drawArc(x, y, r, r-4, (int)amax, 330, TFT_BLACK, TFT_BLACK, true); // Clear rest of the arc
    Gauge::draw();
  }
private:
  int r; // Outer radius
  float min, max;
};


static Gauge *screen1[] = {
  new LabelGauge(20, 30, "HR"),
  new Gauge(90, 30, &sensor_data_buf.hr),
  new ColorLabelGauge(180, 30, "<3", &hr_blink, TFT_LIGHTGREY, TFT_RED),  // This one blinks when new data is received form HR meter
  new LabelGauge(20, 55, "VO2"),
  new Gauge(90, 55, &sensor_data_buf.vo2),
  new LabelGauge(20, 80, "VE"),
  new Gauge(90, 80, &sensor_data_buf.veMean),
//  new ArcGauge(160, 80, &sensor_data_buf.flow_value, 4, 0, 200.0, 48),
};

static Gauge *screen2[] = {
  new LabelGauge(20, 30, "Pa"),
  new Gauge(70, 30, &sensor_data_buf.ambient_pressure),
  new LabelGauge(180, 30, "hPa"),
  new LabelGauge(20, 55, "Ta"),
  new Gauge(70, 55, &sensor_data_buf.ambient_temperature),
  new LabelGauge(180, 55, "C"),
  new LabelGauge(20, 80, "Te"),
  new Gauge(70, 80, &sensor_data_buf.exhale_temperature),
  new LabelGauge(180, 80, "C"),
};

static Gauge *screen3[] = {
  new LabelGauge(20, 30, "O2"),
  new Gauge(90, 30, &sensor_data_buf.o2),
  new LabelGauge(180, 30, "%"),
  new LabelGauge(20, 55, "Vo2max"),
  new Gauge(140, 55, &sensor_data_buf.vo2Max),
  new LabelGauge(20, 80, "Vemax"),
  new Gauge(140, 80, &sensor_data_buf.veMax),
};


/* ***
 * For reference the sensor data struct is copied here in comments
  float vo2;      // Latest volume of o2 consumed in one minute [ml/min]
  float vo2Max;   // Abs. maximum of previous [ml/min]
  float ve;       // Latest exhaust air volume (single breath) [l]
  float veMax;    // Maximum of single breath exhaust volume [l]
  float veMean;   // One minute average of exhaust air volume [l]
  float vco2;     // Latest volume of co2 consumed in one minute [ml/min]??
  float vco2Max;  // Abs. max of previous [ml/min]??
  float rq;       // Respiratory quotient, Vco2/Vo2 (also respiratory exchange ratio, RER)
  float resp_rate;// Respiratory rate
  float ambient_pressure; // Ambient air pressure [hPa]
  float ambient_temperature;  // Ambient air temperature [degC]
  float exhale_temperature;   // Exhale temperature from pressure sensor
  float o2;       // O2 sensor reading (raw value) [%]
  float flow_value;   // Instant value from the flow sensor (pressure) [Pa]

// Extra sensors (outside the sensorTask)
  float hr;           // Heart rate
  float rr;           // Heart rate r-r variation

// Debug variables
  uint16_t errors;    // Error counter
*/


class StatusScreen {
public:
  StatusScreen() : current{0} { }
  void add_screen(Gauge **screen, int n, int o) { screens.push_back(screen); n_gauges.push_back(n); orientations.push_back(o); } // Orientation: 0,1,2,3; 0/2=portrait; 1/3=landscape
  void next() { current++; if(current >= screens.size()) current = 0; tft.setRotation(orientations[current]); tft.fillScreen(TFT_BLACK); }
  void prev() { current--; if(current < 0) current = screens.size()-1; tft.setRotation(orientations[current]); tft.fillScreen(TFT_BLACK); }
  void set(int n) { current = (n < 0) ? 0 : ((n >= screens.size()) ? screens.size()-1 : n); tft.setRotation(orientations[current]); tft.fillScreen(TFT_BLACK); }
  void draw() {
    //tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    drawStatusBar(); // Show date & status icons on top row of each screen
    tft.setTextDatum(TL_DATUM); // Top left alignment for gauges
    uint16_t oldpad = tft.getTextPadding();
    tft.setTextPadding(5);    // Help erase old numbers
    for(int i=0; i<n_gauges[current]; i++) screens[current][i]->draw();
    tft.setTextPadding(oldpad);
  }
private:
  std::vector<Gauge**> screens;
  std::vector<int> n_gauges;
  std::vector<int> orientations;
  int current;
};

extern volatile uint32_t seconds_from_start;

// Draw time & status symbols
void drawStatusBar()
{
  uint32_t seconds = seconds_from_start;
  uint16_t hours;
  uint8_t minutes;
  uint16_t status;
  static uint16_t maxBufferPos = 0;

  // Convert seconds to hours,minutes and seconds
  hours = seconds / 3600;
  seconds -= hours*3600;
  minutes = seconds / 60;
  seconds -= minutes * 60;

  // Write running time on top
  tft.setTextDatum(TL_DATUM);
  tft.setCursor(60, 0, 4);
  tft.printf("%02d:%02d:%02d", hours, minutes, seconds);

  // Write battery voltage
  float volts = battery_voltage;
  if(volts > 4.4) tft.setTextColor(TFT_SKYBLUE, TFT_BLACK);       // Charger connected, at least 250 mV+150 mV drop from 5V
  else if(volts > 3.7) tft.setTextColor(TFT_GREEN, TFT_BLACK);    // Full battery
  else if(volts > 3.59) tft.setTextColor(TFT_YELLOW, TFT_BLACK);  // Possily 300 mV drop from battery to 3.3 V rail
  else tft.setTextColor(TFT_RED, TFT_BLACK);                      // 3.3 V might not stay up --> possible errors
  tft.setCursor(0, 116, 2);
  tft.printf("%4.2fV", volts);

  // Write feature status on the bottom
  status = sensorGetStatus();
  tft.setCursor(45, 116, 2);

  if(status & SENSOR_HAS_FLOW) tft.setTextColor(TFT_GREEN, TFT_BLACK);
  else tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.print("F ");

  if(status & SENSOR_HAS_O2) tft.setTextColor(TFT_GREEN, TFT_BLACK);
  else tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.print("O2 ");

  if(status & SENSOR_HAS_PRESSURE) tft.setTextColor(TFT_GREEN, TFT_BLACK);
  else tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.print("P ");

  if(status & SENSOR_HAS_CO2) tft.setTextColor(TFT_GREEN, TFT_BLACK);
  else tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.print("CO2 ");

  status = wifiGetStatus();
  if(status & WIFI_HAS_CLIENTS) tft.setTextColor(TFT_GREEN, TFT_BLACK);
  else if(status & WIFI_STATUS_STARTED) tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  else tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.print("W ");

  if(BLEHRIsConnected()) tft.setTextColor(TFT_GREEN, TFT_BLACK);
  else if(BLEHRIsInitOK()) tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  else tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.print("HR ");

  if(BLEGCHasClient()) tft.setTextColor(TFT_GREEN, TFT_BLACK);
  else if(BLEGCIsStarted()) tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  else tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.print("GC ");

  if(sensorHasErrors()) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.print("E");
  }

  // Write storage buffer position
  status = getStorageBufferPosition();
  if(status > maxBufferPos) maxBufferPos = status;
  // TODO: Need to handle reset here also!
  tft.fillRect(0, 131, maxBufferPos/5, 3, TFT_RED);
  tft.fillRect(maxBufferPos/5+1, 131, 205, 3, TFT_DARKGREY);


  // Buttons
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("M", 220, 0, 4);
  tft.drawString(">", 220, 105, 4);
}


// Show the initialization screen
void statusInitial()
{
  uint32_t target = seconds_from_start + 180; // 3 minutes warm-up time
  uint32_t left = 180;
  bool ready = false;
  uint8_t buttons = 0, oldButtons = 0;
  uint16_t sensors = sensorGetStatus();

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setCursor(180, 105, 4);
  tft.print("Skip");

  // Warn if O2 sensor was not found
  if(!(sensors & SENSOR_HAS_O2))
  {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setCursor(20, 14, 4);
    tft.println("Warning!");
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.println("Oxygen sensor");
    tft.print("not found");
    while(!waitButtonPress(&buttons, pdMS_TO_TICKS(1000)));
  }
  // Warn if flow (pressure) sensor was not found
  if(!(sensors & SENSOR_HAS_FLOW))
  {
    tft.fillRect(0, 20, 240, 80, TFT_BLACK); // Clear  
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setCursor(20, 14, 4);
    tft.println("Warning!");
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.println("Flow sensor");
    tft.print("not found");
    while(!waitButtonPress(&buttons, pdMS_TO_TICKS(1000)));
  }

  tft.fillRect(0, 20, 240, 80, TFT_BLACK); // Clear

  // No point warming up the O2 sensor if it was not properly initialized/found
  if(sensors & SENSOR_HAS_O2) 
  {
    tft.setCursor(20, 14, 4);
    tft.print("Warming up...");

    while(!ready)
    {
      if(seconds_from_start >= target) ready = true;

      // Print the warm-up time left
      left = target - seconds_from_start;
      tft.fillRect(48, 48, 60, 12, TFT_BLACK); // Clear
      tft.setCursor(48, 48, 7);
      tft.printf("%03d", left);

      // Print current oxygen level
      sensorGetData(&sensor_data_buf);
      tft.setCursor(20, 105, 4);
      tft.printf("O2  %04.1f", sensor_data_buf.o2);
      tft.print("%");

      // If "skip" button is pressed or timeout
      waitButtonPress(&buttons, pdMS_TO_TICKS(1000));
      if(buttons == BUTTON_LOWER) ready = true;
    }
  }
  tft.fillScreen(TFT_BLACK);

  // Check whether O2 level is sensible now
  if(sensor_data_buf.o2 < 20.0)
  {
    tft.setCursor(20, 14, 4);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.println("Warning!");
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.println("Initial O2 level low!");
    tft.setCursor(180, 105, 4);
    tft.print("Skip");

    do {
      sensorGetData(&sensor_data_buf);
      tft.setCursor(20, 105, 4);
      tft.printf("O2  %04.1f", sensor_data_buf.o2);
      tft.print("%");
      waitButtonPress(&buttons, pdMS_TO_TICKS(1000));
    } while(buttons != BUTTON_LOWER);
    tft.fillScreen(TFT_BLACK);
  }
  // TODO: Do we need some sanity check here after the warning..?
  sensorSetInitial();   // Set initial O2 level after warm-up period

  seconds_from_start = 0; // Reset the clock tick
}


StatusScreen st_screen;

// Build and add all the screens to the system, and select default one
void initScreens()
{
  st_screen.add_screen(screen1, sizeof(screen1)/sizeof(screen1[0]), 1);
  st_screen.add_screen(screen2, sizeof(screen2)/sizeof(screen2[0]), 1);
  st_screen.add_screen(screen3, sizeof(screen3)/sizeof(screen3[0]), 1);
  setScreen(0);
}

// Show (draw) the active status screen
void showScreen() { // select active screen
  sensorGetData(&sensor_data_buf);  // Update data
  hr_blink = BLEHRWasNotified(true);  // Blink HR if data was received
  st_screen.draw(); // Draw the screen
}

// Change next or previous screen
void changeScreen(bool forwards)
{
  if(forwards)
    st_screen.next();
  else
    st_screen.prev();
}

// Set screen index directly
void setScreen(int n)
{
  st_screen.set(n);
}