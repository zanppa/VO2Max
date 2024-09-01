/*
 * Wifi support for VO2 sensor
 * Sends data over Wifi with UDP for further analysis
 *
 * Copyright (C) 2024 Lauri Peltonen
 * GPL V3
 */

 
#include <WiFi.h>
#include <AsyncUDP.h>
#include "esp_wifi.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include "config.h"
#include "wifiTask.h"
#include "sensorTask.h"


// UDP library
static AsyncUDP udp;

// Access point (server) configuration
static char wifiStationName[32] = "ABCDEFGHIJ"; // Station name, overriden by default configuration
static char wifiPassword[32] = "QWERTYUI"; // Password... At least 8 chars? Empty to have no security. Overriden by default config
static int wifiChannel = 1;                     // Wifi channel, tune this if necessary
static IPAddress wifiIP(192, 168, 30 ,1);   // Default IP address of the server
static IPAddress wifiNetmask(255, 255, 255, 0);
static int wifiLocalPort = 8008;

// UDP broadcast settings
static IPAddress wifiBcastIP(255, 255, 255, 255); // Broadcast to all clients connected
static int wifiBcastPort = 8008;                  // Port where the clients should listen

static wifiUdpPacket_t packet;


// Event flags for other tasks to monitor wifi status
EventGroupHandle_t wifiEvent = nullptr;

// Queue to handle setting the parameters
QueueHandle_t wifiQueue = nullptr;
wifiSetting_t wifiSettingBuf;

static sensorData_t sensor_data_buf;


// Build the UDP data packet to be sent
void inline createPacket()
{
  packet.flow = sensor_data_buf.flow_value;
  packet.o2 = sensor_data_buf.o2;
  packet.ve = sensor_data_buf.veMean;
  packet.vo2 = sensor_data_buf.vo2;
  packet.vco2 = sensor_data_buf.vco2;
  packet.resp_rate = sensor_data_buf.resp_rate;
  packet.pressure = sensor_data_buf.ambient_pressure;
  packet.temperature = sensor_data_buf.ambient_temperature;
  packet.exhale_temperature = sensor_data_buf.exhale_temperature;
  packet.hr = sensor_data_buf.hr;
  packet.rr = sensor_data_buf.rr;
  packet.errors = sensor_data_buf.errors;
}

void wifiTask(void *params)
{
  EventBits_t wifiEventCode = SENSOR_EVENT_FLOW;

  wifiEvent = xEventGroupCreate();
  configASSERT(wifiEvent); // DEBUG
  xEventGroupSetBits(wifiEvent, WIFI_STATUS_STOPPED);

  wifiQueue = xQueueCreate(10, sizeof(wifiSetting_t));
  configASSERT(wifiQueue);

  // Copy station name and password from global settings
  strncpy(wifiStationName, global_settings.wifiStationName, 31);
  strncpy(wifiPassword, global_settings.wifiPassword, 31);

  strncat(wifiStationName, SERIALNUMBER, 31-strlen(wifiStationName)); // Copy the "unique" ID to the station name

  for(;;)
  {
    switch(global_settings.wifiDataRate) {
    case 2:
      wifiEventCode = SENSOR_EVENT_AVE;   // When averaging time has passed, ~15 s
      break;
    case 1:
      wifiEventCode = SENSOR_EVENT_VOL;   // Every volume calculation step, ~100 ms
      break;
    case 0:
    default:
      wifiEventCode = SENSOR_EVENT_FLOW;  // Every other sensor step, ~20 ms
      break;
    };

    if(sensorWaitEvent(wifiEventCode, pdMS_TO_TICKS(1000)))
    {
      // New data available --> broadcast over udp if wifi is enabled
      if(xEventGroupGetBits(wifiEvent) & WIFI_STATUS_STARTED)
      {
        sensorGetData(&sensor_data_buf);
        createPacket();
        
        // AsyncUDP
        udp.broadcastTo((uint8_t *)&packet, sizeof(packet), wifiBcastPort);
      }
    }

    // Check if clients are connected
    if(WiFi.softAPgetStationNum()) {
      xEventGroupSetBits(wifiEvent, WIFI_HAS_CLIENTS);
    } else {
      xEventGroupClearBits(wifiEvent, WIFI_HAS_CLIENTS);
    }


    // Request start of wifi, create access point
    if((xEventGroupGetBits(wifiEvent) & WIFI_REQUEST_START) && !(xEventGroupGetBits(wifiEvent) & WIFI_STATUS_STARTED)) {
      xEventGroupClearBits(wifiEvent, WIFI_REQUEST_START);

      // Hostname needs to be set before turning wifi on
      WiFi.setHostname(wifiStationName);

      // Initialize wifi in access point mode, i.e.
      // clients can connect to this one
      WiFi.softAP(wifiStationName, wifiPassword, wifiChannel);
      vTaskDelay(pdMS_TO_TICKS(500));  // Assume wifi gets started in 0.5 seconds. TODO: Crude hack, should wait for SYSTEM_EVENT_AP_START

      WiFi.softAPConfig(wifiIP, wifiIP, wifiNetmask); // IP, gateway (not used), netmask

      // TODO: Check whether wifi was properly started?
      xEventGroupClearBits(wifiEvent, WIFI_STATUS_STOPPED);
      xEventGroupClearBits(wifiEvent, WIFI_HAS_CLIENTS);
      xEventGroupSetBits(wifiEvent, WIFI_STATUS_STARTED);
    }

    else if((xEventGroupGetBits(wifiEvent) & WIFI_REQUEST_STOP) &&  !(xEventGroupGetBits(wifiEvent) & WIFI_STATUS_STOPPED)) {
      // Stop the wifi
      xEventGroupClearBits(wifiEvent, WIFI_REQUEST_STOP);

      // De-auth all clients
      esp_wifi_deauth_sta(0);

      WiFi.softAPdisconnect(true);    // Also turn wifi off

      xEventGroupClearBits(wifiEvent, WIFI_STATUS_STARTED);
      xEventGroupClearBits(wifiEvent, WIFI_HAS_CLIENTS);
      xEventGroupSetBits(wifiEvent, WIFI_STATUS_STOPPED);
    }

    // Handle configuration requests
    while(xQueueReceive(wifiQueue, &wifiSettingBuf, 0) == pdTRUE) {
      switch(wifiSettingBuf.type) {
      case WIFI_SETTING_NAME:
        strncpy(wifiStationName, (char *)wifiSettingBuf.value, 31);
        break;
      case WIFI_SETTING_PASS:
        strncpy(wifiPassword, (char *)wifiSettingBuf.value, 31);
        break;
      default: break;
      };
    }

  }
}

// Request start of wifi
void wifiRequestStart(void)
{
  if(wifiEvent)
    xEventGroupSetBits(wifiEvent, WIFI_REQUEST_START);
}

// Request stopping of wifi (turn off wifi)
void wifiRequestStop(void)
{
  if(wifiEvent)
    xEventGroupSetBits(wifiEvent, WIFI_REQUEST_STOP);
}

// Set wifi configuration
// should be set before starting...
bool wifiSetConfig(uint8_t type, void *param, TickType_t timeout)
{
  wifiSetting_t data = {type, param};
  if(wifiQueue)
    return xQueueSendToBack(wifiQueue, (void *)&data, timeout) == pdTRUE;
}

uint16_t wifiGetStatus(void)
{
  if(wifiEvent)
    return xEventGroupGetBits(wifiEvent);
  else
    return 0;
}