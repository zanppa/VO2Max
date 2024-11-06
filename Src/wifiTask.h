/*
 * Copyright (C) 2024 Lauri Peltonen
 * GPL V3
 */

#ifndef __WIFI_H__
#define __WIFI_H__

extern EventGroupHandle_t wifiEvent;
#define WIFI_SETTINGS_UPDATE    0x0001
#define WIFI_SETTINGS_COPIED    0x0002
#define WIFI_REQUEST_START      0x0004
#define WIFI_STATUS_STARTED     0x0008
#define WIFI_REQUEST_STOP       0x0010
#define WIFI_STATUS_STOPPED     0x0020
#define WIFI_HAS_CLIENTS        0x0100

// Struct to handle setting wifi name or password
typedef struct _wifiSetting_t {
  uint8_t type;
  void *value;    // Pointer to data
} wifiSetting_t;
#define WIFI_SETTING_NAME     0x01  // Set wifi station name
#define WIFI_SETTING_PASS     0x02  // Set wifi password
#define WIFI_SEND_STORED      0x10  // Send stored data over UDP, not really a setting...


#define WIFI_PACKET_REALTIME    0x01
#define WIFI_PACKET_STORED      0x02

// Structure of the UDP data packets, i.e. variables
typedef struct __attribute__ ((packed)) _wifiUdpPacket_t {
  uint8_t status;   // Status bits (0x01 = real time data, 0x02 = stored data)
  uint8_t dummy1;   // Reserved -- for packing
  uint16_t errors;
  float flow;
  float o2;
  float ve;
  float vo2;
  float vco2;
  float resp_rate;
  float pressure;
  float temperature;
  float exhale_temperature;
  float hr;
  float rr;
} wifiUdpPacket_t;


void wifiTask(void *params);

void wifiRequestStart(void);
void wifiRequestStop(void);

bool wifiSetConfig(uint8_t type, void *param, TickType_t timeout);

uint16_t wifiGetStatus(void);

#endif