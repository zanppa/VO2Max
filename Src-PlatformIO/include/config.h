/*
 * Copyright (C) 2024 Lauri Peltonen
 * GPL V3
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__


#define SETTINGS_MAGIC     0xAB
#define SETTINGS_VERSION   3

// Settings that are stored to eeprom (4 kB max)
typedef struct __attribute__ ((packed)) _settings_t {
    // Magic and version are not stored in the struct! But they are the first 2 bytes in EEPROM
    // uint8_t magic;                // Magic byte to know the data is what is expected
    // uint8_t version;              // Version of the data stored, to know that the format is/values are compatible

    float flowCorrectionFactor;   // Correction for flow calculation, from 3 L syringe test
    float userWeight;             // User weight
    int wifiDataRate;             // Wifi data rate, 0=all, 1=less, 2=minimal
    int integrationTime;          // Integration time for vo2 etc. calculation, default 15 seconds
    int storeDataRate;            // Data rate to store values to memory, multiples of integrationTime
    bool hrsensor_enable;         // Enable BLE heart rate sensor support
    bool co2sensor_enable;        // CO2 sensor active
    bool wifi_enable;             // Enable wifi data transmit support
    bool cheetah_enable;          // Enable BT BLE transmit to Golden Cheetah as VO2 Master
    uint8_t HRSensorAddress[6];   // BLE Address of the HR sensor
    char wifiStationName[32];     // Wifi station name, serial number will be concatenated to this
    char wifiPassword[32];        // Wifi password
} settings_t;

extern settings_t global_settings;

#define BUTTON_1_PIN    0     // Lower
#define BUTTON_2_PIN    35    // Upper

#define BUTTON_LOWER  0x01    // Button 1
#define BUTTON_UPPER  0x02    // Button 2


#endif