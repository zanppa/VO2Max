/*
 * Copyright (C) 2024 Lauri Peltonen
 * GPL V3
 */

#ifndef __BLE_HRBELT__
#define __BLE_HRBELT__

#include <NimBLEDevice.h>
#include <NimBLEUtils.h>

#define BLE_SCAN_TIME       6       // In seconds
#define BLE_CONNECTION_TIMEOUT  10  // Seconds
#define BLE_GC_NOTIFY_INTERVAL  pdMS_TO_TICKS(5000)    // interval in ticks, 5000 = send every 5 seconds

#define BLE_HR_INIT_OK      0x0001
#define BLE_HR_CONNECTED    0x0002
#define BLE_HR_SCANNING     0x0004
#define BLE_HR_SCAN_DONE    0x0008
#define BLE_HR_NOTIFIED     0x0010
#define BLE_GC_STARTED      0x0020
#define BLE_GC_HAS_CLIENT   0x0040

#define BLE_HR_CONNECT         0x1000
#define BLE_HR_DISCONNECT      0x2000
#define BLE_HR_START_SCAN      0x4000
#define BLE_GC_START           0x0100
#define BLE_GC_STOP            0x0200

// Data structure of Golden Cheetah VO2
typedef struct __attribute__ ((packed)) _goldenCheetah_t {
  uint16_t freq;
  int8_t  temperature;
  uint8_t  humidity;
  uint16_t rmv;
  uint16_t feo2;
  uint16_t vo2;
} goldenCheetah_t;


// Class to handle HR belt connections
class BLE_HRBelt
{
public:
  BLE_HRBelt();

  bool init(void);
  bool connect(void);
  bool connect(BLEAddress address);
  void disconnect(void);

  bool scan(void);

  bool isConnected(void);

  void setAddress(uint8_t *addr);

  static bool connected;

private:
  NimBLEAddress HRSensorAddress;
  NimBLEClient *pClient;
  NimBLEScan *pScan;
  NimBLEScanResults scanResults;
  bool initialized;
};

// Class to handle Golden Cheetah as VO2 Master
class BLE_GC
{
public:
  BLE_GC();
  bool init(void);
  bool startServer(void);
  bool stopServer(void);
  bool notify(void);

  static bool client_connected;
private:
  bool initialized;
  goldenCheetah_t variables;
  sensorData_t sensor_data_buf;
  NimBLEServer *pServer;
  NimBLEService *pService;
  NimBLEDescriptor *pDescriptor;
  NimBLEAdvertising *pAdvertising;
};


void BLETask(void *params);

// HR helper functions
void BLEHRConnect(void);
void BLEHRDisconnect(void);
void BLEHRStartScan(void);
bool BLEHRIsScanning(void);
bool BLEHRIsInitOK(void);
bool BLEHRIsConnected(void);
bool BLEHRIsScanDone(void);
bool BLEHRWasNotified(bool clear);
const char *BLEGetHRName(void);

// Golden Cheetah helper functions
void BLEGCStart(void);
void BLEGCStop(void);
bool BLEGCIsStarted(void);
bool BLEGCHasClient(void);


#endif