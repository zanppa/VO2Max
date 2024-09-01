/*
 * BLE_HRBelt.cpp - Bluetooth Low Energy Heart Rate belt reader
 * Copyright (C) 2024 Lauri Peltonen
 * GPL V3
 *
 * BLE Golden Cheetah service
 * Based on IHewitt's VO2 max code
 * Copyright (C) 2024 Lauri Peltonen
 * GPL V3
 **/


// Bluetooth assigned numbers
// https://www.bluetooth.com/specifications/assigned-numbers-html/

// Heart rate service is 0x180D --> starts with "0000180d"
// Heart rate characteristic is 0x2A37 --> starts with "00002a37"
// 0x2902 is "Client charasteristic configuration", something should be written here in some cases? 

// Battery service, if needed, would be 0x180F
// Where battery level characteristic is 0x2a19

// Heart rate sensor specification
// https://www.bluetooth.com/specifications/specs/heart-rate-profile-1-0/

// byte 0 = flags
//    bit 0 (LSb) = 1 --> data is 16 bits , 0 --> 8 bits
//    bits 1..2   = 3 = sensor contact, 2 = no contact, others = not supported
//    bit 3       = 1 --> "EE status" present(?)
//    bit 4       = 1 --> Heart rate R-R interval (variation?)
// following 1 or 2 bytes are heart rate (LSB first)
// If EE, then 2 bytes of EE, LSB first
// If RR, then 2 bytes of RR, LSB first, divide by 1024 to get value in seconds

// Other references
// https://github.com/fg1/BLEHeartRateLogger/blob/master/BLEHeartRateLogger.py


#include <freertos/FreeRTOS.h>

#include "esp_log.h"

#include "config.h"
#include "sensorTask.h"
#include "BLE_HRBelt.h"


const char *TAG_BLE = "BLE";


// Actual bluetooth class instance
static BLE_HRBelt HRBelt;

// HR Belt service & characteristic we're looking for
// The remote service we wish to connect to.
static NimBLEUUID HRServiceUUID((uint16_t)0x180D);
// The characteristic of the remote service we are interested in.
static NimBLEUUID    HRCharacteristicUUID((uint16_t)0x2A37);

//static NimBLEAddress HRSensorAddress("0c:8c:dc:26:7f:84");
static char HRSensorName[32] = "Unknown";    // Name of the BLE HR device (or address if no name)

bool BLE_HRBelt::connected = false;


// Golden Cheetah service
bool BLE_GC::client_connected = false;
static NimBLEUUID cheetahServiceUUID("00001523-1212-EFDE-1523-785FEABCD123");
static NimBLECharacteristic cheetahCharacteristics(NimBLEUUID("00001524-1212-EFDE-1523-785FEABCD123"), NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ); // | 0


static BLE_GC GoldenCheetah;


EventGroupHandle_t BLEHREvent;


// Callback when the HR belt sends new data
static void notifyCallback(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData,
            size_t length, bool isNotify)
{
  uint16_t hr;
  uint8_t idx;
  int16_t rr;

  if(pBLERemoteCharacteristic->getUUID() != HRCharacteristicUUID) {
    ESP_LOGI(TAG_BLE, "Unknown characteristic: %s", pBLERemoteCharacteristic->getUUID().toString().c_str());
    return;
  }

  // Should always have flags & one byte of HR data at least
  if(length < 2) {
    ESP_LOGW(TAG_BLE, "Too few data bytes for HR message");
    return;
  }

  // Whether the sensor is properly in contact with skin (i.e. sensing)
  //bool contact = (((pData[0] >> 1) & 3) == 3) ? true : false;
  
  hr = pData[1];
  idx = 2;  // Index of EE or RR data

  if((pData[0] & 0x01) && length >= idx+1)  { // 16 bits HR data
    hr += pData[2] << 8;
    idx += 1;
  }

  // Add this data to the sensor list
  sensorAddExtraValue(SENSOR_EXTRA_HR, (float)hr, pdMS_TO_TICKS(10));

  if(pData[0] & 0x08) {
    idx += 2; // EE data, skipped
  }
  if((pData[0] & 0x10) && length >= idx + 2) {
    // RR data
    rr = pData[idx++];
    rr += (int16_t)(pData[idx++] << 8); // In milliseconds

    // Add the value in seconds (or actually in milliseconds?) to data list
    sensorAddExtraValue(SENSOR_EXTRA_RR, (float)rr/1024.0, pdMS_TO_TICKS(10));
  }

  xEventGroupSetBits(BLEHREvent, BLE_HR_NOTIFIED);
}

class HRClientCallback : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pclient) {
  }

  void onDisconnect(NimBLEClient* pclient) {
    BLE_HRBelt::connected = false;
  }
};


// Constructor
BLE_HRBelt::BLE_HRBelt() : pClient{nullptr}, pScan{nullptr}, initialized{false} { }

// Connect to default address
bool BLE_HRBelt::connect()
{
  return connect(HRSensorAddress);
}

// Connect to HR belt device with a given address
// Store the address for later use
bool BLE_HRBelt::connect(NimBLEAddress address)
{
  if(connected) return true;  // true = already connected

  if(!initialized)
    if(!init())
      return false;

  ESP_LOGI(TAG_BLE, "Connect: Connecting to %s", address.toString().c_str());

  // Connect to the remove BLE Server.
  pClient->connect(address, true);  // Delete previous attributes that may have been existing
  // pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise) // TODO: What's this?

  vTaskDelay(pdMS_TO_TICKS(300));

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(HRServiceUUID);
  if (pRemoteService == nullptr) {
    ESP_LOGE(TAG_BLE, "Connect: Service UUID %s not found!", HRServiceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }

  vTaskDelay(pdMS_TO_TICKS(300));

  // Obtain a reference to the characteristic in the service of the remote BLE server.
  BLERemoteCharacteristic* pRemoteCharacteristic = pRemoteService->getCharacteristic(HRCharacteristicUUID);
  if (pRemoteCharacteristic == nullptr) {
    ESP_LOGE(TAG_BLE, "Connect: Characteristic UUID %s not found!", HRCharacteristicUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }

  vTaskDelay(pdMS_TO_TICKS(300));

  if(pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(notifyCallback);
  } else {
    ESP_LOGE(TAG_BLE, "Connect: BLE service does not support notifications!");
    pClient->disconnect();
    return false;
  }

  vTaskDelay(pdMS_TO_TICKS(300));

  // Copy address
  HRSensorAddress = NimBLEAddress(address);
  memcpy(global_settings.HRSensorAddress, HRSensorAddress.getNative(), 6);

  ESP_LOGI(TAG_BLE, "Connect: Connected");

  connected = true;
  xEventGroupSetBits(BLEHREvent, BLE_HR_CONNECTED);

  return true;
}

// Scan for suitable devices, and try to connect
// to those that seem to have necessary capabilities
bool BLE_HRBelt::scan(void)
{
  bool result = false;

  xEventGroupClearBits(BLEHREvent, BLE_HR_SCAN_DONE);
  xEventGroupSetBits(BLEHREvent, BLE_HR_SCANNING);

  if(!pClient || !initialized)
    if(!init())
      return false;

  if(!pScan) pScan = NimBLEDevice::getScan();

  ESP_LOGI(TAG_BLE, "Scan: Starting scan");

  // Need to disconnect before scanning
  disconnect();

  // TODO: No idea what interval and window are...
  pScan->setInterval(1349);
  pScan->setWindow(449);
  pScan->setActiveScan(true);


  scanResults = pScan->start(BLE_SCAN_TIME, false);  // Blocking scan

  ESP_LOGI(TAG_BLE, "Scan: Scan done");
  vTaskDelay(pdMS_TO_TICKS(500));

  // Go through the results and try to connect to suitable ones
  // First to connect succesfully will be our device

  // If we got a "disconnect" command, the scan was canceled so do not try to connect

  if(!(xEventGroupGetBits(BLEHREvent) & BLE_HR_DISCONNECT))
  {
    for(int i=0; i<scanResults.getCount(); i++)
    {
      vTaskDelay(pdMS_TO_TICKS(500)); // TODO: Try to prevent watchdog from reseting the device...

      if(scanResults.getDevice(i).haveServiceUUID() && scanResults.getDevice(i).isAdvertisingService(HRServiceUUID)) {
        // The device should have the service we need --> try to connect

        if(connect(scanResults.getDevice(i).getAddress())) {
          // Succesfully connected
          ESP_LOGI(TAG_BLE, "Scan: Connected");
          result = true;
          
          // Store the name of the device
          strncpy(HRSensorName, scanResults.getDevice(i).getName().c_str(), 31);
          if(strlen(HRSensorName) < 1) // No name, use address instead
            strncpy(HRSensorName, scanResults.getDevice(i).getAddress().toString().c_str(), 31);

          break;
        } // Otherwise try next device...
      }
    }
  }

  xEventGroupClearBits(BLEHREvent, BLE_HR_SCANNING);
  xEventGroupClearBits(BLEHREvent, BLE_HR_CONNECT);
  xEventGroupSetBits(BLEHREvent, BLE_HR_SCAN_DONE);

  ESP_LOGI(TAG_BLE, "Scan: End");

  return result;
}


// Disconnect from a connected BLE device
void BLE_HRBelt::disconnect(void)
{
  if(!pClient) return;
  pClient->disconnect();
  connected = false;
  xEventGroupClearBits(BLEHREvent, BLE_HR_CONNECTED);
}

// Setup the BLE Device
bool BLE_HRBelt::init(void)
{
  if(initialized || BLEGCHasClient()) return false;

  if(!BLEDevice::getInitialized())
    BLEDevice::init("VO2Max");

  pClient = BLEDevice::createClient();

  pClient->setConnectTimeout(BLE_CONNECTION_TIMEOUT);
  pClient->setClientCallbacks(new HRClientCallback());

  connected = false;
  initialized = true;
  xEventGroupSetBits(BLEHREvent, BLE_HR_INIT_OK);

  return true;
}

bool BLE_HRBelt::isConnected(void)
{
  return connected;
}

void BLE_HRBelt::setAddress(uint8_t *addr)
{
  HRSensorAddress = NimBLEAddress(addr);
  ESP_LOGI(TAG_BLE, "Init: Address set to %s", HRSensorAddress.toString().c_str());
}


// Golden Cheetah class implementation
class GCServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) { BLE_GC::client_connected = true; xEventGroupSetBits(BLEHREvent, BLE_GC_HAS_CLIENT); };
    void onDisconnect(BLEServer *pServer) { BLE_GC::client_connected = false; xEventGroupClearBits(BLEHREvent, BLE_GC_HAS_CLIENT); }
};

BLE_GC::BLE_GC() : variables{0}, initialized{false}, pServer{nullptr}, pService{nullptr}, pDescriptor{nullptr}, pAdvertising{nullptr}  { } 

// Initialize BLE
bool BLE_GC::init()
{
  if(initialized || BLEHRIsConnected()) return false;

  if(!BLEDevice::getInitialized())
    BLEDevice::init("VO2Max");

  if(!pServer) {
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new GCServerCallbacks());
  }

  if(!pDescriptor) {
    pDescriptor = cheetahCharacteristics.createDescriptor(NimBLEUUID((uint16_t)0x2901), NIMBLE_PROPERTY::READ);
    //pDescriptor = new NimBLEDescriptor(NimBLEUUID((uint16_t)0x2901), NIMBLE_PROPERTY::READ);
    //cheetahCharacteristics.addDescriptor(pDescriptor);
    pDescriptor->setValue("VO2 Data");
    //cheetahCharacteristics.addDescriptor(new NimBLE2902());   // 0x2902 is automatic when NOTIFY is enabled in the characteristics
  }
  if(!pService) {
    pService = pServer->createService(cheetahServiceUUID);
    pService->addCharacteristic(&cheetahCharacteristics);
  }
  if(!pAdvertising) {
    pAdvertising = pServer->getAdvertising();
    pAdvertising->addServiceUUID(pService->getUUID());
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
  }

  initialized = true;
  return true;
}

bool BLE_GC::startServer()
{
  if(!initialized)
    if(!init())
      return false;

  if(pService) {
    pService->start();
    BLEDevice::startAdvertising();
    xEventGroupSetBits(BLEHREvent, BLE_GC_STARTED);
    return true;
  }
  return false;
}

bool BLE_GC::stopServer()
{
  BLEDevice::stopAdvertising();
  // TODO: Disconnect clients?
  xEventGroupClearBits(BLEHREvent, BLE_GC_STARTED);
  return true;
}

bool BLE_GC::notify()
{
  if(!initialized || !client_connected) return false;

  // Copy actual sensor data
  if(!sensorGetData(&sensor_data_buf)) return false;

  // Fill the struct correctly
  // TODO: Need to check whether these are actually correct values the software expects
  variables.freq = 0;  // ?
  variables.temperature = sensor_data_buf.ambient_temperature;
  variables.humidity = 0; // Not implemented (yet?)
  variables.rmv = sensor_data_buf.veMean;
  variables.feo2 = sensor_data_buf.o2 * 100;
  variables.vo2 = sensor_data_buf.vo2;

  cheetahCharacteristics.setValue((uint8_t *)&variables, sizeof(variables));
  cheetahCharacteristics.notify();
}


static TickType_t lastNotifyTick = 0;
static TickType_t currentTick = 0;

// Main bluetooth task
void BLETask(void *params)
{
  EventBits_t bits;

  BLEHREvent = xEventGroupCreate();
  configASSERT(BLEHREvent); // DEBUG

  configASSERT(sizeof(goldenCheetah_t) == 10);  // Debug, make sure the struct is correctly packed

  HRBelt.setAddress(global_settings.HRSensorAddress); // Set stored (default) address

  if(global_settings.cheetah_enable)
    GoldenCheetah.init();

  if(global_settings.hrsensor_enable)
    HRBelt.init();

  lastNotifyTick = xTaskGetTickCount();
  for(;;)
  {
    // Send Golden Cheetah packet every now and then
    currentTick = xTaskGetTickCount();
    if((currentTick - lastNotifyTick) >= BLE_GC_NOTIFY_INTERVAL && (xEventGroupGetBits(BLEHREvent) & BLE_GC_HAS_CLIENT)) {
      GoldenCheetah.notify();
      lastNotifyTick = currentTick;
    }

    vTaskDelay(pdMS_TO_TICKS(500));
    bits = xEventGroupWaitBits(BLEHREvent, 0xFF00, true, false, pdMS_TO_TICKS(500));  // Also clears the bits once handled

    if(bits & BLE_HR_START_SCAN) {
      HRBelt.scan();
      //xEventGroupClearBits(BLEHREvent, BLE_HR_START_SCAN);
    }
    if(bits & BLE_HR_CONNECT) {
      if(!HRBelt.connect())
        xEventGroupClearBits(BLEHREvent, BLE_HR_CONNECTED);
      //xEventGroupClearBits(BLEHREvent, BLE_HR_CONNECT);
    }
    if(bits & BLE_HR_DISCONNECT) {
      HRBelt.disconnect();
      //xEventGroupClearBits(BLEHREvent, BLE_HR_DISCONNECT);
    }

    if(bits & BLE_GC_START) {
      GoldenCheetah.startServer();
    }
    if(bits & BLE_GC_STOP) {
      GoldenCheetah.stopServer();
    }
  }
}

// Request to try to connect to default (previously connected) HR belt
void BLEHRConnect(void)
{
  xEventGroupSetBits(BLEHREvent, BLE_HR_CONNECT);
}

// Request disconnection from connected HR belt
void BLEHRDisconnect(void)
{
  xEventGroupSetBits(BLEHREvent, BLE_HR_DISCONNECT);
}

// Request BLE to start scanning for HR belts
void BLEHRStartScan(void)
{
  xEventGroupSetBits(BLEHREvent, BLE_HR_START_SCAN);
}

// Return true if BLE is scanning for suitable devices
bool BLEHRIsScanning(void)
{
  EventBits_t bits = xEventGroupGetBits(BLEHREvent);
  return ((bits & BLE_HR_SCANNING) || (bits & BLE_HR_START_SCAN)) ? true : false;
}

// Check whether BLE has been initialized
bool BLEHRIsInitOK(void)
{
  return (xEventGroupGetBits(BLEHREvent) & BLE_HR_INIT_OK) ? true : false;
}

// Return true if BLE connection to a HR belt is established
bool BLEHRIsConnected(void)
{
  return (xEventGroupGetBits(BLEHREvent) & BLE_HR_CONNECTED) ? true : false;
}

// Return true if scan has been done and is finished
bool BLEHRIsScanDone(void)
{
  return (xEventGroupGetBits(BLEHREvent) & BLE_HR_SCAN_DONE) ? true : false;
}

// Return if notify flag is on, clear the flag if clear==true
bool BLEHRWasNotified(bool clear)
{
  // Wait with timeout set to 0 --> instant return
  return (xEventGroupWaitBits(BLEHREvent, BLE_HR_NOTIFIED, clear, false, 0) & BLE_HR_NOTIFIED) ? true : false;
}

// Return pointer to the HR belt name
const char *BLEGetHRName(void)
{
  return HRSensorName;
}

// Request start of the Golden Cheetah service
void BLEGCStart(void)
{
  xEventGroupSetBits(BLEHREvent, BLE_GC_START);
}

// Request stop of the golden cheetah service
void BLEGCStop(void)
{
  xEventGroupSetBits(BLEHREvent, BLE_GC_STOP);
}

// Return true if Golden Cheetah service is running
bool BLEGCIsStarted(void)
{
  return (xEventGroupGetBits(BLEHREvent) & BLE_GC_STARTED) ? true : false;
}

// Return true if Golden Cheetah client is connected
bool BLEGCHasClient(void)
{
  return (xEventGroupGetBits(BLEHREvent) & BLE_GC_HAS_CLIENT) ? true : false;
}