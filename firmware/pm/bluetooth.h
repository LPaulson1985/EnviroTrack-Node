#include <BLEServer.h>
#include <BLEDevice.h>

// List of Service and Characteristic UUIDs
#define SERVICE_UUID  "012d2753-8b26-479b-97a9-fcaf9be13bb8"
#define WIFI_UUID     "00002753-8b26-479b-97a9-fcaf9be13bb8"

/** Characteristic for digital output */
extern BLECharacteristic *pCharacteristicWiFi;

void initBLE();

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer);
  void onDisconnect(BLEServer* pServer);
};

class MyCallbackHandler: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic);
  void onRead(BLECharacteristic *pCharacteristic);
};
