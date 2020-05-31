#include <Arduino.h>

// Includes for BLE
#include <BLEUtils.h>
#include <BLEAdvertising.h>
#include <Preferences.h>

#include <nvs.h>
#include <nvs_flash.h>

#include "bluetooth.h"

void createName();

/** BLE Advertiser */
static BLEAdvertising* pAdvertising;
/** BLE Service */
static BLEService *pService;
/** BLE Server */
static BLEServer *pServer;
/** Characteristic for digital output */
static BLECharacteristic *pCharacteristicWiFi;

/** Unique device name */
static char apName[] = "ESP32-xxxxxxxxxxxx";

bool hasCredentials;
static BLECallbacks *callbacks = nullptr;

void setCallbacks(BLECallbacks *newCallbacks) {
  callbacks = newCallbacks;
}
/**
 * MyServerCallbacks
 * Callbacks for client connection and disconnection
 */
  // TODO this doesn't take into account several clients being connected
void MyServerCallbacks::onConnect(BLEServer* pServer) {
  Serial.println("BLE client connected");
};

void MyServerCallbacks::onDisconnect(BLEServer* pServer) {
  Serial.println("BLE client disconnected");
  pAdvertising->start();
}

/**
 * MyCallbackHandler
 * Callbacks for BLE client read/write requests
 */
void MyCallbackHandler::onWrite(BLECharacteristic *pCharacteristic) {
  std::string value = pCharacteristic->getValue();
  if (value.length() == 0) {
    return;
  }
  Serial.println("Received over BLE: " + String((char *)&value[0]));

  // Decode data
  int keyIndex = 0;
  for (int index = 0; index < value.length(); index ++) {
    value[index] = (char) value[index] ^ (char) apName[keyIndex];
    keyIndex++;
    if (keyIndex >= strlen(apName)) keyIndex = 0;
  }

  /** Json object for incoming data */
  DeserializationError success = deserializeJson(jsonBuffer, (char *)&value[0]);
  if (success == DeserializationError::Ok) {
    auto jsonIn = jsonBuffer;
    if (jsonIn.containsKey("ssidPrim") &&
        jsonIn.containsKey("pwPrim") && 
        jsonIn.containsKey("ssidSec") &&
        jsonIn.containsKey("pwSec")) {
      ssidPrim = jsonIn["ssidPrim"].as<String>();
      pwPrim = jsonIn["pwPrim"].as<String>();
      ssidSec = jsonIn["ssidSec"].as<String>();
      pwSec = jsonIn["pwSec"].as<String>();

      Preferences preferences;
      preferences.begin("WiFiCred", false);
      preferences.putString("ssidPrim", ssidPrim);
      preferences.putString("ssidSec", ssidSec);
      preferences.putString("pwPrim", pwPrim);
      preferences.putString("pwSec", pwSec);
      preferences.putBool("valid", true);
      preferences.end();

      Serial.println("Received over bluetooth:");
      Serial.println("primary SSID: "+ssidPrim+" password: "+pwPrim);
      Serial.println("secondary SSID: "+ssidSec+" password: "+pwSec);
      if (callbacks) {
        callbacks->preferencesChanged();
      }
      hasCredentials = true;
    } else if (jsonIn.containsKey("erase")) {
      Serial.println("Received erase command");
      Preferences preferences;
      preferences.begin("WiFiCred", false);
      preferences.clear();
      preferences.end();
      if (callbacks) {
        callbacks->preferencesChanged();
      }
      hasCredentials = false;
      ssidPrim = "";
      pwPrim = "";
      ssidSec = "";
      pwSec = "";

      int err;
      err=nvs_flash_init();
      Serial.println("nvs_flash_init: " + err);
      err=nvs_flash_erase();
      Serial.println("nvs_flash_erase: " + err);
    } else if (jsonIn.containsKey("reset")) {
      esp_restart();
    }
  } else {
    Serial.println("Received invalid JSON");
  }
  jsonBuffer.clear();
}

void MyCallbackHandler::onRead(BLECharacteristic *pCharacteristic) {
  Serial.println("BLE onRead request");
  String wifiCredentials;

  /** Json object for outgoing data */
  StaticJsonDocument<1024> jsonOut;
  jsonOut["ssidPrim"] = ssidPrim;
  jsonOut["pwPrim"] = pwPrim;
  jsonOut["ssidSec"] = ssidSec;
  jsonOut["pwSec"] = pwSec;
  if (callbacks) {
    ipAddress = callbacks->getWifiStatus();
  } else {
    ipAddress = "";
  }
  
  isConnected = (ipAddress.length() > 0);
  jsonOut["connected"] = isConnected;
  if (isConnected) {
    jsonOut["ipAddress"] = ipAddress;
  }
  // Convert JSON object into a string
  serializeJson(jsonOut, wifiCredentials);

  // encode the data
  int keyIndex = 0;
  Serial.println("Stored settings: " + wifiCredentials);
  for (int index = 0; index < wifiCredentials.length(); index ++) {
    wifiCredentials[index] = (char) wifiCredentials[index] ^ (char) apName[keyIndex];
    keyIndex++;
    if (keyIndex >= strlen(apName)) keyIndex = 0;
  }
  pCharacteristicWiFi->setValue((uint8_t*)&wifiCredentials[0],wifiCredentials.length());
  jsonBuffer.clear();
}


/**
 * initBLE
 * Initialize BLE service and characteristic
 * Start BLE server and service advertising
 */
void initBLE() {
  createName();
  
  // Initialize BLE and set output power
  BLEDevice::init(apName);
  BLEDevice::setPower(ESP_PWR_LVL_P7);

  // Create BLE Server
  pServer = BLEDevice::createServer();

  // Set server callbacks
  pServer->setCallbacks(new MyServerCallbacks());

  // Create BLE Service
  pService = pServer->createService(BLEUUID(SERVICE_UUID),20);

  // Create BLE Characteristic for WiFi settings
  pCharacteristicWiFi = pService->createCharacteristic(
    BLEUUID(WIFI_UUID),
    // WIFI_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE
  );
  pCharacteristicWiFi->setCallbacks(new MyCallbackHandler());

  // Start the service
  pService->start();

  // Start advertising
  pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  pAdvertising->start();
}

/**
 * Create unique device name from MAC address
 **/
void createName() {
  uint8_t baseMac[6];
  // Get MAC address for WiFi station
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  // Write unique name into apName
  sprintf(apName, "ESP32-%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
}
