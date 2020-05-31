#include <ArduinoJson.h>
#include <BLEServer.h>
#include <BLEDevice.h>

// List of Service and Characteristic UUIDs
#define SERVICE_UUID  "012d2753-8b26-479b-97a9-fcaf9be13bb8"
#define WIFI_UUID     "00002753-8b26-479b-97a9-fcaf9be13bb8"

void initBLE();
extern bool hasCredentials;

// Delegation methods
class BLECallbacks {
  public:
    // indicates to the caller that preferences have been changed
    void (*preferencesChanged)();
    // requests the wifi status - should return the IP address if connected, or empty if not (or connecting)
    String (*getWifiStatus)();
};
void setCallbacks(BLECallbacks*);

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer);
  void onDisconnect(BLEServer* pServer);
};

class MyCallbackHandler: public BLECharacteristicCallbacks {
  public:
  void onWrite(BLECharacteristic *pCharacteristic);
  void onRead(BLECharacteristic *pCharacteristic);
  /** SSIDs of local WiFi networks */
  String ssidPrim;
  String ssidSec;
  /** Password for local WiFi network */
  String pwPrim;
  String pwSec;
  String ipAddress;
  bool isConnected;
  bool connStatusChanged;
  private:
  /** Buffer for JSON string */
  // MAx size is 51 bytes for frame: 
  // {"ssidPrim":"","pwPrim":"","ssidSec":"","pwSec":""}
  // + 4 x 32 bytes for 2 SSID's and 2 passwords
  StaticJsonDocument<200> jsonBuffer;

};
