#include <CStringBuilder.h>   // Install StreamLib

#define ACE_TIME_NTP_CLOCK_DEBUG 1
#include <AceTime.h>          // Install AceTime

#include <ArduinoJson.h>      // Install ArduinoJson
#include <SoftwareSerial.h>   // for circular_queue, install EspSoftwareSerial
#include <ESP_WiFiManager.h>  // Install ESP_WiFiManager
#include <WiFi.h>             // built in
#include <HTTPClient.h>       // Built In
#include <ArduinoOTA.h>       // Install ArdionoOTA
#include <deque>              // built in
#include <Preferences.h>      // Install Preferences
#include <Sds011.h>           // Install Sds011

#include "bluetooth.h"
#include "config.h"

using namespace ace_time;
using namespace ace_time::clock;

static BasicZoneProcessor pacificProcessor;
static NtpClock ntpClock;
SystemClockLoop systemClock(&ntpClock, nullptr /*backup*/);

ESP_WiFiManager ESP_wifiManager;

struct SensorValues {
  double pm25;
  double pm10;
  int numSamples;

  SensorValues(int ns, double p10, double p25) : numSamples(ns), pm10(p10), pm25(p25) {}
};

constexpr int pm_tablesize = 20;
int pm25_table[pm_tablesize];
int pm10_table[pm_tablesize];

#define PIN_LED           2
#define LED_ON      HIGH
  #define LED_OFF     LOW  
#define SP30_COMMS I2C_COMMS
#define DEBUG 0

enum States {
  IDLE,
  WIFI_CONNECTING,
  WARMUP,
  READING,
  RECONNECTING,
  WIFI_WAITING_CREDENTIALS,
  ERRORSTATE
};

// ms from setting one of the states above until the state should transition to the next
uint32_t transitionDelaysMs[] = {
//IDLE   CONN   WARMUP READ   RECON ERROR
  60000, 10000, 10000, 30000, 5000, 99999
};

#define SDS_PIN_RX 16
#define SDS_PIN_TX 17

#ifdef ESP32
HardwareSerial& serialSDS(Serial2);
Sds011Async< HardwareSerial > sds011(serialSDS);
#else
SoftwareSerial serialSDS;
Sds011Async< SoftwareSerial > sds011(serialSDS);
#endif

/** Build time */
const char compileDate[] = __DATE__ " " __TIME__;

States currentState = IDLE;
uint32_t nextStateTime = 0;

// Transistor to control power to the sensor
const uint8_t sensor = 4;

const float vcesat = 0.3;
uint32_t sleepTimeUS = 10 * 60 * 1e6;

const int NUM_SAMPLES = 32;
std::deque<uint16_t> samples;
uint32_t sample_sum = 0;
int sample_index = 0;

WiFiClient espClient;
HTTPClient client;
BLECallbacks bleCallbacks;
String bleGetWifiStatus();
void blePreferencesChanged();
void WifiConnect(String Router_SSID = "", String Router_Pass = "");

void setup() {
  // put your setup code here, to run once:
  pinMode(sensor, OUTPUT);
  Serial.begin(115200);

  bleCallbacks.preferencesChanged = blePreferencesChanged;
  bleCallbacks.getWifiStatus = bleGetWifiStatus;
  setCallbacks(&bleCallbacks);
  initBLE();

  Serial.println("starting wifiManager");
  
  // Now go back to our normal connection.
  setState(WIFI_CONNECTING);

  ntpClock.setup();

  ArduinoOTA.setHostname(host);

   ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() {
      Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.print("Progress: ");
      Serial.print(progress / (total / 100));
      Serial.println("%");
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.print("Error");
      Serial.print(error);
      Serial.print(": ");
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    ArduinoOTA.begin();

  Serial.println("initializing sds011");

 #ifdef ESP32
    serialSDS.begin(9600, SERIAL_8N1, SDS_PIN_RX, SDS_PIN_TX);
    delay(100);
#else
    serialSDS.begin(9600, SWSERIAL_8N1, SDS_PIN_RX, SDS_PIN_TX, false, 192);
#endif
  Sds011::Report_mode report_mode;
  if (!sds011.get_data_reporting_mode(report_mode)) {
      Serial.println("Sds011::get_data_reporting_mode() failed");
  }
  if (Sds011::REPORT_ACTIVE != report_mode) {
      Serial.println("Turning on Sds011::REPORT_ACTIVE reporting mode");
      if (!sds011.set_data_reporting_mode(Sds011::REPORT_ACTIVE)) {
          Serial.println("Sds011::set_data_reporting_mode(Sds011::REPORT_ACTIVE) failed");
      }
  }
}

void loop() {
  ArduinoOTA.handle();
  systemClock.loop();
  if (currentState == WIFI_CONNECTING) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("connected. Local IP: ");
      Serial.println(WiFi.localIP());
      setState(IDLE);
      return;
    } else {
      Serial.print("connecting. Status: ");
      Serial.println(ESP_wifiManager.getStatus(WiFi.status()));
    }
  } else if (currentState == READING) {
    sds011.perform_work();
  }
  if (millis() < nextStateTime && currentState != READING) return;
  switch (currentState) {
    case IDLE:
      Serial.println("Transition to Warmup");
      if (sds011.set_sleep(false))
        Serial.println(F("Measurement started"));
      else {
        Serial.println("Could NOT start measurement");
      setState(ERRORSTATE);
    }
      setState(WARMUP);
      break;
    case WARMUP: 
      Serial.println("Transition to Reading");
      setState(READING); break;
    case READING: {
      Serial.println("Transition to Idle");
      setState(IDLE);
      esp_wifi_stop();
      sds011.set_sleep(true);
      esp_deep_sleep(sleepTimeUS);
      break;
    }
    case RECONNECTING:
      Serial.println("transition to reading");
      setState(READING);
      break;
    case ERRORSTATE:
      break;
    case WIFI_CONNECTING:
      // if we're here, we've timed out. take readings, but don't try to send until we're connected.
      Serial.println("Timeout on wifi connect. Going to AP mode.");
      setState(WIFI_WAITING_CREDENTIALS);
      break;
  }
  nextStateTime = millis() + transitionDelaysMs[currentState];
  Serial.print("current state "); Serial.println(currentState);
  Serial.print("nextStateTime "); Serial.println(nextStateTime);
  Serial.print("current millis "); Serial.println(millis());
}

// CurrentState should be considered readonly except here.
// perhaps this should be an object.
void setState(States newState) {
  if (currentState == newState) return;
  States prevState = currentState;
  currentState = newState;

  switch (newState) 
  {
  case WIFI_CONNECTING:
    getPrefsAndConnect();
    break;
  case WIFI_WAITING_CREDENTIALS:
    //it starts an access point 
    //and goes into a blocking loop awaiting configuration
    if (!ESP_wifiManager.startConfigPortal(apName, apPass)) 
      Serial.println("Failed to start config portal.");
    else 
      Serial.println("Config portal started :)"); 
    break;
  case READING:
    read_all();
    break;
  default:
    break;
  }
}

void send_data(SensorValues val) {
  acetime_t epochSeconds = systemClock.getNow();
  auto pacificTz = TimeZone::forZoneInfo(&zonedb::kZoneAmerica_Los_Angeles,
    &pacificProcessor);
  auto pacificTime = ZonedDateTime::forEpochSeconds(epochSeconds, pacificTz);
  char timeBuf[100];
  CStringBuilder timePrint(timeBuf, sizeof(timeBuf));
  pacificTime.printTo(timePrint);
  // Single reading, and an array of readings
  DynamicJsonDocument readingsDoc(1024);
  JsonArray readings = readingsDoc.to<JsonArray>();
  JsonObject reading = readings.createNestedObject();
  reading["app_key"] = "app";
  reading["net_key"] = "net";
  reading["device_id"] = "esp32-sds011-rmd";
  reading["captured_at"] = String(timeBuf);
  JsonObject channels = reading.createNestedObject("channels");
  channels["ch1"] = String(val.pm25);
  channels["ch2"] = String(val.pm10);
  Serial.print("sending bytes "); serializeJsonPretty(readings, Serial);
  if (client.begin(espClient, apiServer)) {
    client.addHeader("Content-Type", "application/json");
    int status = serializeJson(readings, client.getStream());
    Serial.print("send status "); Serial.println(status);
  } else {
    Serial.println("Failed to connect to server");
  }
  
  Serial.print("Transition to IDLE, deep sleep for");
  Serial.print(sleepTimeUS);
  Serial.println("us");
}

/**
 * @brief : read and display all values
 */
void read_all()
{
  static bool header = true;
  uint8_t ret, error_cnt = 0;

  // loop to get data
  sds011.on_query_data_auto_completed([](int n) {
      Serial.println("Begin Handling SDS011 query data");
      int pm25;
      int pm10;
      Serial.print("n = "); Serial.println(n);
      if (sds011.filter_data(n, pm25_table, pm10_table, pm25, pm10) &&
          !isnan(pm10) && !isnan(pm25)) {
          Serial.print("PM10: ");
          Serial.println(float(pm10) / 10);
          Serial.print("PM2.5: ");
          Serial.println(float(pm25) / 10);
      }
      Serial.println("End Handling SDS011 query data");

      // only print header first time
      if (header) {
        Serial.println(F("------------- Number --------------"));
        Serial.println(F("     Concentration [#/cm3]"));
        Serial.println(F("P2.5\tP10\n"));
        header = false;
      }
      SensorValues sv(n, pm10, pm25);
      send_data(sv);
  });

  if (!sds011.query_data_auto_async(pm_tablesize, pm25_table, pm10_table)) {
      Serial.println("measurement capture start failed");
  }
}

String bleGetWifiStatus() {
  if (currentState == WIFI_WAITING_CREDENTIALS || WiFi.status() != WL_CONNECTED) {
    return "";
  } else if (WiFi.status() != WL_CONNECTED) {
    return WiFi.localIP().toString();
  }
  return "";
}


void WifiConnect(String Router_SSID, String Router_Pass)
{
  //Local intialization. Once its business is done, there is no need to keep it around
  String ssid;

  // We can't use WiFi.SSID() in ESP32as it's only valid after connected. 
  // SSID and Password stored in ESP32 wifi_ap_record_t and wifi_config_t are also cleared in reboot
  // Have to create a new function to store in EEPROM/SPIFFS for this purpose
  if (Router_SSID == "") {
    Router_SSID = ESP_wifiManager.WiFi_SSID();
    Router_Pass = ESP_wifiManager.WiFi_Pass();
  }
  
  //Remove this line if you do not want to see WiFi password printed
  Serial.println("Stored: SSID = " + Router_SSID + ", Pass = " + Router_Pass);

  // SSID to uppercase 
  ssid.toUpperCase();
  
  if (Router_SSID == "")
  {
    Serial.println("We haven't got any access point credentials, so get them now");       
    setState(WIFI_WAITING_CREDENTIALS);
    return;
  }
    
  if ( (WiFi.status() != WL_CONNECTED) )
  {   
    WiFi.mode(WIFI_STA);
    WiFi.persistent (true);
    // We start by connecting to a WiFi network
  
    Serial.print("Connecting to ");
    Serial.println(Router_SSID);
  
    WiFi.begin(Router_SSID.c_str(), Router_Pass.c_str());
    Serial.println("WiFi connecting");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }
}

void blePreferencesChanged() {
  setState(WIFI_CONNECTING);
}
void getPrefsAndConnect() {
  Preferences preferences;
	preferences.begin("WiFiCred", false);
	bool hasPref = preferences.getBool("valid", false);
	if (hasPref) {
		String ssidPrim = preferences.getString("ssidPrim","");
		String ssidSec = preferences.getString("ssidSec","");
		String pwPrim = preferences.getString("pwPrim","");
		String pwSec = preferences.getString("pwSec","");

		if (ssidPrim.equals("") 
				|| pwPrim.equals("")
				/* || ssidSec.equals("")
				|| pwSec.equals("") */) {
			Serial.println("Found preferences but credentials are invalid");
      WifiConnect();
		} else {
			Serial.println("Read from preferences:");
			Serial.println("primary SSID: "+ssidPrim+" password: "+pwPrim);
			Serial.println("secondary SSID: "+ssidSec+" password: "+pwSec);
			hasCredentials = true;
      WifiConnect(ssidPrim, pwPrim);
		}
	} else {
		Serial.println("Could not find preferences, need send data over BLE");
	}
	preferences.end();
}
