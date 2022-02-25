/*
 * Watermeter sensor (Proximity Sensor)
 * (c) 2022 PA1DVB
 * Rev. 1.2 - 25 Feb 2022
 */
 
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiManager.h>

#include "Config.h"

#define TIME_SERVER "nl.pool.ntp.org"
#ifdef TIME_SERVER
  #include <time.h>
#endif

uint8_t mqttRetryCounter = 0;

WiFiManager wifiManager;
WiFiClient wifiClient;
PubSubClient mqttClient;
ESP8266WebServer webServer(80);

WiFiManagerParameter custom_mqtt_server("server", "MQTT Server", Config::mqtt_server, sizeof(Config::mqtt_server));
WiFiManagerParameter custom_mqtt_user("user", "MQTT Username", Config::mqtt_username, sizeof(Config::mqtt_username));
WiFiManagerParameter custom_mqtt_pass("pass", "MQTT Password", Config::mqtt_password, sizeof(Config::mqtt_password));

uint32_t lastMqttConnectionAttempt = 0;
const uint16_t mqttConnectionInterval = 60000; // 1 minute = 60 seconds = 60000 milliseconds

uint32_t statusPublishPreviousMillis = 0;
const uint16_t statusPublishInterval = 5000; // 30 seconds = 30000 milliseconds

uint32_t pinReadPreviousMillis = 0;
const uint16_t pinReadInterval = 10; //100 times a second

const int gpio_input_pin = 4; //GPIO4
int gpio_state = 0;
String output4State = "off";
uint16_t pulse_counts = 0;

char identifier[24];

#define app_version "2022.02.25 rev 1.2"
#define FIRMWARE_PREFIX "esp8266-watermeter-sensor"
#define AVAILABILITY_ONLINE "online"
#define AVAILABILITY_OFFLINE "offline"
char MQTT_TOPIC_AVAILABILITY[128];
char MQTT_TOPIC_STATE[128];
char MQTT_TOPIC_COMMAND[128];

char MQTT_TOPIC_AUTOCONF_WIFI_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_PULSE_SENSOR[128];

bool shouldSaveConfig = false;

void saveConfigCallback() {
    shouldSaveConfig = true;
}

void AddToStringLF(String &mystring, const String &toAdd) {
  mystring += toAdd + "\r\n";
}

#ifdef TIME_SERVER
String GetLocalTimeString()
{
  time_t atime = time(nullptr);
  struct tm *gtime = gmtime(&atime);
  char szTime[40];
  if (gtime->tm_year == 0)
    return "Not synced yet!";
  sprintf(szTime,"%04d/%02d/%02d,%02d:%02d:%02d", gtime->tm_year+1900, gtime->tm_mon, gtime->tm_mday, gtime->tm_hour, gtime->tm_min, gtime->tm_sec);
  return String(szTime);
}
#endif

void handleWebRoot() {
  String returnStr;
  AddToStringLF(returnStr, "<!DOCTYPE html><html>");
  AddToStringLF(returnStr, "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
  AddToStringLF(returnStr, "<meta http-equiv=\"refresh\" content=\"3\" >");
  AddToStringLF(returnStr, "<link rel=\"icon\" href=\"data:,\">");
  // CSS to style the on/off buttons 
  // Feel free to change the background-color and font-size attributes to fit your preferences
  AddToStringLF(returnStr, "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
  AddToStringLF(returnStr, ".button { background-color: #195B6A; border: none; color: white; padding: 16px 40px;");
  AddToStringLF(returnStr, "text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
  AddToStringLF(returnStr, ".buttonOn {background-color: #17A2FC;}");
  AddToStringLF(returnStr, ".lgray {color: #ccc;}");
  AddToStringLF(returnStr, "</style></head>");
  
  // Web Page Heading
  AddToStringLF(returnStr, "<body><h1>PA1DVB Watersensor Server</h1>");
  
  // Display current state, and ON/OFF buttons for GPIO 4  
  AddToStringLF(returnStr, "<p>GPIO 4 - State " + output4State + "</p>");
  // If the output4State is off, it displays the ON button       
  if (output4State=="LOW") {
    AddToStringLF(returnStr, "<p><button class=\"button\">LOW</button></p>");
  } else {
    AddToStringLF(returnStr, "<p><button class=\"button buttonOn\">HIGH</button></p>");
  }
  AddToStringLF(returnStr, "<p><small class=\"lgray\">version: " + String(app_version) + "</small></p>");
#ifdef TIME_SERVER
  AddToStringLF(returnStr, "<p><small class=\"lgray\">UTC_Time: " + GetLocalTimeString() + "</small></p>");
#endif
  AddToStringLF(returnStr, "</body></html>");  
  webServer.send(200, "text/html", returnStr);
}

void handleWebReset() {
  webServer.send(200, "text/plain", "Done! Device is restarting and will be available as a WiFi access point again!");
  resetWifiSettingsAndReboot();
}

void handleWebNotFound() {
  String message = "File Not Found!?\n\n";
  message += "URI: ";
  message += webServer.uri();
  message += "\nMethod: ";
  message += (webServer.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += webServer.args();
  message += "\n";
  for (uint8_t i = 0; i < webServer.args(); i++) {
    message += " " + webServer.argName(i) + ": " + webServer.arg(i) + "\n";
  }
  webServer.send(404, "text/plain", message);
}

void setup() {
    Serial.begin(9600);

    Serial.println("\n");
    Serial.println("Watermeter sensor (c) 2022 PDA1DVB!");
    Serial.printf("App Version: %s\n", app_version);
    Serial.printf("Core Version: %s\n", ESP.getCoreVersion().c_str());
    Serial.printf("Boot Version: %u\n", ESP.getBootVersion());
    Serial.printf("Boot Mode: %u\n", ESP.getBootMode());
    Serial.printf("CPU Frequency: %u MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("Reset reason: %s\n", ESP.getResetReason().c_str());

    delay(3000);

    // initialize the GPIO Input pin for our water sensor
    pinMode(gpio_input_pin, INPUT_PULLUP);

    snprintf(identifier, sizeof(identifier), "WATERMETER-%X", ESP.getChipId());
    WiFi.hostname(identifier);

    snprintf(MQTT_TOPIC_AVAILABILITY, 127, "%s/%s/status", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_STATE, 127, "%s/%s/state", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_COMMAND, 127, "%s/%s/command", FIRMWARE_PREFIX, identifier);

    snprintf(MQTT_TOPIC_AUTOCONF_PULSE_SENSOR, 127, "homeassistant/sensor/%s/%s_pulses/config", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_AUTOCONF_WIFI_SENSOR, 127, "homeassistant/sensor/%s/%s_wifi/config", FIRMWARE_PREFIX, identifier);

    Config::load();

    //After connection it could take some time (mostly less then 1 minute) before time is synced
    configTime(0, 0, TIME_SERVER, "time.nist.gov");
    setenv("TZ", "Central Europe Time-2:00", 0);
    

    setupWifi();
    setupOTA();

    mqttClient.setServer(Config::mqtt_server, 1883);
    mqttClient.setKeepAlive(10);
    mqttClient.setBufferSize(2048);
    mqttClient.setCallback(mqttCallback);

    Serial.printf("Hostname: %s\n", identifier);
    Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());

    Serial.println("-- Current GPIO Configuration --");
    Serial.printf("PIN_GPIO_IN: %d\n", gpio_input_pin);
}

void setupOTA() {
    ArduinoOTA.onStart([]() { Serial.println("Start"); });
    ArduinoOTA.onEnd([]() { Serial.println("\nEnd"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) {
            Serial.println("Auth Failed");
        } else if (error == OTA_BEGIN_ERROR) {
            Serial.println("Begin Failed");
        } else if (error == OTA_CONNECT_ERROR) {
            Serial.println("Connect Failed");
        } else if (error == OTA_RECEIVE_ERROR) {
            Serial.println("Receive Failed");
        } else if (error == OTA_END_ERROR) {
            Serial.println("End Failed");
        }
    });

    ArduinoOTA.setHostname(identifier);

    // This is less of a security measure and more a accidential flash prevention
    ArduinoOTA.setPassword(identifier);
    ArduinoOTA.begin();
}

void loop() {
    ArduinoOTA.handle();
    mqttClient.loop();
    webServer.handleClient();

    const uint32_t currentMillis = millis();

    if (currentMillis - pinReadPreviousMillis >= pinReadInterval) {
        pinReadPreviousMillis = currentMillis;

        // read the state of the water sensor
        int gpioState_ = digitalRead(gpio_input_pin);
        if (gpioState_ != gpio_state) {
          //we got a state change
          gpio_state = gpioState_;
          if (gpio_state == LOW) {
            output4State = "LOW";
//REMOVE COMMENT TO DEBUG            printf("Count erbij!\n");
            pulse_counts++;
          } else {
            output4State = "HIGH";
          }
        }
    }

    if (currentMillis - statusPublishPreviousMillis >= statusPublishInterval) {
        statusPublishPreviousMillis = currentMillis;

        if (pulse_counts > 0) {
          printf("total published pulses: %d!\n", pulse_counts);
          publishState();
          pulse_counts=0;
        }
    }

    if (!mqttClient.connected() && currentMillis - lastMqttConnectionAttempt >= mqttConnectionInterval) {
        lastMqttConnectionAttempt = currentMillis;
        printf("Reconnect mqtt\n");
        mqttReconnect();
    }
}

void setupWifi() {
    // Uncomment and run it once, if you want to erase all the stored information
    //wifiManager.resetSettings();
  
    wifiManager.setDebugOutput(false);
    wifiManager.setSaveConfigCallback(saveConfigCallback);

    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_user);
    wifiManager.addParameter(&custom_mqtt_pass);

    WiFi.hostname(identifier);
    wifiManager.autoConnect(identifier);
    mqttClient.setClient(wifiClient);

    webServer.on("/", handleWebRoot);
    webServer.on("/reset", handleWebReset);
    webServer.onNotFound(handleWebNotFound);
    webServer.begin();

    strcpy(Config::mqtt_server, custom_mqtt_server.getValue());
    strcpy(Config::mqtt_username, custom_mqtt_user.getValue());
    strcpy(Config::mqtt_password, custom_mqtt_pass.getValue());

    if (shouldSaveConfig) {
        Config::save();
    } else {
        // For some reason, the read values get overwritten in this function
        // To combat this, we just reload the config
        // This is most likely a logic error which could be fixed otherwise
        Config::load();
    }
}

void resetWifiSettingsAndReboot() {
    wifiManager.resetSettings();
    delay(3000);
    ESP.restart();
}

void mqttReconnect() {
    for (uint8_t attempt = 0; attempt < 3; ++attempt) {
        if (mqttClient.connect(identifier, Config::mqtt_username, Config::mqtt_password, MQTT_TOPIC_AVAILABILITY, 1, true, AVAILABILITY_OFFLINE)) {
            mqttClient.publish(MQTT_TOPIC_AVAILABILITY, AVAILABILITY_ONLINE, true);
            publishAutoConfig();

            // Make sure to subscribe after polling the status so that we never execute commands with the default data
            mqttClient.subscribe(MQTT_TOPIC_COMMAND);
            break;
        }
        delay(5000);
    }
}

void publishState() {
    DynamicJsonDocument wifiJson(192);
    DynamicJsonDocument stateJson(604);
    char payload[256];

    wifiJson["ssid"] = WiFi.SSID();
    wifiJson["ip"] = WiFi.localIP().toString();
    wifiJson["rssi"] = WiFi.RSSI();

    stateJson["pulses"] = pulse_counts;

    stateJson["wifi"] = wifiJson.as<JsonObject>();

    serializeJson(stateJson, payload);
    mqttClient.publish(&MQTT_TOPIC_STATE[0], &payload[0], false);
}

void mqttCallback(char* topic, uint8_t* payload, unsigned int length) { }

void publishAutoConfig() {
    char mqttPayload[2048];
    DynamicJsonDocument device(256);
    DynamicJsonDocument autoconfPayload(1024);
    StaticJsonDocument<64> identifiersDoc;
    JsonArray identifiers = identifiersDoc.to<JsonArray>();

    identifiers.add(identifier);

    device["identifiers"] = identifiers;
    device["manufacturer"] = "PA1DVB";
    device["model"] = "WATERSENSOR";
    device["name"] = identifier;
    device["sw_version"] = app_version;

    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["name"] = identifier + String(" WiFi");
    autoconfPayload["value_template"] = "{{value_json.wifi.rssi}}";
    autoconfPayload["unique_id"] = identifier + String("_wifi");
    autoconfPayload["unit_of_measurement"] = "dBm";
    autoconfPayload["json_attributes_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["json_attributes_template"] = "{\"ssid\": \"{{value_json.wifi.ssid}}\", \"ip\": \"{{value_json.wifi.ip}}\"}";
    autoconfPayload["icon"] = "mdi:wifi";

    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_WIFI_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();

    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["name"] = identifier;
    autoconfPayload["unit_of_measurement"] = "L";
    autoconfPayload["value_template"] = "{{value_json.pulses}}";
    autoconfPayload["unique_id"] = identifier + String("_pulses");
    autoconfPayload["icon"] = "mdi:counter-inc"; //incremental counter
#ifdef TIME_SERVER
    autoconfPayload["timestamp"] = time(nullptr);
#endif
    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_PULSE_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();
}
