/*
 * Watermeter sensor (Proximity Sensor)
 * (c) 2022 PA1DVB
 * Rev. 1.2 - 25 Feb 2022
 * Rev. 1.3 - 7 Mrt 2022, Added TEST_ONLY flag
 * Rev. 1.4 - Timestamp in state message
 * Rev. 1.5 - 19 Jan 2025
 * Rev. 1.6 - 23 May 2026, WiFi auto-reconnect, non-blocking MQTT retry, fetch-poll web UI
 */
 
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiManager.h>

#include "Config.h"

//#define TEST_ONLY

#ifndef TEST_ONLY
  #define TIME_SERVER "nl.pool.ntp.org"
#endif
#ifdef TIME_SERVER
  #include <time.h>
#endif

uint8_t mqttRetryCounter = 0;

#ifndef TEST_ONLY
WiFiManager wifiManager;
WiFiClient wifiClient;
PubSubClient mqttClient;
ESP8266WebServer webServer(80);

WiFiManagerParameter custom_mqtt_server("server", "MQTT Server", Config::mqtt_server, sizeof(Config::mqtt_server));
WiFiManagerParameter custom_mqtt_user("user", "MQTT Username", Config::mqtt_username, sizeof(Config::mqtt_username));
WiFiManagerParameter custom_mqtt_pass("pass", "MQTT Password", Config::mqtt_password, sizeof(Config::mqtt_password));

uint32_t lastMqttConnectionAttempt = 0;
const uint16_t mqttConnectionInterval = 60000; // 1 minute = 60 seconds = 60000 milliseconds

uint32_t lastWifiCheck = 0;
const uint16_t wifiCheckInterval = 30000; // every 30s, retry STA if dropped
#endif

uint32_t statusPublishPreviousMillis = 0;
const uint16_t statusPublishInterval = 5000; // 30 seconds = 30000 milliseconds

uint32_t pinReadPreviousMillis = 0;
const uint16_t pinReadInterval = 10; //100 times a second

#define gpio_input_pin D2 //GPIO4
int gpio_state = 0;
String output4State = "off";
uint16_t pulse_counts = 0;

char identifier[24];

#define app_version "2026.05.23 rev 1.6"
#define FIRMWARE_PREFIX "esp8266-watermeter-sensor"
#define AVAILABILITY_ONLINE "online"
#define AVAILABILITY_OFFLINE "offline"

#ifndef TEST_ONLY
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
  sprintf(szTime,"%04d/%02d/%02d,%02d:%02d:%02d", gtime->tm_year+1900, gtime->tm_mon + 1, gtime->tm_mday, gtime->tm_hour, gtime->tm_min, gtime->tm_sec);
  return String(szTime);
}
#endif

void handleWebRoot() {
  String returnStr;
  AddToStringLF(returnStr, "<!DOCTYPE html><html>");
  AddToStringLF(returnStr, "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
  AddToStringLF(returnStr, "<link rel=\"icon\" href=\"data:,\">");
  AddToStringLF(returnStr, "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
  AddToStringLF(returnStr, ".button { background-color: #195B6A; border: none; color: white; padding: 16px 40px;");
  AddToStringLF(returnStr, "text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
  AddToStringLF(returnStr, ".buttonOn {background-color: #17A2FC;}");
  AddToStringLF(returnStr, ".lgray {color: #ccc;}");
  AddToStringLF(returnStr, "</style></head>");

  AddToStringLF(returnStr, "<body><h1>PA1DVB Watersensor Server</h1>");
  AddToStringLF(returnStr, "<p>GPIO 4 - State <span id=\"st\">--</span></p>");
  AddToStringLF(returnStr, "<p><button id=\"btn\" class=\"button\">--</button></p>");
  AddToStringLF(returnStr, "<p><small class=\"lgray\">version: " + String(app_version) + "</small></p>");
#ifdef TIME_SERVER
  AddToStringLF(returnStr, "<p><small class=\"lgray\">UTC_Time: <span id=\"tm\">--</span></small></p>");
#endif
  AddToStringLF(returnStr, "<script>");
  AddToStringLF(returnStr, "async function poll(){try{const r=await fetch('/status');const j=await r.json();");
  AddToStringLF(returnStr, "document.getElementById('st').textContent=j.state;");
  AddToStringLF(returnStr, "const b=document.getElementById('btn');b.textContent=j.state;");
  AddToStringLF(returnStr, "b.className='button'+(j.state=='HIGH'?' buttonOn':'');");
#ifdef TIME_SERVER
  AddToStringLF(returnStr, "if(j.time)document.getElementById('tm').textContent=j.time;");
#endif
  AddToStringLF(returnStr, "}catch(e){}}");
  AddToStringLF(returnStr, "poll();setInterval(poll,3000);");
  AddToStringLF(returnStr, "</script></body></html>");
  webServer.send(200, "text/html", returnStr);
}

void handleWebStatus() {
  JsonDocument json;
  json["state"] = output4State;
#ifdef TIME_SERVER
  json["time"] = GetLocalTimeString();
#endif
  String out;
  serializeJson(json, out);
  webServer.send(200, "application/json", out);
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
#endif

void setup() {
    Serial.begin(115200);

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
    pinMode(gpio_input_pin, INPUT_PULLUP); //INPUT_PULLUP

    snprintf(identifier, sizeof(identifier), "WATERMETER-%X", ESP.getChipId());
#ifndef TEST_ONLY    
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
#endif
    Serial.println("-- Current GPIO Configuration --");
    Serial.printf("PIN_GPIO_IN: %d\n", gpio_input_pin);
}

void loop() {
    ArduinoOTA.handle();
#ifndef TEST_ONLY    
    mqttClient.loop();
    webServer.handleClient();
#endif
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
#ifdef TEST_ONLY            
            printf("Count erbij!\n");
#endif            
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
#ifndef TEST_ONLY
          // Only clear the counter when we actually got the pulses out the door.
          // Otherwise (no WiFi / no MQTT) we keep accumulating instead of losing them.
          if (mqttClient.connected() && publishState()) {
              pulse_counts = 0;
          } else {
              printf("publish failed, retaining %d pulses\n", pulse_counts);
          }
#else
          pulse_counts = 0;
#endif
        }
    }
#ifndef TEST_ONLY
    // WiFi-drop watchdog: if the link dropped after boot, kick the radio.
    if (currentMillis - lastWifiCheck >= wifiCheckInterval) {
        lastWifiCheck = currentMillis;
        if (WiFi.status() != WL_CONNECTED) {
            printf("WiFi: link down, calling reconnect()\n");
            WiFi.reconnect();
        }
    }
    if (!mqttClient.connected() && currentMillis - lastMqttConnectionAttempt >= mqttConnectionInterval) {
        lastMqttConnectionAttempt = currentMillis;
        if (WiFi.status() == WL_CONNECTED) {
            printf("Reconnect mqtt\n");
            mqttReconnect();
        }
    }
#endif
}

#ifndef TEST_ONLY
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

void setupWifi() {
    // Uncomment and run it once, if you want to erase all the stored information
    //wifiManager.resetSettings();

    wifiManager.setDebugOutput(false);
    wifiManager.setSaveConfigCallback(saveConfigCallback);
    wifiManager.setConfigPortalBlocking(false);
    wifiManager.setConnectTimeout(60);       // retry saved WiFi for 60 s before opening portal
    wifiManager.setConfigPortalTimeout(0);   // keep portal open indefinitely; we retry saved creds in parallel

    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_user);
    wifiManager.addParameter(&custom_mqtt_pass);

    WiFi.hostname(identifier);

    Serial.println("WiFi: connecting...");
    if (!wifiManager.autoConnect(identifier)) {
        // Saved credentials failed — portal is now open. Keep retrying the
        // stored network so we recover automatically when the router/DNS
        // comes back after an outage, without rebooting.
        Serial.println("WiFi: AP portal open, waiting for credentials (retrying saved network every 30s)...");
        uint32_t apStart   = millis();
        uint32_t lastRetry = millis();
        while (WiFi.status() != WL_CONNECTED) {
            wifiManager.process();
            ArduinoOTA.handle();   // keep OTA reachable while we're stuck here
            webServer.handleClient();
            delay(20);
            yield();
            if (millis() - lastRetry > 30000UL) {
                lastRetry = millis();
                Serial.println("WiFi: retrying saved credentials...");
                WiFi.reconnect();
            }
            // Safety net: if the portal has been open for >12 h reboot.
            if (millis() - apStart > 12UL * 60UL * 60UL * 1000UL) {
                Serial.println("WiFi: AP open >12h with no connection, rebooting...");
                ESP.restart();
            }
        }
    }
    mqttClient.setClient(wifiClient);

    webServer.on("/", handleWebRoot);
    webServer.on("/status", handleWebStatus);
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
    // Single non-blocking attempt; the outer loop retries every mqttConnectionInterval ms,
    // which keeps ArduinoOTA / web server responsive between attempts.
    if (mqttClient.connect(identifier, Config::mqtt_username, Config::mqtt_password,
                           MQTT_TOPIC_AVAILABILITY, 1, true, AVAILABILITY_OFFLINE)) {
        mqttClient.publish(MQTT_TOPIC_AVAILABILITY, AVAILABILITY_ONLINE, true);
        publishAutoConfig();
        // Subscribe after auto-config so we never execute commands with default data
        mqttClient.subscribe(MQTT_TOPIC_COMMAND);
    } else {
        printf("MQTT connect failed (state=%d)\n", mqttClient.state());
    }
}

bool publishState() {
    JsonDocument stateJson;
    char payload[256];

    JsonObject wifi = stateJson["wifi"].to<JsonObject>();
    wifi["ssid"] = WiFi.SSID();
    wifi["ip"]   = WiFi.localIP().toString();
    wifi["rssi"] = WiFi.RSSI();

    stateJson["pulses"] = pulse_counts;
#ifdef TIME_SERVER
    stateJson["timestamp"] = time(nullptr);
#endif

    serializeJson(stateJson, payload);
    return mqttClient.publish(MQTT_TOPIC_STATE, payload, false);
}

void mqttCallback(char* topic, uint8_t* payload, unsigned int length) { }

void publishAutoConfig() {
    char mqttPayload[2048];
    JsonDocument device;
    JsonDocument autoconfPayload;

    JsonArray identifiers = device["identifiers"].to<JsonArray>();
    identifiers.add(identifier);

    device["manufacturer"] = "PA1DVB";
    device["model"] = "WATERSENSOR";
    device["name"] = identifier;
    device["sw_version"] = app_version;

    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["device_class"] = "signal_strength";
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["state_class"] = "measurement";
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
    autoconfPayload["device_class"] = "water";
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["state_class"] = "measurement";
    autoconfPayload["name"] = identifier;
    autoconfPayload["unit_of_measurement"] = "L";
    autoconfPayload["value_template"] = "{{value_json.pulses}}";
    autoconfPayload["unique_id"] = identifier + String("_pulses");
    autoconfPayload["icon"] = "mdi:counter-inc"; //incremental counter
    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_PULSE_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();
}
#endif
