/*
 * Watermeter sensor (Proximity Sensor)
 * (c) 2022 PA1DVB
 * Rev. 1.2 - 25 Feb 2022
 * Rev. 1.3 - 7 Mrt 2022, Added TEST_ONLY flag
 * Rev. 1.4 - Timestamp in state message
 * Rev. 1.5 - 19 Jan 2025
 * Rev. 1.6 - 23 May 2026, WiFi auto-reconnect, non-blocking MQTT retry, fetch-poll web UI
 * Rev. 1.7 - 23 May 2026, Lifetime pulse/m3 counter and dark-themed status dashboard
 */
 
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecureBearSSL.h>
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
WiFiClientSecure wifiClientSecure;
PubSubClient mqttClient;
ESP8266WebServer webServer(80);

WiFiManagerParameter custom_mqtt_server("server", "MQTT Server", Config::mqtt_server, sizeof(Config::mqtt_server));
WiFiManagerParameter custom_mqtt_user("user", "MQTT Username", Config::mqtt_username, sizeof(Config::mqtt_username));
WiFiManagerParameter custom_mqtt_pass("pass", "MQTT Password", Config::mqtt_password, sizeof(Config::mqtt_password));

// Port and TLS need their own string-backed buffers because they aren't char arrays in Config.
char mqtt_port_str[6]   = "1883";
char mqtt_secure_str[2] = "0";
WiFiManagerParameter custom_mqtt_port  ("port",   "MQTT Port (default 1883)", mqtt_port_str,   sizeof(mqtt_port_str));
WiFiManagerParameter custom_mqtt_secure("secure", "Use TLS? 1=yes, 0=no",     mqtt_secure_str, sizeof(mqtt_secure_str));

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
uint16_t pulse_counts = 0;        // pulses pending publish (cleared on successful MQTT publish)
uint32_t total_pulses = 0;        // lifetime pulses since boot (never reset)
uint32_t last_pulse_millis = 0;   // millis() of most recent pulse, 0 if none yet

char identifier[24];

#define app_version "2026.05.23 rev 1.7"
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

// Bind the right TCP client to PubSubClient based on the current secure flag,
// then apply the configured server + port. Call after changing any of those.
void applyMqttClient() {
  if (Config::mqtt_secure) {
    wifiClientSecure.setInsecure();   // skip cert validation per design
    mqttClient.setClient(wifiClientSecure);
  } else {
    mqttClient.setClient(wifiClient);
  }
  mqttClient.setServer(Config::mqtt_server, Config::mqtt_port);
}

String formatUptime(uint32_t ms) {
  uint32_t s = ms / 1000;
  uint32_t d = s / 86400;  s %= 86400;
  uint32_t h = s / 3600;   s %= 3600;
  uint32_t m = s / 60;     s %= 60;
  char buf[32];
  if (d > 0) snprintf(buf, sizeof(buf), "%lud %02lu:%02lu:%02lu", (unsigned long)d, (unsigned long)h, (unsigned long)m, (unsigned long)s);
  else       snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", (unsigned long)h, (unsigned long)m, (unsigned long)s);
  return String(buf);
}

String formatAgo(uint32_t deltaMs) {
  uint32_t s = deltaMs / 1000;
  if (s < 60)    return String(s) + "s ago";
  if (s < 3600)  return String(s / 60) + "m " + String(s % 60) + "s ago";
  if (s < 86400) return String(s / 3600) + "h " + String((s % 3600) / 60) + "m ago";
  return String(s / 86400) + "d " + String((s % 86400) / 3600) + "h ago";
}

void handleWebRoot() {
  String h;
  h.reserve(4096);
  h += F("<!DOCTYPE html><html><head><meta charset=\"utf-8\">");
  h += F("<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">");
  h += F("<title>Watermeter</title>");
  h += F("<link rel=\"icon\" href=\"data:,\">");
  h += F("<style>");
  h += F(":root{--bg:#0e1626;--card:#172033;--card2:#1f2a44;--fg:#e6ecf5;--mut:#8a98b3;--acc:#5ad1ff;--ok:#54e09d;--off:#ff7a7a;--lo:#7fbcff;}");
  h += F("*{box-sizing:border-box}html,body{margin:0;padding:0;background:var(--bg);color:var(--fg);font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Helvetica,Arial,sans-serif;line-height:1.4}");
  h += F(".wrap{max-width:760px;margin:0 auto;padding:24px 16px}");
  h += F("h1{margin:0 0 4px 0;font-size:1.6rem;letter-spacing:.5px}h1 span{color:var(--acc)}");
  h += F(".sub{color:var(--mut);font-size:.9rem;margin-bottom:20px}");
  h += F(".card{background:var(--card);border-radius:10px;padding:16px 18px;margin-bottom:16px;box-shadow:0 1px 3px rgba(0,0,0,.3)}");
  h += F(".card h2{margin:0 0 12px 0;font-size:1rem;color:var(--acc);text-transform:uppercase;letter-spacing:1px}");
  h += F(".big{font-size:2.8rem;font-weight:700;color:var(--ok);text-align:center;margin:8px 0 2px 0;font-variant-numeric:tabular-nums}");
  h += F(".big .u{font-size:1rem;color:var(--mut);font-weight:400;margin-left:6px}");
  h += F(".sm{text-align:center;color:var(--mut);font-size:.85rem;margin-bottom:6px}");
  h += F(".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:10px}");
  h += F(".stat{background:var(--card2);padding:10px 12px;border-radius:8px}");
  h += F(".stat .k{color:var(--mut);font-size:.75rem;text-transform:uppercase;letter-spacing:.5px}");
  h += F(".stat .v{font-size:1.05rem;font-weight:600;margin-top:2px;word-break:break-all;font-variant-numeric:tabular-nums}");
  h += F(".pin{display:inline-block;padding:2px 10px;border-radius:12px;font-size:.8rem;font-weight:600}");
  h += F(".pin.HIGH{background:#1e3a55;color:var(--lo)}.pin.LOW{background:#163826;color:var(--ok)}.pin.off{background:#2a2f44;color:var(--mut)}");
  h += F(".foot{color:var(--mut);font-size:.75rem;text-align:center;margin-top:16px}");
  h += F(".pulse{display:inline-block;width:8px;height:8px;border-radius:50%;background:var(--ok);margin-right:6px;vertical-align:middle;animation:p 2s ease-in-out infinite}");
  h += F("@keyframes p{0%,100%{opacity:1}50%{opacity:.3}}");
  h += F("</style></head><body><div class=\"wrap\">");

  h += F("<h1><span>Water</span>meter</h1>");
  h += F("<div class=\"sub\"><span class=\"pulse\"></span><span id=\"live\">live</span> &middot; v");
  h += String(app_version);
  h += F("</div>");

  h += F("<div class=\"card\"><h2>Lifetime usage (since boot)</h2>");
  h += F("<div class=\"big\"><span id=\"m3\">0.000</span><span class=\"u\">m&sup3;</span></div>");
  h += F("<div class=\"sm\"><span id=\"liters\">0</span> L &middot; <span id=\"pulses\">0</span> pulses</div></div>");

  h += F("<div class=\"card\"><h2>Sensor</h2><div class=\"grid\">");
  h += F("<div class=\"stat\"><div class=\"k\">GPIO state</div><div class=\"v\"><span id=\"st\" class=\"pin off\">--</span></div></div>");
  h += F("<div class=\"stat\"><div class=\"k\">Last pulse</div><div class=\"v\" id=\"last\">--</div></div>");
  h += F("<div class=\"stat\"><div class=\"k\">Pending publish</div><div class=\"v\" id=\"pend\">--</div></div>");
  h += F("</div></div>");

  h += F("<div class=\"card\"><h2>System</h2><div class=\"grid\">");
  h += F("<div class=\"stat\"><div class=\"k\">Hostname</div><div class=\"v\" id=\"host\">--</div></div>");
  h += F("<div class=\"stat\"><div class=\"k\">IP</div><div class=\"v\" id=\"ip\">--</div></div>");
  h += F("<div class=\"stat\"><div class=\"k\">RSSI</div><div class=\"v\" id=\"rssi\">--</div></div>");
  h += F("<div class=\"stat\"><div class=\"k\">MQTT</div><div class=\"v\" id=\"mqtt\">--</div></div>");
  h += F("<div class=\"stat\"><div class=\"k\">MQTT broker</div><div class=\"v\" id=\"mqtt_srv\" style=\"font-size:.85rem\">--</div></div>");
  h += F("<div class=\"stat\"><div class=\"k\">Uptime</div><div class=\"v\" id=\"up\">--</div></div>");
#ifdef TIME_SERVER
  h += F("<div class=\"stat\"><div class=\"k\">UTC time</div><div class=\"v\" id=\"tm\">--</div></div>");
#endif
  h += F("</div></div>");

  h += F("<div class=\"foot\">Each pulse = 1&nbsp;L &middot; <a href=\"/mqtt\" style=\"color:var(--mut)\">configure MQTT</a> &middot; <a href=\"/reset\" style=\"color:var(--mut)\">factory reset</a></div></div>");

  h += F("<script>");
  h += F("function esc(s){return String(s).replace(/[&<>\"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','\"':'&quot;',\"'\":'&#39;'}[c]));}");
  h += F("async function poll(){try{const r=await fetch('/status');const j=await r.json();");
  h += F("document.getElementById('m3').textContent=(j.total/1000).toFixed(3);");
  h += F("document.getElementById('liters').textContent=j.total;");
  h += F("document.getElementById('pulses').textContent=j.total;");
  h += F("const stE=document.getElementById('st');stE.textContent=j.state;stE.className='pin '+(j.state||'off');");
  h += F("document.getElementById('last').textContent=j.last_pulse||'never';");
  h += F("document.getElementById('pend').textContent=j.pending+' L';");
  h += F("document.getElementById('host').textContent=j.host;");
  h += F("document.getElementById('ip').textContent=j.ip;");
  h += F("document.getElementById('rssi').textContent=j.rssi+' dBm';");
  h += F("const sMap={'-4':'timeout','-3':'lost','-2':'connect failed','-1':'disconnected','0':'connected','1':'bad protocol','2':'bad client id','3':'unavailable','4':'bad auth','5':'unauthorized'};");
  h += F("const mE=document.getElementById('mqtt');mE.textContent=j.mqtt?'connected':('disconnected ('+(sMap[j.mqtt_state]||j.mqtt_state)+')');");
  h += F("mE.style.color=j.mqtt?'var(--ok)':'var(--off)';");
  h += F("document.getElementById('mqtt_srv').textContent=j.mqtt_server||'(not set)';");
  h += F("document.getElementById('up').textContent=j.uptime;");
#ifdef TIME_SERVER
  h += F("if(j.time)document.getElementById('tm').textContent=j.time;");
#endif
  h += F("document.getElementById('live').textContent='live';");
  h += F("}catch(e){document.getElementById('live').textContent='offline';}}");
  h += F("poll();setInterval(poll,3000);");
  h += F("</script></body></html>");

  webServer.send(200, "text/html", h);
}

void handleWebStatus() {
  JsonDocument json;
  json["host"]    = identifier;
  json["ip"]      = WiFi.localIP().toString();
  json["rssi"]    = WiFi.RSSI();
  json["mqtt"]    = mqttClient.connected();
  json["mqtt_server"] = String(Config::mqtt_server) + ":" + String(Config::mqtt_port) + (Config::mqtt_secure ? " (TLS)" : "");
  json["mqtt_state"]  = mqttClient.state();
  json["state"]   = output4State;
  json["total"]   = total_pulses;
  json["pending"] = pulse_counts;
  json["uptime"]  = formatUptime(millis());
  json["last_pulse"] = (last_pulse_millis == 0) ? String("never") : formatAgo(millis() - last_pulse_millis);
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

void handleMqttConfig() {
  String h;
  h.reserve(2048);
  h += F("<!DOCTYPE html><html><head><meta charset=\"utf-8\">");
  h += F("<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">");
  h += F("<title>MQTT config</title><link rel=\"icon\" href=\"data:,\">");
  h += F("<style>:root{--bg:#0e1626;--card:#172033;--card2:#1f2a44;--fg:#e6ecf5;--mut:#8a98b3;--acc:#5ad1ff;--ok:#54e09d;}");
  h += F("*{box-sizing:border-box}html,body{margin:0;padding:0;background:var(--bg);color:var(--fg);font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Helvetica,Arial,sans-serif}");
  h += F(".wrap{max-width:480px;margin:0 auto;padding:24px 16px}");
  h += F("h1{margin:0 0 16px 0;font-size:1.4rem}h1 span{color:var(--acc)}");
  h += F(".card{background:var(--card);border-radius:10px;padding:20px}");
  h += F("label{display:block;color:var(--mut);font-size:.8rem;text-transform:uppercase;letter-spacing:.5px;margin:12px 0 4px}");
  h += F("input{width:100%;padding:10px 12px;background:var(--card2);border:1px solid #2a3148;color:var(--fg);border-radius:6px;font-size:1rem}");
  h += F("input:focus{outline:none;border-color:var(--acc)}");
  h += F("button{margin-top:18px;width:100%;padding:12px;background:var(--ok);color:#0e1626;border:none;border-radius:6px;font-size:1rem;font-weight:600;cursor:pointer}");
  h += F("a{color:var(--mut);font-size:.85rem;display:inline-block;margin-top:12px}");
  h += F("</style></head><body><div class=\"wrap\"><h1><span>MQTT</span> config</h1><div class=\"card\">");
  h += F("<form method=\"POST\" action=\"/mqtt\">");
  h += F("<label>Broker host (IP or hostname)</label>");
  h += F("<input name=\"server\" maxlength=\"79\" value=\"");
  h += String(Config::mqtt_server);
  h += F("\">");
  h += F("<label>Port</label>");
  h += F("<input name=\"port\" type=\"number\" min=\"1\" max=\"65535\" value=\"");
  h += String(Config::mqtt_port);
  h += F("\">");
  h += F("<label style=\"display:flex;align-items:center;gap:8px;text-transform:none;letter-spacing:0;font-size:.9rem;color:var(--fg)\">");
  h += F("<input type=\"checkbox\" name=\"secure\" value=\"1\" style=\"width:auto;margin:0\"");
  if (Config::mqtt_secure) h += F(" checked");
  h += F("> Use TLS (skip certificate validation)</label>");
  h += F("<label>Username (optional)</label>");
  h += F("<input name=\"user\" maxlength=\"23\" value=\"");
  h += String(Config::mqtt_username);
  h += F("\">");
  h += F("<label>Password (leave empty to keep current)</label>");
  h += F("<input name=\"pass\" type=\"password\" maxlength=\"23\" value=\"\">");
  h += F("<button type=\"submit\">Save and reconnect</button></form>");
  h += F("<a href=\"/\">&larr; back to status</a></div></div></body></html>");
  webServer.send(200, "text/html", h);
}

void handleMqttConfigSave() {
  if (webServer.hasArg("server")) {
    strncpy(Config::mqtt_server, webServer.arg("server").c_str(), sizeof(Config::mqtt_server) - 1);
    Config::mqtt_server[sizeof(Config::mqtt_server) - 1] = 0;
  }
  if (webServer.hasArg("port")) {
    long p = webServer.arg("port").toInt();
    if (p >= 1 && p <= 65535) Config::mqtt_port = (uint16_t)p;
  }
  // Checkbox is only sent in the POST when checked; treat absence as off.
  Config::mqtt_secure = webServer.hasArg("secure");
  if (webServer.hasArg("user")) {
    strncpy(Config::mqtt_username, webServer.arg("user").c_str(), sizeof(Config::mqtt_username) - 1);
    Config::mqtt_username[sizeof(Config::mqtt_username) - 1] = 0;
  }
  // Only overwrite the password when a value was actually entered.
  if (webServer.hasArg("pass") && webServer.arg("pass").length() > 0) {
    strncpy(Config::mqtt_password, webServer.arg("pass").c_str(), sizeof(Config::mqtt_password) - 1);
    Config::mqtt_password[sizeof(Config::mqtt_password) - 1] = 0;
  }
  Config::save();

  // Apply the new server and force a fresh connect attempt on the next loop.
  mqttClient.disconnect();
  applyMqttClient();
  lastMqttConnectionAttempt = 0;

  String h;
  h += F("<!DOCTYPE html><html><head><meta charset=\"utf-8\"><meta http-equiv=\"refresh\" content=\"2;url=/\">");
  h += F("<style>body{background:#0e1626;color:#e6ecf5;font-family:-apple-system,sans-serif;text-align:center;padding:40px}</style>");
  h += F("</head><body><h2>Saved.</h2><p>Returning to status page...</p></body></html>");
  webServer.send(200, "text/html", h);
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

    applyMqttClient();
    mqttClient.setKeepAlive(10);
    mqttClient.setBufferSize(2048);
    mqttClient.setCallback(mqttCallback);

    Serial.printf("Hostname: %s\n", identifier);
    Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("MQTT broker: %s:%u (%s)\n", Config::mqtt_server, Config::mqtt_port, Config::mqtt_secure ? "TLS" : "plain");

    // Trigger the first MQTT connect attempt immediately rather than waiting
    // a full mqttConnectionInterval (60s).
    printf("Initial mqtt connect\n");
    mqttReconnect();
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
            total_pulses++;
            last_pulse_millis = millis();
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

    // Pre-fill portal fields from the currently loaded config so the captive
    // portal shows what's actually saved (not the C++ static-init defaults).
    custom_mqtt_server.setValue(Config::mqtt_server, sizeof(Config::mqtt_server));
    custom_mqtt_user  .setValue(Config::mqtt_username, sizeof(Config::mqtt_username));
    custom_mqtt_pass  .setValue(Config::mqtt_password, sizeof(Config::mqtt_password));
    snprintf(mqtt_port_str, sizeof(mqtt_port_str), "%u", Config::mqtt_port);
    custom_mqtt_port  .setValue(mqtt_port_str, sizeof(mqtt_port_str));
    custom_mqtt_secure.setValue(Config::mqtt_secure ? "1" : "0", sizeof(mqtt_secure_str));

    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_port);
    wifiManager.addParameter(&custom_mqtt_user);
    wifiManager.addParameter(&custom_mqtt_pass);
    wifiManager.addParameter(&custom_mqtt_secure);

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

    // Disable WiFi modem sleep. By default the ESP8266 parks the radio between DTIM
    // beacons, adding ~100 ms latency to every web/OTA request when the device is
    // otherwise idle. Costs ~15 mA more idle current. (ESP8266 API: setSleepMode,
    // there is no WiFi.setSleep() as on the ESP32.)
    WiFi.setSleepMode(WIFI_NONE_SLEEP);

    // (applyMqttClient() in setup() binds the right TCP client based on Config::mqtt_secure)

    webServer.on("/", handleWebRoot);
    webServer.on("/status", handleWebStatus);
    webServer.on("/mqtt", HTTP_GET,  handleMqttConfig);
    webServer.on("/mqtt", HTTP_POST, handleMqttConfigSave);
    webServer.on("/reset", handleWebReset);
    webServer.onNotFound(handleWebNotFound);
    webServer.begin();

    strcpy(Config::mqtt_server, custom_mqtt_server.getValue());
    strcpy(Config::mqtt_username, custom_mqtt_user.getValue());
    strcpy(Config::mqtt_password, custom_mqtt_pass.getValue());
    {
      long p = atol(custom_mqtt_port.getValue());
      if (p >= 1 && p <= 65535) Config::mqtt_port = (uint16_t)p;
    }
    Config::mqtt_secure = (custom_mqtt_secure.getValue()[0] == '1');

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
