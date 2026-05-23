#pragma once

#include <ArduinoJson.h>
#include <FS.h>

namespace Config {
    char     mqtt_server[80]   = "example.tld";
    char     mqtt_username[24] = "";
    char     mqtt_password[24] = "";
    uint16_t mqtt_port         = 1883;
    bool     mqtt_secure       = false;   // TLS with cert validation skipped

    void save() {
        JsonDocument json;
        json["mqtt_server"]   = mqtt_server;
        json["mqtt_username"] = mqtt_username;
        json["mqtt_password"] = mqtt_password;
        json["mqtt_port"]     = mqtt_port;
        json["mqtt_secure"]   = mqtt_secure;

        File configFile = SPIFFS.open("/config.json", "w");
        if (!configFile) {
            return;
        }

        serializeJson(json, configFile);
        configFile.close();
    }

    void load() {
        if (SPIFFS.begin()) {

            if (SPIFFS.exists("/config.json")) {
                File configFile = SPIFFS.open("/config.json", "r");

                if (configFile) {
                    const size_t size = configFile.size();
                    std::unique_ptr<char[]> buf(new char[size]);

                    configFile.readBytes(buf.get(), size);
                    JsonDocument json;

                    if (DeserializationError::Ok == deserializeJson(json, buf.get())) {
                        strcpy(mqtt_server, json["mqtt_server"]);
                        strcpy(mqtt_username, json["mqtt_username"]);
                        strcpy(mqtt_password, json["mqtt_password"]);
                        if (json["mqtt_port"].is<uint16_t>()) {
                            mqtt_port = json["mqtt_port"].as<uint16_t>();
                        }
                        if (json["mqtt_secure"].is<bool>()) {
                            mqtt_secure = json["mqtt_secure"].as<bool>();
                        }
                    }
                }
            }
        }
    }
} // namespace Config
