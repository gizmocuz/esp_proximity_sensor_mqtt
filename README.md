## ESP8266 Proximity sensor (Watermeter)

A small ESP8266 sketch that watches a proximity sensor and reports pulses to MQTT (Home Assistant auto-discovery). Originally built to clamp onto a water meter — each rotation of the meter dial trips the sensor and increments a litre counter.

## First-time setup

On first boot the device exposes a WiFi access point called `WATERMETER-<chipid>`. Connect to it and the captive portal will prompt you for:

| Field | Notes |
|-------|-------|
| WiFi SSID / password | Your home network |
| MQTT Server | Broker host or IP |
| MQTT Port | Default `1883`; use `8883` for TLS |
| MQTT Username / Password | Optional |
| Use TLS? | `1` = TLS with certificate validation skipped, `0` = plain |

All settings persist in SPIFFS (`/config.json`) across reboots and OTA updates.

## Web UI

Once on your LAN, `http://<device-ip>/` shows a dashboard:

- Lifetime usage in m³ (since last boot) and total pulses
- Current GPIO state, time of last pulse, pending pulses awaiting publish
- Hostname, IP, RSSI, MQTT connection state, broker, uptime, UTC time
- Live indicator that flips to *offline* if the device stops responding

Additional endpoints:

| URL | Purpose |
|-----|---------|
| `/status` | JSON status feed (polled by the dashboard every 3 s) |
| `/mqtt`   | Edit broker host / port / username / password / TLS without touching WiFi |
| `/reset`  | Wipe WiFi credentials and reboot into AP mode |

## MQTT

| Topic | Direction | Notes |
|-------|-----------|-------|
| `esp8266-watermeter-sensor/<id>/status` | retained | `online` / `offline` |
| `esp8266-watermeter-sensor/<id>/state`  | publish  | JSON with pulses, wifi info, timestamp |
| `homeassistant/sensor/.../config`       | retained | HA auto-discovery payloads |

Pulses are published every 5 s when non-zero, and only cleared on a successful publish — so pulses are not lost during a broker outage.

## Resilience features

- **WiFi auto-reconnect**: if the router or DNS is down at boot, the device keeps retrying the saved network every 30 s while leaving the captive portal open. After 12 h with no link, it reboots as a safety net.
- **Non-blocking MQTT reconnect**: a single attempt per 60 s tick so ArduinoOTA stays responsive.
- **OTA**: hostname and password both equal the device identifier (e.g. `WATERMETER-A1B2C3`). Find it under *Tools → Port → Network ports* in the Arduino IDE.

## Hardware

- Proximity Sensor NPN 5V — directly to ESP8266 (GND / VIn / D2). ~$5
- ESP8266 NodeMCU. ~$3
- On NodeMCU V3, use the `VU` pin instead of `VIn` to power the sensor.

## Enclosure

3D-printable mounts for various water meter models (Elster V200 in my case):

https://www.thingiverse.com/search?q=proximity+watermeter

I mount with two large screws/nuts smaller than the holes in the meter so the sensor lifts straight off when needed.

## Building

Arduino IDE with the ESP8266 board core. Required libraries (Library Manager):

| Library | Notes |
|---------|-------|
| ArduinoJson (v7+) | JSON config + MQTT payloads |
| ArduinoOTA | OTA updates |
| PubSubClient | MQTT client |
| WiFiManager (tzapu) | Captive portal |

Use it as a boiler template if you wish.
