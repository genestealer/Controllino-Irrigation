# Controllino-Irrigation

[![PlatformIO CI](https://github.com/genestealer/Controllino-Irrigation/actions/workflows/build.yml/badge.svg)](https://github.com/genestealer/Controllino-Irrigation/actions/workflows/build.yml)
[![Release](https://github.com/genestealer/Controllino-Irrigation/actions/workflows/release.yml/badge.svg)](https://github.com/genestealer/Controllino-Irrigation/actions/workflows/release.yml)
[![License](https://img.shields.io/github/license/genestealer/Controllino-Irrigation)](LICENSE)
[![GitHub release (latest by date)](https://img.shields.io/github/v/release/genestealer/Controllino-Irrigation)](https://github.com/genestealer/Controllino-Irrigation/releases)
[![PlatformIO](https://img.shields.io/badge/PlatformIO-Ready-blue)](https://platformio.org/)

Ethernet-based irrigation controller running on a **Controllino Maxi** (ATmega 2560 + W5100).
Controls up to **four water solenoid valves** via relay outputs with full
[Home Assistant](https://www.home-assistant.io/) integration through MQTT auto-discovery.

---

## Features

- Controls 4 independent water valves via relay outputs
- MQTT valve entities auto-discovered by Home Assistant (no manual HA configuration)
- Per-valve `open`/`closed` state reporting retained on the broker
- Configurable watchdog timer (0–120 min) prevents valves from running indefinitely; `0` disables the timeout
- NTP time-sync with a scheduled 7-day maintenance reboot (2–4 AM window)
- Non-blocking MQTT reconnect with automatic Ethernet/reboot fallback
- Ethernet DHCP with periodic `Ethernet.maintain()` calls

---

## Hardware

| Component | Details |
|-----------|---------|
| Controller | [Controllino Maxi](https://www.controllino.biz/product/controllino-maxi/) – ATmega 2560 + W5100 Ethernet |
| Power | 12 V PoE Active Splitter (IEEE 802.3af) |
| Valves | 12 V Electric Solenoid Valves |
| Protection | Vishay 1N4001 flyback diodes on each valve |

### Relay outputs

| Screw terminal | Valve |
|----------------|-------|
| `DIGITAL_OUT_11` | Valve 1 |
| `DIGITAL_OUT_10` | Valve 2 |
| `DIGITAL_OUT_09` | Valve 3 |
| `DIGITAL_OUT_08` | Valve 4 |

> Relays can switch higher voltages or provide galvanic isolation if needed.

---

## Getting started

### 1 – Prerequisites

- [PlatformIO](https://platformio.org/) (CLI or VS Code extension)
- MQTT broker (e.g. [Mosquitto](https://mosquitto.org/)) on your local network
- Home Assistant (optional, for GUI control)

### 2 – Create your private configuration

```bash
cp include/Exampleprivate.h include/Private.h
```

Edit `include/Private.h` and fill in:

- `secret_mqtt_server` – your MQTT broker IP/hostname
- `secret_mqtt_username` / `secret_mqtt_password`
- `secret_byte` – a unique MAC address for each controller
- MQTT command and state topics for each valve
- `ntp_server` – NTP server (default: `pool.ntp.org`)

> `Private.h` is listed in `.gitignore` and will never be committed.

#### Optional: legacy combined status JSON

Home Assistant integration uses MQTT auto-discovery, so no manual configuration is needed. For older, manually-configured Home Assistant setups, the firmware can still publish the legacy combined status JSON (`Valve1`-`Valve4` + `WatchdogMinutes`) to `secret_publishNodeStatusJsonTopic`. It is **disabled by default**. To re-enable it, set the following in `Private.h`:

```c
#define ENABLE_LEGACY_MQTT_STATUS_JSON 1
```

When the flag is undefined or `0`, the legacy payload is not published.

### 3 – Select your target controller

Two configurations are supported. Set `IRRIGATION_CONTROLLER` in `platformio.ini` (or use the named environments below):

| Value | Description |
|-------|-------------|
| `1` | Front Garden |
| `2` | Back Garden (default) |

### 4 – Build and upload

```bash
# Build for Back Garden (default)
pio run

# Build for Front Garden
pio run -e front_garden

# Build for Back Garden explicitly
pio run -e back_garden

# Upload to Front Garden controller
pio run -e front_garden -t upload

# Monitor serial output (115200 baud)
pio device monitor
```

---

## MQTT topics

Valve command payloads follow the [Home Assistant valve](https://www.home-assistant.io/integrations/valve.mqtt/) standard:

| Direction | Payload |
|-----------|---------|
| Open valve | `OPEN` |
| Close valve | `CLOSE` |
| Valve open state | `open` |
| Valve closed state | `closed` |

The watchdog duration (in minutes, range **0–120**) is also exposed as a Home Assistant `number` entity on topic `Home/Irrigation/Watchdog/Command`. Set to `0` to disable the watchdog timeout.

> **Legacy combined status JSON:** the old `StatusJSON` payload (`Valve1`-`Valve4` + `WatchdogMinutes`) on `secret_publishNodeStatusJsonTopic` is no longer published by default, as auto-discovery provides per-valve state topics and a watchdog number entity. Re-enable it with `#define ENABLE_LEGACY_MQTT_STATUS_JSON 1` in `Private.h` if a manual Home Assistant configuration depends on it.

---

## CI/CD and Automation

The project includes GitHub Actions workflows for:

- **Build Pipeline** – Builds firmware for all three environments on every push/PR
- **Release Pipeline** – Builds and attaches firmware binaries to GitHub releases
- **Dependency Check** – Weekly scan for outdated library versions

Firmware artifacts are available for download from the [Actions](https://github.com/genestealer/Controllino-Irrigation/actions) tab.

---

## Scheduled maintenance reboot

The firmware reboots once every 7 days during the 2–4 AM window (UTC).
NTP is synced on boot via `pool.ntp.org` (configurable in `Private.h`).

---

## Matching Home Assistant configuration

https://github.com/Genestealer/Home-Assistant-Configuration

---

## Screenshots

![Home Assistant main view](https://raw.githubusercontent.com/genestealer/Controllino-Irrigation/master/images/Home%20Assistant%20Webpage%20GUI%20Main.JPG)

![Home Assistant settings view](https://raw.githubusercontent.com/genestealer/Controllino-Irrigation/master/images/Home%20Assistant%20Webpage%20GUI%20Setting.JPG)

---

## Bill of Materials

- [Controllino Maxi](https://www.controllino.com/product/controllino-maxi/)
- [12 V Electric Solenoid Valve](https://www.aliexpress.com/item/32951916193.html)
- [Active 12 V PoE Splitter – IEEE 802.3af, 10/100 Mbps](https://www.aliexpress.com/item/32620368747.html)

---

## PlatformIO configuration summary

The `platformio.ini` follows PlatformIO best practices:

- Global `[platformio]` section for project-level settings
- `[common]` section for shared flags and dependencies (DRY principle)
- Version-pinned library dependencies for reproducible builds
- Three named environments: `controllino_maxi`, `front_garden`, `back_garden`
- Compiler warnings enabled (`-Wall -Wextra`)

---

*Based on the earlier ESP8266 variant: <https://github.com/genestealer/Irrigation-Controller>*
