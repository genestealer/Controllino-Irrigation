# Controllino-Irrigation

[![PlatformIO CI](https://github.com/genestealer/Controllino-Irrigation/actions/workflows/build.yml/badge.svg)](https://github.com/genestealer/Controllino-Irrigation/actions/workflows/build.yml)
[![Release](https://github.com/genestealer/Controllino-Irrigation/actions/workflows/release.yml/badge.svg)](https://github.com/genestealer/Controllino-Irrigation/actions/workflows/release.yml)
[![License](https://img.shields.io/github/license/genestealer/Controllino-Irrigation)](LICENSE)
[![GitHub release (latest by date)](https://img.shields.io/github/v/release/genestealer/Controllino-Irrigation)](https://github.com/genestealer/Controllino-Irrigation/releases)
[![PlatformIO](https://img.shields.io/badge/PlatformIO-Ready-blue)](https://platformio.org/)

Controllino Powered Irrigation Controller

## CI/CD and Automation

The project includes GitHub Actions workflows for:

- **Build Pipeline** - Automatically builds firmware for all environments on push/PR
- **Release Pipeline** - Builds and attaches firmware binaries to GitHub releases
- **Dependency Check** - Weekly checks for library updates

Firmware artifacts are automatically built and available for download from the [Actions](https://github.com/genestealer/Controllino-Irrigation/actions) tab.

## Matching Home Assistant Home Automation Hub Configuration

https://github.com/Genestealer/Home-Assistant-Configuration

## Info

Richard Huish 2017-2020
  
Controllino Maxi Ethernet based with local home-assistant.io GUI, relay/digital output for dual irrigation control via MQTT.    

MQTT command message with 'on' payload command one of the outputs on.

MQTT command message with 'off' payload command one of the outputs off.
    
Note: My code is based on my other ESP8266 based projects, so there are may be some ESP nameing used.

### Based on ESP8266 variant: https://github.com/genestealer/Irrigation-Controller
 
## Screen shots of Home Assistant interface
![Diagram](https://raw.githubusercontent.com/genestealer/Controllino-Irrigation/master/images/Home%20Assistant%20Webpage%20GUI%20Main.JPG)

![Diagram](https://raw.githubusercontent.com/genestealer/Controllino-Irrigation/master/images/Home%20Assistant%20Webpage%20GUI%20Setting.JPG) 
 
## Copy of code header (may be out of date, see code for latest)
  GUI: Locally hosted home assistant on network https://www.home-assistant.io/
  
  MQTT: Locally hosted broker on network https://mosquitto.org/
  
  OTA updates: Not supported by default by the ATmega without flashing with custom bootloader and I have not done this.
 
 ----------
  
  ### The circuit:
   
    Controllino Maxi - ATmega 2560-16AU with W5100 ethernet 
  https://www.controllino.com/product/controllino-maxi/
   
    CONTROLLINO customized bootloaders https://github.com/CONTROLLINO-PLC/CONTROLLINO_Library/tree/master/Bootloaders/MAXI
    
    W5100 ethernet (Built-In)
    
    12V PoE Active splitter Adapter, to power Controllino
    
    12V Electric Solenoid Valve for Water
   
    Flyback diode: Vishay 50V 1A, Diode, 2-Pin DO-204AL 1N4001-E3/54
 
 ### Inputs:
   
    Analog Capacitive Soil Moisture Sensor V1.2 https://www.aliexpress.com/item/32832538686
 
### Outputs:
   
    CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_10 (2 Amp output) - 1st water valve
    
    CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_11 (2 Amp output) - 2nd water valve
    
    (Note: Use could use the relays to switch higher voltages or have galvanic isolation)
    
    Multiple on-board LEDS tos how MQTT connection, ethernet connection, status, etc    Notes:
  
  
  ### Example Bill Of Materials:
   
   - [Controllino Maxi](https://www.controllino.com/product/controllino-maxi/)
   - [1/2 Inch 12V Electric Solenoid Valve for Water](https://www.aliexpress.com/item/32951916193.html)
   - [Active 12V PoE power over ethernet Splitter Adapter, IEEE 802.3af Compliant 10/100Mbps, 12V output]( https://www.aliexpress.com/item/32620368747.html)


### PlatformIO Configuration

The project uses PlatformIO with multiple build environments for different controller configurations:

- **controllino_maxi** - Default environment (Back Garden)
- **front_garden** - Front Garden controller (IRRIGATION_CONTROLLER=1)
- **back_garden** - Back Garden controller (IRRIGATION_CONTROLLER=2)

#### Building for specific environments

```bash
# Build for default environment (Back Garden)
pio run

# Build for Front Garden
pio run -e front_garden

# Build for Back Garden
pio run -e back_garden

# Upload to Front Garden controller
pio run -e front_garden -t upload

# Monitor serial output
pio device monitor
```

The `platformio.ini` follows current PlatformIO best practices with:
- Global `[platformio]` section for project-level settings
- `[common]` section for shared configuration (DRY principle)
- Version-pinned library dependencies for reproducible builds
- Multiple named environments for different deployment targets
- Proper library identifiers (author/library@version)

