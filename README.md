# Controllino-Irrigation
 Controllino Powered Irrigation Controller

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


  ### Edits made to the PlatformIO Project Configuration File:
    platform = atmelavr
    board = controllino_maxi
    framework = arduino
    lib_deps = PubSubClient, ArduinoJson, SPI, Ethernet, Controllino, I2CSoilMoistureSensor, Wire
    monitor_speed = 115200

