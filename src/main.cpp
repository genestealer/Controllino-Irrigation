/***************************************************
  Irrigation Controller - Arduino Controllino Maxi (Ethernet-based)
  Author: Richard Huish (2017-2024)

  **Description:**
  Dual irrigation system with local control via Home Assistant GUI and MQTT integration.
  - MQTT 'on' payload commands one output on.
  - MQTT 'off' payload commands one output off.
  - Ethernet connectivity via W5100.
  - Based on previous ESP8266-based projects: https://github.com/genestealer/Irrigation-Controller

  **Project Repository:**
  GitHub: https://github.com/genestealer/Controllino-Irrigation

  **System Overview:**
  - GUI: Locally hosted Home Assistant (https://www.home-assistant.io/)
  - MQTT Broker: Mosquitto (https://mosquitto.org/)
  - OTA Updates: Not supported on ATmega without custom bootloader flashing.

  **Hardware Components:**
  - Controller: Controllino Maxi (ATmega 2560-16AU with W5100 Ethernet)
    - Datasheet: https://www.controllino.biz/product/controllino-maxi/
    - Pinout: https://www.controllino.biz/wp-content/uploads/2018/10/CONTROLLINO-MAXI-Pinout.pdf
    - Bootloaders: https://github.com/CONTROLLINO-PLC/CONTROLLINO_Library/tree/master/Bootloaders/MAXI
  - Power: 12V PoE Active Splitter Adapter (IEEE 802.3af compliant)
  - Valves: 12V Electric Solenoid Valves for water control
  - Protection: Vishay 1N4001 Flyback Diodes (50V 1A)

  **Connections:**
  - Outputs:
    - CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_10: 1st Water Valve (2A output)
    - CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_11: 2nd Water Valve (2A output)
    - CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_09: 3rd Water Valve (2A output)
    - CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_08: 4th Water Valve (2A output)
      *Note: Relays can switch higher voltages or provide galvanic isolation.
  - Indicators:
    - On-board LEDs for MQTT, Ethernet, and system status.

  **Bill of Materials (BOM):**
  - Controllino Maxi: https://www.controllino.biz/product/controllino-maxi/
  - 12V Solenoid Valve: https://www.aliexpress.com/item/32951916193.html
  - 12V PoE Splitter: https://www.aliexpress.com/item/32620368747.html

  **PlatformIO Configuration:**
    platform = atmelavr
    board = controllino_maxi
    framework = arduino
    lib_deps = PubSubClient, ArduinoJson, SPI, arduino-libraries/Ethernet, Controllino, NTPClient
    monitor_speed = 115200

****************************************************/

// Note: Libraries are included in "Project Dependencies" file platformio.ini
// Use correct case to avoid build issues on case-sensitive filesystems
#include "Private.h"      // Passwords etc. not for GitHub
#include <PubSubClient.h> // Arduino Client for MQTT https://github.com/knolleary/pubsubclient
#include <ArduinoJson.h>  // Updated ArduinoJson to Version 6. For sending MQTT JSON messages https://bblanchon.github.io/ArduinoJson/
#include <Arduino.h>      // Core Arduino library https://github.com/arduino/Arduino
#include <limits.h>       // For ULONG_MAX constant
#include <Controllino.h>  // Core Arduino Controllino library https://github.com/CONTROLLINO-PLC/CONTROLLINO_Library
#include <SPI.h>          // Arduino Serial Peripheral Interface - for Ethernet connection https://www.arduino.cc/en/reference/SPI
#include <Ethernet.h>     // Arduino Ethernet https://www.arduino.cc/en/reference/Ethernet
#include <avr/wdt.h>      // Watchdog timer library
#include <NTPClient.h>
#include <EthernetUdp.h> // Use EthernetUdp for NTP communication

// Ethernet parameters
byte mac[] = secret_byte;
int noEthernetConnectionCountRebootCount = 0;
const int noEthernetConnectionCountRebootLimit = 5;

// Initialize the Ethernet client library
EthernetClient ethClient;
PubSubClient mqttClient(ethClient);

// NTP settings
EthernetUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntp_server, 0, 86400000); // Update every 24 hours (86400000 ms)

// Constants for reboot logic
unsigned long lastRebootCheckDate = 0; // Epoch day of the last reboot-eligibility check (set after NTP sync)
unsigned long bootEpochDay = 0;        // Epoch day the device booted / last rebooted (0 until first NTP sync)
const int rebootStartHour = 2;         // Start of reboot window (2:00 AM)
const int rebootEndHour = 4;           // End of reboot window (4:00 AM)
const unsigned long rebootIntervalDays = 7; // Reboot after this many days of uptime
// ULONG_MAX forces an NTP update on the very first call to updateNTPTime()
unsigned long lastNtpUpdateDate = ULONG_MAX;

// MQTT Settings
char message_buff[100];
unsigned long lastReconnectAttempt = 0;           // Reconnecting MQTT - non-blocking https://github.com/knolleary/pubsubclient/blob/master/examples/mqtt_reconnect_nonblocking/mqtt_reconnect_nonblocking.ino
const char *mqtt_server = secret_mqtt_server;     // E.G. 192.168.1.xx
const char *clientName = secret_clientName;       // Client to report to MQTT
const char *mqtt_username = secret_mqtt_username; // MQTT Username
const char *mqtt_password = secret_mqtt_password; // MQTT Password

// Optional Home Assistant discovery device name (defaults to MQTT client name)
#ifdef secret_ha_deviceName
const char *haDeviceName = secret_ha_deviceName;
#else
const char *haDeviceName = secret_clientName;
#endif

bool willRetain = true;              // MQTT Last Will and Testament
const char *willMessage = "offline"; // MQTT Last Will and Testament Message
#define json_buffer_size (256)       // Correct buffer overflow, ref https://github.com/knolleary/pubsubclient/commit/98ad16eff8848bffeb812c4d347dfdb5ddef5a31
int noMqttConnectionCount = 0;
const int noMqttConnectionCountLimit = 5; // Maximum MQTT reconnection attempts before Ethernet reset
// MQTT Subscribe
const char *subscribeCommandTopic1 = secret_commandTopic1; // E.G. Home/Irrigation/Command1
const char *subscribeCommandTopic2 = secret_commandTopic2; // E.G. Home/Irrigation/Command2
const char *subscribeCommandTopic3 = secret_commandTopic3; // E.G. Home/Irrigation/Command3
const char *subscribeCommandTopic4 = secret_commandTopic4; // E.G. Home/Irrigation/Command4
// MQTT State publish topics (for HA discovery switch state)
const char *stateTopic1 = secret_stateTopic1; // E.G. Home/Irrigation/State1
const char *stateTopic2 = secret_stateTopic2; // E.G. Home/Irrigation/State2
const char *stateTopic3 = secret_stateTopic3; // E.G. Home/Irrigation/State3
const char *stateTopic4 = secret_stateTopic4; // E.G. Home/Irrigation/State4
// Friendly names shown in Home Assistant for each valve
const char *valveName1 = secret_valveName1;
const char *valveName2 = secret_valveName2;
const char *valveName3 = secret_valveName3;
const char *valveName4 = secret_valveName4;
// MQTT Watchdog duration control
const char *watchdogCommandTopic = "Home/Irrigation/Watchdog/Command"; // Command topic for watchdog duration (in minutes)
const char *watchdogStateTopic = "Home/Irrigation/Watchdog/State";     // State topic for watchdog duration (in minutes)
// MQTT Publish
const char *publishLastWillTopic = secret_publishLastWillTopic;             // MQTT last will
const char *publishNodeStatusJsonTopic = secret_publishNodeStatusJsonTopic; // State of the node
const char *publishNodeHealthJsonTopic = secret_publishNodeHealthJsonTopic; // Health of the node
// MQTT publish frequency
unsigned long previousMillis = 0;
const long publishInterval = 120000; // Publish frequency in milliseconds 120000 = 2 min (status updates every 2 minutes)

// Home Assistant MQTT Discovery
const char *haDiscoveryPrefix = "homeassistant"; // HA discovery root topic
// Firmware version reported to Home Assistant (single source of truth for all
// discovery entities and the origin metadata).
#define FIRMWARE_VERSION "2026.6.15"

// Forward declarations
void mqttPublishStatusData(bool ignorePublishInterval);

// LED output parameters
const int DIGITAL_PIN_LED_POWER_STATUS = CONTROLLINO_D0;
const int DIGITAL_PIN_LED_NETWORK_CONNECTED = CONTROLLINO_D1;
const int DIGITAL_PIN_LED_MQTT_CONNECTED = CONTROLLINO_D2;
const int DIGITAL_PIN_LED_MQTT_FLASH = CONTROLLINO_D3;

// Relay output pins
const int DIGITAL_PIN_OUTPUT_ONE = CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_11;   // Define output one
const int DIGITAL_PIN_OUTPUT_TWO = CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_10;   // Define output two
const int DIGITAL_PIN_OUTPUT_THREE = CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_09; // Define output three
const int DIGITAL_PIN_OUTPUT_FOUR = CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_08;  // Define output four

// Output powered status
bool outputOnePoweredStatus = false;
bool outputTwoPoweredStatus = false;
bool outputThreePoweredStatus = false;
bool outputFourPoweredStatus = false;

// Define state machine states
typedef enum
{
  s_idle1 = 0,        // state idle
  s_Output1Start = 1, // state start
  s_Output1On = 2,    // state on
  s_Output1Stop = 3,  // state stop
} e_state1;
e_state1 stateMachine1 = s_idle1;

typedef enum
{
  s_idle2 = 0,        // state idle
  s_Output2Start = 1, // state start
  s_Output2On = 2,    // state on
  s_Output2Stop = 3,  // state stop
} e_state2;
e_state2 stateMachine2 = s_idle2;

typedef enum
{
  s_idle3 = 0,        // state idle
  s_Output3Start = 1, // state start
  s_Output3On = 2,    // state on
  s_Output3Stop = 3,  // state stop
} e_state3;
e_state3 stateMachine3 = s_idle3;

typedef enum
{
  s_idle4 = 0,        // state idle
  s_Output4Start = 1, // state start
  s_Output4On = 2,    // state on
  s_Output4Stop = 3,  // state stop
} e_state4;
e_state4 stateMachine4 = s_idle4;

typedef enum
{
  outputOne = 0,
  outputTwo = 1,
  outputThree = 2,
  outputFour = 3,
} irrigationOutputs;

// Watchdog duration timer, to set maximum duration in milliseconds keep outputs on. (In case of network/server connection break)
int watchdogDurationMinutes = 60;                        // Default: 60 minutes (controllable via MQTT)
unsigned long watchdogDurationTimeSetMillis = 3600000UL; // Calculated from watchdogDurationMinutes (60 mins = 3600000 millis)
// Per-valve watchdog start time. Each valve has its own independent timer so
// that turning one valve on does not affect the timeout of another. Indexed by
// the irrigationOutputs enum (outputOne..outputFour).
unsigned long watchdogTimeStarted[4] = {0UL, 0UL, 0UL, 0UL};

/*
Reboot the Arduino using the watchdog timer.
*/
void rebootArduino()
{
  wdt_enable(WDTO_15MS); // Enable watchdog timer with a 15ms timeout
  while (true)
  {
    // Wait for the watchdog timer to reset the Arduino
  }
}

/*
Setup the ethernet connection.
Normally called only once from setup.
Also called if the MQTT connection fails after 5 re-tries.
*/
bool setup_ethernet()
{
  if (Ethernet.begin(mac) == 0)
  {
    Serial.println("Failed to configure Ethernet using DHCP");
    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware)
    {
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware.");
      return false;
    }
  }
  else
  {
    Serial.print("  DHCP assigned IP ");
    Serial.println(Ethernet.localIP());
    digitalWrite(DIGITAL_PIN_LED_NETWORK_CONNECTED, HIGH); // Lights on HIGH.
    return true;
  }
  return false; // Catch all
}

void checkEthernetConnection()
{
  Serial.println("Inside checkEthernetConnection() function");
  switch (Ethernet.maintain())
  {
  case 0:
    Serial.println("DHCP: Nothing happened");
    break;
  case 1:
    Serial.println("DHCP: Renew failed");
    break;
  case 2:
    Serial.println("DHCP: Renew success");
    break;
  case 3:
    Serial.println("DHCP: Rebind fail");
    break;
  case 4:
    Serial.println("DHCP: Rebind success");
    break;
  default:
    Serial.println("DHCP: Unexpected number");
    break;
  }
  // The 2 lines below cause a crash for some reason!!!
  // print your local IP address:
  // Serial.println("Current IP address: " + Ethernet.localIP());
}

// Publish this nodes state via MQTT
void publishNodeHealth()
{
  Serial.println("Inside publishNodeHealth() function");
  // Update status to online, retained = true - last will Message will drop in if we go offline
  mqttClient.publish(publishLastWillTopic, "online", true);

  // Gather data
  char bufIP[16];  // IP address
  char bufMAC[18]; // MAC address (formatted as XX:XX:XX:XX:XX:XX)
  sprintf(bufIP, "%d.%d.%d.%d", Ethernet.localIP()[0], Ethernet.localIP()[1], Ethernet.localIP()[2], Ethernet.localIP()[3]);

  // Format MAC address as XX:XX:XX:XX:XX:XX
  sprintf(bufMAC, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  // Prepare and send the data in JSON to MQTT
  // Use a static JSON document with bounded capacity (saves SRAM and avoids dynamic allocation)
  StaticJsonDocument<json_buffer_size> doc1;
  // const char* values are stored by reference; the char buffers above are
  // copied into the document before it is serialized below, so both are safe.
  doc1["ClientName"] = clientName;
  doc1["IP"] = bufIP;
  doc1["MAC"] = bufMAC; // Include the MAC address in the JSON payload

  serializeJsonPretty(doc1, Serial);
  Serial.println(""); // Add new line as serializeJson() leaves the line open.
  char buffer[json_buffer_size];
  // Serialize to a temporary buffer
  serializeJson(doc1, buffer);
  if (!mqttClient.publish(publishNodeHealthJsonTopic, buffer, true)) // Retain data.
  {
    Serial.print("  Failed to publish JSON sensor data to [");
    Serial.print(publishNodeHealthJsonTopic);
    Serial.println("]");
  }
  else
  {
    Serial.print("  JSON Sensor data published to [");
    Serial.print(publishNodeHealthJsonTopic);
    Serial.println("] ");
  }

  Serial.println("  Completed publishNodeHealth() function");
}

// Publish valve state to its dedicated state topic for HA
void publishValveState(uint8_t valveIndex, bool isOn)
{
  const char *topic = nullptr;
  switch (valveIndex)
  {
  case 1:
    topic = stateTopic1;
    break;
  case 2:
    topic = stateTopic2;
    break;
  case 3:
    topic = stateTopic3;
    break;
  case 4:
    topic = stateTopic4;
    break;
  default:
    return;
  }
  // Use proper valve state payloads: "open" or "closed"
  const char *payload = isOn ? "open" : "closed";
  if (!mqttClient.publish(topic, payload, true))
  {
    Serial.println("  Failed to publish valve state to [" + String(topic) + "]");
  }
  else
  {
    Serial.println("  Published valve " + String(valveIndex) + " state: " + String(payload));
  }
}

void publishAllValveStates()
{
  publishValveState(1, outputOnePoweredStatus);
  publishValveState(2, outputTwoPoweredStatus);
  publishValveState(3, outputThreePoweredStatus);
  publishValveState(4, outputFourPoweredStatus);
}

// Publish watchdog duration state to MQTT
void publishWatchdogState()
{
  char buffer[10];
  itoa(watchdogDurationMinutes, buffer, 10);
  if (!mqttClient.publish(watchdogStateTopic, buffer, true))
  {
    Serial.println("  Failed to publish watchdog state");
  }
  else
  {
    Serial.println("  Published watchdog duration: " + String(watchdogDurationMinutes) + " minutes");
  }
}

// Publish JSON as retained MQTT message using streaming to avoid large temporary payload buffers.
bool publishRetainedJson(const char *topic, const JsonDocument &doc)
{
  if (!mqttClient.connected())
  {
    return false;
  }

  const size_t payloadLen = measureJson(doc);
  if (!mqttClient.beginPublish(topic, payloadLen, true))
  {
    return false;
  }

  const size_t written = serializeJson(doc, mqttClient);
  if (written != payloadLen)
  {
    mqttClient.endPublish();
    return false;
  }

  return mqttClient.endPublish();
}

// Update watchdog duration from minutes to milliseconds
void updateWatchdogDuration()
{
  watchdogDurationTimeSetMillis = (unsigned long)watchdogDurationMinutes * 60000UL;
  Serial.println("  Watchdog duration updated to " + String(watchdogDurationMinutes) + " minutes (" + String(watchdogDurationTimeSetMillis) + " ms)");
}

// Add the shared Home Assistant device and origin metadata to a discovery doc.
// Centralises the firmware version and device identity so every entity reports
// the same values.
void addDeviceAndOrigin(JsonDocument &doc)
{
  JsonObject dev = doc.createNestedObject("device");
  JsonArray idArr = dev.createNestedArray("identifiers");
  idArr.add(clientName);
  dev["name"] = haDeviceName;
  dev["model"] = "Controllino Maxi";
  dev["manufacturer"] = "Controllino";
  dev["sw_version"] = FIRMWARE_VERSION;

  JsonObject origin = doc.createNestedObject("o");
  origin["name"] = "Controllino-Irrigation";
  origin["sw"] = FIRMWARE_VERSION;
  origin["url"] = "https://github.com/genestealer/Controllino-Irrigation";
}

// Publish Home Assistant MQTT Discovery configs (retained)
void publishHADiscovery()
{
  Serial.println("Publishing Home Assistant MQTT Discovery configs...");

  // Use clientName as the node_id in discovery topics and unique ids.
  const char *nodeId = clientName;
  // Reused stack buffers avoid repeated String heap allocations.
  char topic[96];
  char uniqueId[64];
  char name[48];

  // Build and publish valve configs for 4 irrigation valves
  for (uint8_t i = 1; i <= 4; i++)
  {
    // Friendly name and topics per valve
    const char *cmdTopic = nullptr;
    const char *stTopic = nullptr;
    const char *vName = nullptr;
    switch (i)
    {
    case 1:
      cmdTopic = subscribeCommandTopic1;
      stTopic = stateTopic1;
      vName = valveName1;
      break;
    case 2:
      cmdTopic = subscribeCommandTopic2;
      stTopic = stateTopic2;
      vName = valveName2;
      break;
    case 3:
      cmdTopic = subscribeCommandTopic3;
      stTopic = stateTopic3;
      vName = valveName3;
      break;
    case 4:
      cmdTopic = subscribeCommandTopic4;
      stTopic = stateTopic4;
      vName = valveName4;
      break;
    default:
      continue; // skip any unexpected index
    }
    {
      // Keep valve buffers scoped to this block to reduce peak stack usage.
      StaticJsonDocument<384> cfg;
      cfg["name"] = vName;
      snprintf(uniqueId, sizeof(uniqueId), "%s_valve%u", nodeId, i);
      cfg["unique_id"] = uniqueId;
      // Device class for water valves
      cfg["device_class"] = "water";
      // Topics
      cfg["cmd_t"] = cmdTopic;
      cfg["stat_t"] = stTopic;
      // Valve command payloads (OPEN/CLOSE are HA defaults)
      cfg["pl_open"] = "OPEN";
      cfg["pl_cls"] = "CLOSE";
      // State payloads (open/closed are HA defaults)
      cfg["stat_open"] = "open";
      cfg["stat_clsd"] = "closed";
      // Valve does not report position
      cfg["reports_position"] = false;
      // Availability
      cfg["avty_t"] = publishLastWillTopic;
      cfg["pl_avail"] = "online";
      cfg["pl_not_avail"] = "offline";
      // Optimistic mode (state updates immediately without waiting for feedback)
      cfg["optimistic"] = false;

      addDeviceAndOrigin(cfg);

      snprintf(topic, sizeof(topic), "%s/valve/%s/valve%u/config", haDiscoveryPrefix, nodeId, i);
      if (!publishRetainedJson(topic, cfg))
      {
        Serial.print("  Failed to publish discovery to [");
        Serial.print(topic);
        Serial.println("]");
      }
      else
      {
        Serial.print("  Published discovery for valve ");
        Serial.println(i);
      }
    }

    // Also expose each valve as a switch entity for automations that
    // specifically require a switch domain entity.
    {
      StaticJsonDocument<384> swCfg;
      snprintf(name, sizeof(name), "%s Switch", vName);
      swCfg["name"] = name;
      snprintf(uniqueId, sizeof(uniqueId), "%s_valve%u_switch", nodeId, i);
      swCfg["unique_id"] = uniqueId;
      swCfg["cmd_t"] = cmdTopic;
      swCfg["stat_t"] = stTopic;
      swCfg["pl_on"] = "OPEN";
      swCfg["pl_off"] = "CLOSE";
      swCfg["stat_on"] = "open";
      swCfg["stat_off"] = "closed";
      swCfg["icon"] = "mdi:water";
      swCfg["avty_t"] = publishLastWillTopic;
      swCfg["pl_avail"] = "online";
      swCfg["pl_not_avail"] = "offline";

      addDeviceAndOrigin(swCfg);

      snprintf(topic, sizeof(topic), "%s/switch/%s/valve%u_switch/config", haDiscoveryPrefix, nodeId, i);
      if (!publishRetainedJson(topic, swCfg))
      {
        Serial.print("  Failed to publish discovery to [");
        Serial.print(topic);
        Serial.println("]");
      }
      else
      {
        Serial.print("  Published discovery for switch ");
        Serial.println(i);
      }
    }
  }

  // Publish watchdog duration number entity for Home Assistant
  {
    StaticJsonDocument<512> cfg;
    cfg["name"] = "Watchdog Duration";
    snprintf(uniqueId, sizeof(uniqueId), "%s_watchdog_duration", nodeId);
    cfg["unique_id"] = uniqueId;
    cfg["cmd_t"] = watchdogCommandTopic;
    cfg["stat_t"] = watchdogStateTopic;
    cfg["unit_of_meas"] = "min";
    cfg["min"] = 0;
    cfg["max"] = 120;
    cfg["step"] = 1;
    cfg["mode"] = "slider";
    cfg["icon"] = "mdi:timer-sand";
    cfg["avty_t"] = publishLastWillTopic;
    cfg["pl_avail"] = "online";
    cfg["pl_not_avail"] = "offline";
    cfg["entity_category"] = "config";

    addDeviceAndOrigin(cfg);

    snprintf(topic, sizeof(topic), "%s/number/%s/watchdog/config", haDiscoveryPrefix, nodeId);
    if (!publishRetainedJson(topic, cfg))
      Serial.println("  Failed to publish watchdog discovery");
    else
      Serial.println("  Published discovery for watchdog duration");
  }

  // Online/connected status from LWT topic
  {
    StaticJsonDocument<512> cfg;
    cfg["name"] = "Online";
    snprintf(uniqueId, sizeof(uniqueId), "%s_online", nodeId);
    cfg["unique_id"] = uniqueId;
    cfg["stat_t"] = publishLastWillTopic;
    cfg["pl_on"] = "online";
    cfg["pl_off"] = "offline";
    cfg["dev_cla"] = "connectivity";
    cfg["entity_category"] = "diagnostic";
    cfg["avty_t"] = publishLastWillTopic;
    cfg["pl_avail"] = "online";
    cfg["pl_not_avail"] = "offline";

    addDeviceAndOrigin(cfg);

    snprintf(topic, sizeof(topic), "%s/binary_sensor/%s/online/config", haDiscoveryPrefix, nodeId);
    if (!publishRetainedJson(topic, cfg))
      Serial.println("  Failed to publish online sensor discovery");
    else
      Serial.println("  Published discovery for online sensor");
  }

  // IP address extracted from node health JSON
  {
    StaticJsonDocument<512> cfg;
    cfg["name"] = "IP";
    snprintf(uniqueId, sizeof(uniqueId), "%s_ip", nodeId);
    cfg["unique_id"] = uniqueId;
    cfg["stat_t"] = publishNodeHealthJsonTopic;
    cfg["val_tpl"] = "{{ value_json.IP }}";
    cfg["icon"] = "mdi:ip-network";
    cfg["entity_category"] = "diagnostic";
    cfg["avty_t"] = publishLastWillTopic;
    cfg["pl_avail"] = "online";
    cfg["pl_not_avail"] = "offline";

    addDeviceAndOrigin(cfg);

    snprintf(topic, sizeof(topic), "%s/sensor/%s/ip/config", haDiscoveryPrefix, nodeId);
    if (!publishRetainedJson(topic, cfg))
      Serial.println("  Failed to publish IP sensor discovery");
    else
      Serial.println("  Published discovery for IP sensor");
  }

  // MAC address extracted from node health JSON
  {
    StaticJsonDocument<512> cfg;
    cfg["name"] = "MAC";
    snprintf(uniqueId, sizeof(uniqueId), "%s_mac", nodeId);
    cfg["unique_id"] = uniqueId;
    cfg["stat_t"] = publishNodeHealthJsonTopic;
    cfg["val_tpl"] = "{{ value_json.MAC }}";
    cfg["icon"] = "mdi:lan-connect";
    cfg["entity_category"] = "diagnostic";
    cfg["avty_t"] = publishLastWillTopic;
    cfg["pl_avail"] = "online";
    cfg["pl_not_avail"] = "offline";

    addDeviceAndOrigin(cfg);

    snprintf(topic, sizeof(topic), "%s/sensor/%s/mac/config", haDiscoveryPrefix, nodeId);
    if (!publishRetainedJson(topic, cfg))
      Serial.println("  Failed to publish MAC sensor discovery");
    else
      Serial.println("  Published discovery for MAC sensor");
  }
}


// Subscribe to MQTT topics
void mqttSubscribe()
{
  mqttClient.subscribe(subscribeCommandTopic1);
  mqttClient.subscribe(subscribeCommandTopic2);
  mqttClient.subscribe(subscribeCommandTopic3);
  mqttClient.subscribe(subscribeCommandTopic4);
  mqttClient.subscribe(watchdogCommandTopic);
}

/*
  Non-Blocking mqtt reconnect.
  Called from checkMqttConnection.
  Based on example from 5ace47b Sep 7, 2015 https://github.com/knolleary/pubsubclient/blob/master/examples/mqtt_reconnect_nonblocking/mqtt_reconnect_nonblocking.ino
*/
boolean mqttReconnect()
{
  Serial.println("Inside mqttReconnect() function");

  Serial.println("  Attempting MQTT connection...");
  // Single, non-blocking connection attempt. Retries are handled by
  // checkMqttConnection(), which calls this every 5 seconds without blocking
  // the main loop (the irrigation watchdog and state machines must keep
  // running while we are disconnected).
  if (!mqttClient.connect(clientName, mqtt_username, mqtt_password, publishLastWillTopic, 0, willRetain, willMessage))
  {
    Serial.println("  Failed MQTT connection.");
    return false; // Return false if connection fails
  }

  // If successful, proceed with publishing and subscribing
  Serial.println("  Call publishNodeHealth() from mqttReconnect()");

  // Publish node state data
  publishNodeHealth();
  Serial.println("  Call mqttSubscribe() from mqttReconnect()");
  mqttSubscribe();
  Serial.println("  Connected to MQTT server");

  // Home Assistant discovery and initial states
  publishHADiscovery();
  publishAllValveStates();
  publishWatchdogState();
  mqttPublishStatusData(true);

  return mqttClient.connected(); // Return connection state
}

/*
  Checks if connection to the MQTT server is ok. Client connected
  using a non-blocking reconnect function. If the client loses
  its connection, it attempts to reconnect every 5 seconds
  without blocking the main loop.
  Called from main loop.
  If MQTT connection fails after x attempts it tries to reconnect ethernet
  If ethernet connections fails after x attempts it reboots the esp
*/
void checkMqttConnection()
{
  if (!mqttClient.connected())
  {
    // We are not connected. Turn off the ethernet LED
    digitalWrite(DIGITAL_PIN_LED_MQTT_CONNECTED, LOW);
    unsigned long now = millis();
    if (now - lastReconnectAttempt > 5000)
    {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (mqttReconnect())
      {
        // We are connected.
        lastReconnectAttempt = 0;
        noMqttConnectionCount = 0;
        noEthernetConnectionCountRebootCount = 0;
        digitalWrite(DIGITAL_PIN_LED_MQTT_CONNECTED, HIGH);
      }
      else
      {
        // Connection to MQTT failed.
        // If no connection after x attempts, then reconnect ethernet, if no connection after x attempts reboot.
        noMqttConnectionCount++; // Increment the counter
        Serial.println("MQTT connection attempt number: " + String(noMqttConnectionCount));
        if (noMqttConnectionCount > noMqttConnectionCountLimit)
        {
          // Max MQTT connection attempts reached, reconnect ethernet.
          noMqttConnectionCount = 0; // Reset MQTT connection attempt counter.
          Serial.println("  MQTT connection count limit reached, reconnecting ethernet");
          // Try to reconnect Ethernet, if this fails after x attemps then reboot.
          if (!setup_ethernet())
          {
            noEthernetConnectionCountRebootCount++; // Increment the counter
            Serial.println("  Ethernet connection attempt number: " + String(noEthernetConnectionCountRebootCount));
            if (noEthernetConnectionCountRebootCount > noEthernetConnectionCountRebootLimit)
            {
              Serial.println("   Ethernet re-connection count limit reached, reboot arduino");
              // Reboot
              rebootArduino();
            }
          }
        }
      }
    }
  }
  else
  {
    // Client connected: MQTT client loop processing
    mqttClient.loop();
  }
}

/*
MQTT Publish with normal or immediate option.
Serialize JSON document into an MQTT message.
*/
void mqttPublishStatusData(bool ignorePublishInterval)
{
  // Only run when publishInterval in milliseconds expires or ignorePublishInterval == true
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= publishInterval || ignorePublishInterval == true)
  {
    previousMillis = currentMillis; // Save the last time this ran
    Serial.println("");
    Serial.println("##############################################");
    Serial.println("Inside mqttPublishStatusData() function");
    digitalWrite(DIGITAL_PIN_LED_MQTT_FLASH, HIGH); // Light LED whilst in this fuction

    // Check connection to MQTT server
    if (mqttClient.connected())
    {

      // Publish node state data
      publishNodeHealth();

      // Prepare and send the data in JSON to MQTT
      // Use a static JSON document with bounded capacity (saves SRAM and avoids dynamic allocation)
      StaticJsonDocument<json_buffer_size> doc;
      // Publish valve states as "open" or "closed" to match HA valve entity standards
      doc["Valve1"] = outputOnePoweredStatus ? "open" : "closed";
      doc["Valve2"] = outputTwoPoweredStatus ? "open" : "closed";
      doc["Valve3"] = outputThreePoweredStatus ? "open" : "closed";
      doc["Valve4"] = outputFourPoweredStatus ? "open" : "closed";
      doc["WatchdogMinutes"] = watchdogDurationMinutes;
      serializeJsonPretty(doc, Serial);
      Serial.println(""); // Add new line as serializeJson() leaves the line open.
      char buffer[json_buffer_size];
      // Serialize to a temporary buffer
      serializeJson(doc, buffer);
      if (!mqttClient.publish(publishNodeStatusJsonTopic, buffer, true)) // Retain data.
        Serial.println("  Failed to publish JSON sensor data to [" + String(publishNodeStatusJsonTopic) + "]");
      else
        Serial.println("  JSON Sensor data published to [" + String(publishNodeStatusJsonTopic) + "] ");
      Serial.println("  Complete mqttPublishStatusData() function");
    }
    // Always turn the flash LED off, even when we skipped publishing because
    // the MQTT client was not connected (otherwise the LED stays stuck on).
    digitalWrite(DIGITAL_PIN_LED_MQTT_FLASH, LOW); // Turn off LED
    Serial.println("##############################################");
    Serial.println("");
    Serial.println("");
  }
}

// MQTT payload
void mqttcallback(char *topic, byte *payload, unsigned int length)
{
  Serial.println("");
  Serial.println("**********************************************");

  Serial.println("Inside mqttcallback() function. Data received.");
  digitalWrite(DIGITAL_PIN_LED_MQTT_CONNECTED, HIGH);

  Serial.print("  Message arrived [");
  Serial.print(topic);
  Serial.println("]");

  // Safely copy the payload into message_buff and null-terminate it
  unsigned int copyLen = length < sizeof(message_buff) - 1 ? length : sizeof(message_buff) - 1;
  memcpy(message_buff, payload, copyLen);
  message_buff[copyLen] = '\0'; // Ensure null-termination

  // Warn if payload was truncated
  if (length >= sizeof(message_buff))
  {
    Serial.println("  WARNING: Payload truncated from " + String(length) + " to " + String(copyLen) + " bytes");
  }

  String msgString = String(message_buff); // Convert to string once
  String normalizedMsg = msgString;
  normalizedMsg.trim();
  normalizedMsg.toUpperCase();
  Serial.println("  Payload: " + msgString);

  // Check the message topic and update state accordingly
  String srtTopic = topic;

  // Process valve commands (OPEN/CLOSE format per HA valve standard)
  if (srtTopic.equals(subscribeCommandTopic1))
  {
    if (normalizedMsg == "OPEN" || normalizedMsg == "ON" || normalizedMsg == "1")
    {
      stateMachine1 = s_Output1Start; // Set output one to be on
    }
    else if (normalizedMsg == "CLOSE" || normalizedMsg == "CLOSED" || normalizedMsg == "OFF" || normalizedMsg == "0")
    {
      stateMachine1 = s_Output1Stop; // Set output one to be off
    }
  }
  else if (srtTopic.equals(subscribeCommandTopic2))
  {
    if (normalizedMsg == "OPEN" || normalizedMsg == "ON" || normalizedMsg == "1")
    {
      stateMachine2 = s_Output2Start; // Set output two to be on
    }
    else if (normalizedMsg == "CLOSE" || normalizedMsg == "CLOSED" || normalizedMsg == "OFF" || normalizedMsg == "0")
    {
      stateMachine2 = s_Output2Stop; // Set output two to be off
    }
  }
  else if (srtTopic.equals(subscribeCommandTopic3))
  {
    if (normalizedMsg == "OPEN" || normalizedMsg == "ON" || normalizedMsg == "1")
    {
      stateMachine3 = s_Output3Start; // Set output three to be on
    }
    else if (normalizedMsg == "CLOSE" || normalizedMsg == "CLOSED" || normalizedMsg == "OFF" || normalizedMsg == "0")
    {
      stateMachine3 = s_Output3Stop; // Set output three to be off
    }
  }
  else if (srtTopic.equals(subscribeCommandTopic4))
  {
    if (normalizedMsg == "OPEN" || normalizedMsg == "ON" || normalizedMsg == "1")
    {
      stateMachine4 = s_Output4Start; // Set output four to be on
    }
    else if (normalizedMsg == "CLOSE" || normalizedMsg == "CLOSED" || normalizedMsg == "OFF" || normalizedMsg == "0")
    {
      stateMachine4 = s_Output4Stop; // Set output four to be off
    }
  }
  // Handle watchdog duration command
  else if (srtTopic.equals(watchdogCommandTopic))
  {
    int newDuration = msgString.toInt();
    if (newDuration >= 0 && newDuration <= 120)
    {
      watchdogDurationMinutes = newDuration;
      updateWatchdogDuration();
      publishWatchdogState(); // Confirm the change
    }
    else
    {
      Serial.println("  Invalid watchdog duration: " + msgString + " (must be 0-120 minutes)");
    }
  }

  digitalWrite(DIGITAL_PIN_LED_MQTT_CONNECTED, LOW);

  Serial.println("  Completed mqttcallback() function");
  Serial.println("");
}

void controlOutputOne(bool state)
{
  if (state == true)
  {
    // Start this valve's own watchdog timer (independent of other valves).
    watchdogTimeStarted[outputOne] = millis();
    // Command the output on.
    Serial.println("controlOutputOne state true");
    digitalWrite(DIGITAL_PIN_OUTPUT_ONE, HIGH);
    outputOnePoweredStatus = true;
  }
  else
  {
    // Command the output off.
    Serial.println("controlOutputOne state false");
    digitalWrite(DIGITAL_PIN_OUTPUT_ONE, LOW);
    outputOnePoweredStatus = false;
  }
  publishValveState(1, outputOnePoweredStatus);
}

void controlOutputTwo(bool state)
{
  if (state == true)
  {
    // Start this valve's own watchdog timer (independent of other valves).
    watchdogTimeStarted[outputTwo] = millis();
    // Command the output on.
    Serial.println("controlOutputTwo state true");
    digitalWrite(DIGITAL_PIN_OUTPUT_TWO, HIGH);
    outputTwoPoweredStatus = true;
  }
  else
  {
    // Command the output off.
    Serial.println("controlOutputTwo state false");
    digitalWrite(DIGITAL_PIN_OUTPUT_TWO, LOW);
    outputTwoPoweredStatus = false;
  }
  publishValveState(2, outputTwoPoweredStatus);
}

void controlOutputThree(bool state)
{
  if (state == true)
  {
    // Start this valve's own watchdog timer (independent of other valves).
    watchdogTimeStarted[outputThree] = millis();
    // Command the output on.
    Serial.println("controlOutputThree state true");
    digitalWrite(DIGITAL_PIN_OUTPUT_THREE, HIGH);
    outputThreePoweredStatus = true;
  }
  else
  {
    // Command the output off.
    Serial.println("controlOutputThree state false");
    digitalWrite(DIGITAL_PIN_OUTPUT_THREE, LOW);
    outputThreePoweredStatus = false;
  }
  publishValveState(3, outputThreePoweredStatus);
}

void controlOutputFour(bool state)
{
  if (state == true)
  {
    // Start this valve's own watchdog timer (independent of other valves).
    watchdogTimeStarted[outputFour] = millis();
    // Command the output on.
    Serial.println("controlOutputFour state true");
    digitalWrite(DIGITAL_PIN_OUTPUT_FOUR, HIGH);
    outputFourPoweredStatus = true;
  }
  else
  {
    // Command the output off.
    Serial.println("controlOutputFour state false");
    digitalWrite(DIGITAL_PIN_OUTPUT_FOUR, LOW);
    outputFourPoweredStatus = false;
  }
  publishValveState(4, outputFourPoweredStatus);
}

// Per-valve watchdog check. Each valve is timed independently from when it was
// turned on, so one valve timing out never affects another.
// valveIndex is an irrigationOutputs enum value (outputOne..outputFour).
bool checkWatchdog(irrigationOutputs valveIndex)
{
  // A duration of 0 means the watchdog is disabled - never shut off by timeout
  if (watchdogDurationMinutes == 0)
  {
    return false;
  }

  // Check this valve's own elapsed on-time against the configured duration.
  if (millis() - watchdogTimeStarted[valveIndex] >= watchdogDurationTimeSetMillis)
  {
    Serial.println("checkWatchdog: duration exceeded for valve " + String((int)valveIndex + 1) + ", shutting it down");
    return true;
  }

  return false;
}

// State machines for controller
// Output 1 State Machine
void checkState1()
{
  switch (stateMachine1)
  {

  case s_idle1:
    // State is currently: idle
    break;

  case s_Output1Start:
    // State is currently: starting
    Serial.println("State is currently: starting output one");
    // Command the output on.
    controlOutputOne(true);
    mqttPublishStatusData(true); // Immediate publish cycle
    // Watchdog timer is managed by checkWatchdog() function
    stateMachine1 = s_Output1On;
    break;

  case s_Output1On:
    // State is currently: On
    // Check if we need to stop, by checking for watchdog duration timer.
    if (checkWatchdog(outputOne))
      stateMachine1 = s_Output1Stop;
    break;

  case s_Output1Stop:
    // State is currently: stopping
    Serial.println("State is currently: stopping output one");
    // Command the output off.
    controlOutputOne(false);
    mqttPublishStatusData(true); // Immediate publish cycle
    // Set state mahcine to idle on the next loop
    stateMachine1 = s_idle1;
    break;
  }
}

// Output 2 State Machine
void checkState2()
{
  switch (stateMachine2)
  {

  case s_idle2:
    // State is currently: idle
    break;

  case s_Output2Start:
    // State is currently: starting
    Serial.println("State is currently: starting output two");
    // Command the output on.
    controlOutputTwo(true);
    mqttPublishStatusData(true); // Immediate publish cycle
    // Watchdog timer is managed by checkWatchdog() function
    stateMachine2 = s_Output2On;
    break;

  case s_Output2On:
    // State is currently: On
    // Check if we need to stop, by checking for watchdog duration timer.
    if (checkWatchdog(outputTwo))
      stateMachine2 = s_Output2Stop;
    break;

  case s_Output2Stop:
    // State is currently: stopping
    Serial.println("State is currently: stopping output two");
    // Command the output off.
    controlOutputTwo(false);
    mqttPublishStatusData(true); // Immediate publish cycle
    // Set state mahcine to idle on the next loop
    stateMachine2 = s_idle2;
    break;
  }
}

// Output 3 State Machine
void checkState3()
{
  switch (stateMachine3)
  {

  case s_idle3:
    // State is currently: idle
    break;

  case s_Output3Start:
    // State is currently: starting
    Serial.println("State is currently: starting output three");
    // Command the output on.
    controlOutputThree(true);
    mqttPublishStatusData(true); // Immediate publish cycle
    // Watchdog timer is managed by checkWatchdog() function
    stateMachine3 = s_Output3On;
    break;

  case s_Output3On:
    // State is currently: On
    // Check if we need to stop, by checking for watchdog duration timer.
    if (checkWatchdog(outputThree))
      stateMachine3 = s_Output3Stop;
    break;

  case s_Output3Stop:
    // State is currently: stopping
    Serial.println("State is currently: stopping output three");
    // Command the output off.
    controlOutputThree(false);
    mqttPublishStatusData(true); // Immediate publish cycle
    // Set state mahcine to idle on the next loop
    stateMachine3 = s_idle3;
    break;
  }
}

// Output 4 State Machine
void checkState4()
{
  switch (stateMachine4)
  {

  case s_idle4:
    // State is currently: idle
    break;

  case s_Output4Start:
    // State is currently: starting
    Serial.println("State is currently: starting output four");
    // Command the output on.
    controlOutputFour(true);
    mqttPublishStatusData(true); // Immediate publish cycle
    // Watchdog timer is managed by checkWatchdog() function
    stateMachine4 = s_Output4On;
    break;

  case s_Output4On:
    // State is currently: On
    // Check if we need to stop, by checking for watchdog duration timer.
    if (checkWatchdog(outputFour))
      stateMachine4 = s_Output4Stop;
    break;

  case s_Output4Stop:
    // State is currently: stopping
    Serial.println("State is currently: stopping output four");
    // Command the output off.
    controlOutputFour(false);
    mqttPublishStatusData(true); // Immediate publish cycle
    // Set state mahcine to idle on the next loop
    stateMachine4 = s_idle4;
    break;
  }
}

/*
  Initialize the NTP client and perform an immediate time sync.
  Also records the boot day so the reboot interval is measured from startup.
*/
void setupNTP()
{
  timeClient.begin();
  // Force an immediate NTP sync so getEpochTime() returns a real value
  // from the very first loop() iteration instead of 0.
  if (timeClient.forceUpdate())
  {
    Serial.print("NTP initial sync. Current time: ");
    Serial.println(timeClient.getFormattedTime());
    // Record today so updateNTPTime() does not re-sync until tomorrow
    // getEpochTime() returns seconds since Unix epoch; divide by 86400 (seconds/day) to get epoch day
    lastNtpUpdateDate = timeClient.getEpochTime() / 86400UL;
    // Record the boot day and pre-seed the reboot check to today so the reboot
    // interval is counted from startup and we never reboot immediately.
    bootEpochDay = lastNtpUpdateDate;
    lastRebootCheckDate = lastNtpUpdateDate;
  }
  else
  {
    // bootEpochDay stays 0; checkRebootCondition() will record it lazily once
    // NTP synchronises in the main loop.
    Serial.println("NTP initial sync failed; will retry in loop.");
  }
}

/*
  Update the NTP time once a day.
  Uses date comparison instead of hour-based flag to avoid missing midnight.
*/
void updateNTPTime()
{
  // Get current date as epoch day (days since 1970-01-01)
  unsigned long currentEpochDay = timeClient.getEpochTime() / 86400UL;

  // Update NTP if we haven't updated today. Only record the date when the
  // update actually succeeds; otherwise lastNtpUpdateDate would be set to an
  // unsynced (near-zero) day and NTP would never retry until the day rolled
  // over, which cannot happen while the clock is unsynced.
  if (currentEpochDay != lastNtpUpdateDate)
  {
    if (timeClient.update())
    {
      lastNtpUpdateDate = timeClient.getEpochTime() / 86400UL;
      Serial.print("NTP updated. Current time: ");
      Serial.println(timeClient.getFormattedTime());
    }
  }
}

/*
  Check if the system should reboot after a fixed number of days of uptime,
  during the night maintenance window (2-4 AM). Uses NTP epoch-day tracking so
  it is immune to millis() overflow, and measures days since boot rather than
  calendar weeks so the interval since the last reboot is consistent.
*/
void checkRebootCondition()
{
  const unsigned long epoch = timeClient.getEpochTime();

  // Ignore an unsynced clock (epoch near zero) so we never reboot on bad time.
  if (epoch < 1000000000UL)
  {
    return;
  }

  unsigned long currentEpochDay = epoch / 86400UL;

  // Lazily record the boot day the first time we have a valid clock. This also
  // covers the case where the initial NTP sync in setup failed.
  if (bootEpochDay == 0)
  {
    bootEpochDay = currentEpochDay;
    lastRebootCheckDate = currentEpochDay;
    return;
  }

  // Only evaluate once per day to avoid repeated reboot attempts.
  if (currentEpochDay != lastRebootCheckDate)
  {
    lastRebootCheckDate = currentEpochDay;

    // Reboot once the configured number of days of uptime has elapsed.
    if ((currentEpochDay - bootEpochDay) >= rebootIntervalDays)
    {
      int currentHour = timeClient.getHours();

      // Check if the current time is within the reboot window (2-4 AM)
      if (currentHour >= rebootStartHour && currentHour < rebootEndHour)
      {
        Serial.println("Scheduled reboot: maintenance window reached");
        rebootArduino(); // Call the reboot function
      }
    }
  }
}

// Custom setup for this program.
void customSetup()
{
  // Initialize pins
  pinMode(DIGITAL_PIN_OUTPUT_ONE, OUTPUT);
  pinMode(DIGITAL_PIN_OUTPUT_TWO, OUTPUT);
  pinMode(DIGITAL_PIN_OUTPUT_THREE, OUTPUT);
  pinMode(DIGITAL_PIN_OUTPUT_FOUR, OUTPUT);

  // Initialize pin start values
  digitalWrite(DIGITAL_PIN_OUTPUT_ONE, LOW);
  digitalWrite(DIGITAL_PIN_OUTPUT_TWO, LOW);
  digitalWrite(DIGITAL_PIN_OUTPUT_THREE, LOW);
  digitalWrite(DIGITAL_PIN_OUTPUT_FOUR, LOW);

  // Initialize NTP
  setupNTP();
}

// Custom loop for this program.
void customLoop()
{
  // Check the status and do actions
  checkState1();
  checkState2();
  checkState3();
  checkState4();

  // Update NTP time
  updateNTPTime();

  // Check reboot condition
  checkRebootCondition();
}

void setup()
{
  // Ensure watchdog is disabled on startup to prevent boot loops
  wdt_disable();
  // Set serial speed
  Serial.begin(115200);
  Serial.println("Setup Starting...");
  delay(250); // Give the IC chance to startup.

  // Initialize pins
  Serial.println("Setup pins..");
  pinMode(DIGITAL_PIN_LED_POWER_STATUS, OUTPUT);
  pinMode(DIGITAL_PIN_LED_MQTT_CONNECTED, OUTPUT);
  pinMode(DIGITAL_PIN_LED_NETWORK_CONNECTED, OUTPUT);
  pinMode(DIGITAL_PIN_LED_MQTT_FLASH, OUTPUT);
  pinMode(CONTROLLINO_D6, OUTPUT);
  pinMode(CONTROLLINO_D7, OUTPUT);
  pinMode(CONTROLLINO_D8, OUTPUT);
  pinMode(CONTROLLINO_D9, OUTPUT);
  pinMode(CONTROLLINO_D10, OUTPUT);

  // Initialize pin start values
  digitalWrite(DIGITAL_PIN_LED_POWER_STATUS, HIGH);
  digitalWrite(DIGITAL_PIN_LED_MQTT_CONNECTED, LOW);
  digitalWrite(DIGITAL_PIN_LED_NETWORK_CONNECTED, LOW);
  digitalWrite(DIGITAL_PIN_LED_MQTT_FLASH, LOW);

  // Set startup debug LED #1
  digitalWrite(CONTROLLINO_D6, HIGH);
  delay(250);
  Serial.println("  Pins setup complete");

  // Setup ethernet
  Serial.println("Setup ethernet..");
  setup_ethernet();
  Serial.println("  Ethernet setup complete");

  // Set startup debug LED #2
  digitalWrite(CONTROLLINO_D7, HIGH);
  delay(250);

  // Setup for this project (output pins, NTP) before MQTT connects so
  // relays are in a known safe state when the first MQTT messages arrive.
  Serial.println("Start custom project setup..");
  customSetup();

  // Set MQTT settings
  Serial.println("Setup MQTT..");
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setBufferSize(1024);
  mqttClient.setCallback(mqttcallback);
  checkMqttConnection();
  Serial.println("  Setup MQTT complete");

  // Set startup debug LED #3
  digitalWrite(CONTROLLINO_D8, HIGH);
  delay(250);

  Serial.println("  Setup Complete");
  Serial.println("");
  Serial.println("");
}

// Main working loop
void loop()
{
  // Check ethernet connection periodically for DHCP maintenance
  static unsigned long lastEthernetCheck = 0;
  if (millis() - lastEthernetCheck >= 60000)
  { // Check every 60 seconds
    checkEthernetConnection();
    lastEthernetCheck = millis();
  }

  // Check connection to the MQTT server
  checkMqttConnection();

  // Publish MQTT
  mqttPublishStatusData(false); // Normal publish cycle

  // Loop for this project.
  customLoop();
}
