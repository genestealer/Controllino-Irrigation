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
#include "Private.h"               // Passwords etc. not for GitHub
#include <PubSubClient.h>          // Arduino Client for MQTT https://github.com/knolleary/pubsubclient
#include <ArduinoJson.h>           // Updated ArduinoJson to Version 6. For sending MQTT JSON messages https://bblanchon.github.io/ArduinoJson/
#include <Arduino.h>               // Core Arduino library https://github.com/arduino/Arduino
#include <Controllino.h>           // Core Arduino Controllino library https://github.com/CONTROLLINO-PLC/CONTROLLINO_Library
#include <SPI.h>                   // Arduino Serial Peripheral Interface - for Ethernet connection https://www.arduino.cc/en/reference/SPI
#include <Ethernet.h>              // Arduino Ethernet https://www.arduino.cc/en/reference/Ethernet
#include <avr/wdt.h>               // Watchdog timer library
#include <NTPClient.h>
#include <EthernetUdp.h>           // Use EthernetUdp for NTP communication

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
const unsigned long sevenDaysMillis = 7UL * 24 * 60 * 60 * 1000; // 7 days in milliseconds
unsigned long systemStartTime = 0; // Store the system start time
const int rebootStartHour = 2; // Start of reboot window (2:00 AM)
const int rebootEndHour = 4;   // End of reboot window (4:00 AM)
bool ntpUpdatedToday = false;  // Flag to ensure NTP is updated only once a day

// MQTT Settings
char message_buff[100];
long lastReconnectAttempt = 0;                    // Reconnecting MQTT - non-blocking https://github.com/knolleary/pubsubclient/blob/master/examples/mqtt_reconnect_nonblocking/mqtt_reconnect_nonblocking.ino
const char *mqtt_server = secret_mqtt_server;     // E.G. 192.168.1.xx
const char *clientName = secret_clientName;       // Client to report to MQTT
const char *mqtt_username = secret_mqtt_username; // MQTT Username
const char *mqtt_password = secret_mqtt_password; // MQTT Password
bool willRetain = true;                           // MQTT Last Will and Testament
const char *willMessage = "offline";              // MQTT Last Will and Testament Message
#define json_buffer_size (256)                    // Correct buffer overflow, ref https://github.com/knolleary/pubsubclient/commit/98ad16eff8848bffeb812c4d347dfdb5ddef5a31
// const int json_buffer_size = 256;
int noMqttConnectionCount = 0;
const int noMqttConnectionCountLimit = 5;
// MQTT Subscribe
const char *subscribeCommandTopic1 = secret_commandTopic1; // E.G. Home/Irrigation/Command1
const char *subscribeCommandTopic2 = secret_commandTopic2; // E.G. Home/Irrigation/Command2
const char *subscribeCommandTopic3 = secret_commandTopic3; // E.G. Home/Irrigation/Command3
const char *subscribeCommandTopic4 = secret_commandTopic4; // E.G. Home/Irrigation/Command4
// MQTT State publish topics (for HA discovery switch state)
const char *stateTopic1 = secret_stateTopic1;   // E.G. Home/Irrigation/State1
const char *stateTopic2 = secret_stateTopic2;   // E.G. Home/Irrigation/State2
const char *stateTopic3 = secret_stateTopic3;   // E.G. Home/Irrigation/State3
const char *stateTopic4 = secret_stateTopic4;   // E.G. Home/Irrigation/State4
// MQTT Watchdog duration control
const char *watchdogCommandTopic = "Home/Irrigation/Watchdog/Command";      // Command topic for watchdog duration (in minutes)
const char *watchdogStateTopic = "Home/Irrigation/Watchdog/State";          // State topic for watchdog duration (in minutes)
// MQTT Publish
const char *publishLastWillTopic = secret_publishLastWillTopic;             // MQTT last will
const char *publishNodeStatusJsonTopic = secret_publishNodeStatusJsonTopic; // State of the node
const char *publishNodeHealthJsonTopic = secret_publishNodeHealthJsonTopic; // Health of the node
// MQTT publish frequency
unsigned long previousMillis = 0;
const long publishInterval = 120000; // Publish frequency in milliseconds 120000 = 2 min

// Home Assistant MQTT Discovery
const char *haDiscoveryPrefix = "homeassistant"; // HA discovery root topic

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
int stateMachine1 = 0;

typedef enum
{
  s_idle2 = 0,        // state idle
  s_Output2Start = 1, // state start
  s_Output2On = 2,    // state on
  s_Output2Stop = 3,  // state stop
} e_state2;
int stateMachine2 = 0;

typedef enum
{
  s_idle3 = 0,        // state idle
  s_Output3Start = 1, // state start
  s_Output3On = 2,    // state on
  s_Output3Stop = 3,  // state stop
} e_state3;
int stateMachine3 = 0;

typedef enum
{
  s_idle4 = 0,        // state idle
  s_Output4Start = 1, // state start
  s_Output4On = 2,    // state on
  s_Output4Stop = 3,  // state stop
} e_state4;
int stateMachine4 = 0;

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
unsigned long watchdogTimeStarted = 0UL;

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
  // Serial.println("Inside setup_ethernet() function");
  // Serial.println("Initialize Ethernet with DHCP:");
  if (Ethernet.begin(mac) == 0)
  {
    Serial.println("Failed to configure Ethernet using DHCP");
    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware)
    {
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware.");
      // while (true) {
      //   delay(1); // do nothing, no point running without Ethernet hardware
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
  // INFO: the data must be converted into a string; a problem occurs when using floats...
  doc1["ClientName"] = String(clientName);
  doc1["IP"] = String(bufIP);
  doc1["MAC"] = String(bufMAC); // Include the MAC address in the JSON payload

  serializeJsonPretty(doc1, Serial);
  Serial.println(""); // Add new line as serializeJson() leaves the line open.
  char buffer[json_buffer_size];
  // Serialize to a temporary buffer
  serializeJson(doc1, buffer);
  if (!mqttClient.publish(publishNodeHealthJsonTopic, buffer, true)) // Retain data.
    Serial.println("  Failed to publish JSON sensor data to [" + String(publishNodeHealthJsonTopic) + "]");
  else
    Serial.println("  JSON Sensor data published to [" + String(publishNodeHealthJsonTopic) + "] ");

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

// Update watchdog duration from minutes to milliseconds
void updateWatchdogDuration()
{
  watchdogDurationTimeSetMillis = (unsigned long)watchdogDurationMinutes * 60000UL;
  Serial.println("  Watchdog duration updated to " + String(watchdogDurationMinutes) + " minutes (" + String(watchdogDurationTimeSetMillis) + " ms)");
}

// Publish Home Assistant MQTT Discovery configs (retained)
void publishHADiscovery()
{
  Serial.println("Publishing Home Assistant MQTT Discovery configs...");

  // Device info shared among entities
  const char *nodeId = clientName; // use clientName as node_id

  // Build and publish valve configs for 4 irrigation valves
  for (uint8_t i = 1; i <= 4; i++)
  {
    // Discovery topic: <prefix>/valve/<node_id>/valve<i>/config
    String topic = String(haDiscoveryPrefix) + "/valve/" + nodeId + "/valve" + String(i) + "/config";

    StaticJsonDocument<512> cfg;
    // Friendly name - set to null to inherit device name
    cfg["name"] = String("Valve ") + String(i);
    // Unique id
    cfg["unique_id"] = String(nodeId) + String("_valve") + String(i);
    // Device class for water valves
    cfg["device_class"] = "water";
    // Topics
    switch (i)
    {
    case 1:
      cfg["cmd_t"] = subscribeCommandTopic1;
      cfg["stat_t"] = stateTopic1;
      break;
    case 2:
      cfg["cmd_t"] = subscribeCommandTopic2;
      cfg["stat_t"] = stateTopic2;
      break;
    case 3:
      cfg["cmd_t"] = subscribeCommandTopic3;
      cfg["stat_t"] = stateTopic3;
      break;
    case 4:
      cfg["cmd_t"] = subscribeCommandTopic4;
      cfg["stat_t"] = stateTopic4;
      break;
    }
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

    // Device metadata
    JsonObject dev = cfg.createNestedObject("device");
    JsonArray idArr = dev.createNestedArray("identifiers");
    idArr.add(nodeId);
    dev["name"] = nodeId;
    dev["model"] = "Controllino Maxi";
    dev["manufacturer"] = "Controllino";
    dev["sw_version"] = "2024.1.0";

    // Origin info (who created this discovery message)
    JsonObject origin = cfg.createNestedObject("o");
    origin["name"] = "Controllino-Irrigation";
    origin["sw"] = "2024.1.0";
    origin["url"] = "https://github.com/genestealer/Controllino-Irrigation";

    char payload[512];
    size_t n = serializeJson(cfg, payload, sizeof(payload));
    if (n == 0 || n >= sizeof(payload))
    {
      Serial.println("  Discovery payload too large for valve " + String(i));
    }
    else
    {
      if (!mqttClient.publish(topic.c_str(), payload, true))
      {
        Serial.println("  Failed to publish discovery to [" + topic + "]");
      }
      else
      {
        Serial.println("  Published discovery for valve " + String(i));
      }
    }
  }

  // Publish watchdog duration number entity for Home Assistant
  {
    String topic = String(haDiscoveryPrefix) + "/number/" + nodeId + "/watchdog/config";
    StaticJsonDocument<512> cfg;
    cfg["name"] = "Watchdog Duration";
    cfg["unique_id"] = String(nodeId) + "_watchdog_duration";
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

    // Device metadata
    JsonObject dev = cfg.createNestedObject("device");
    JsonArray idArr = dev.createNestedArray("identifiers");
    idArr.add(nodeId);
    dev["name"] = nodeId;
    dev["model"] = "Controllino Maxi";
    dev["manufacturer"] = "Controllino";
    dev["sw_version"] = "2024.1.0";

    // Origin info
    JsonObject origin = cfg.createNestedObject("o");
    origin["name"] = "Controllino-Irrigation";
    origin["sw"] = "2024.1.0";
    origin["url"] = "https://github.com/genestealer/Controllino-Irrigation";

    char payload[512];
    size_t n = serializeJson(cfg, payload, sizeof(payload));
    if (n == 0 || n >= sizeof(payload))
    {
      Serial.println("  Discovery payload too large for watchdog");
    }
    else
    {
      if (!mqttClient.publish(topic.c_str(), payload, true))
      {
        Serial.println("  Failed to publish watchdog discovery");
      }
      else
      {
        Serial.println("  Published discovery for watchdog duration");
      }
    }
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

  int retryCount = 0;
  // Retry up to 5 times if the MQTT connection fails
  while (!mqttClient.connect(clientName, mqtt_username, mqtt_password, publishLastWillTopic, 0, willRetain, willMessage) && retryCount < 5)
  {
    Serial.println("  Failed MQTT connection, retrying...");
    delay(1000);
    retryCount++;
  }

  if (retryCount == 5)
  {
    Serial.println("  Failed to connect to MQTT after 5 attempts. Aborting.");
    return false; // Return false if connection fails after retries
  }

  // If successful, proceed with publishing and subscribing
  Serial.println("  Attempting MQTT connection...");
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
    // Check ethernet connection
    checkEthernetConnection();

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
      digitalWrite(DIGITAL_PIN_LED_MQTT_FLASH, LOW); // Turn off LED
    }
    Serial.println("##############################################");
    Serial.println("");
    Serial.println("");
  }
}

// MQTT payload
// Add a flag to prevent simultaneous state changes
bool stateChanging = false; // Flag to indicate state transition in progress

void mqttcallback(char *topic, byte *payload, unsigned int length)
{
  // If state is already changing, ignore new state transitions
  if (stateChanging)
  {
    return; // Prevent multiple state transitions simultaneously
  }

  stateChanging = true; // Set flag to indicate state change

  Serial.println("");
  Serial.println("**********************************************");

  Serial.println("Inside mqttcallback() function. Data received.");
  digitalWrite(CONTROLLINO_D2, HIGH);

  Serial.print("  Message arrived [");
  Serial.print(topic);
  Serial.println("]");

  // Safely copy the payload into message_buff and null-terminate it
  unsigned int copyLen = length < sizeof(message_buff) - 1 ? length : sizeof(message_buff) - 1;
  memcpy(message_buff, payload, copyLen);
  message_buff[copyLen] = '\0'; // Ensure null-termination

  String msgString = String(message_buff); // Convert to string once
  Serial.println("  Payload: " + msgString);

  // Check the message topic and update state accordingly
  String srtTopic = topic;

  // Process valve commands (OPEN/CLOSE format per HA valve standard)
  if (srtTopic.equals(subscribeCommandTopic1))
  {
    if (msgString == "OPEN" || msgString == "1")
    {
      stateMachine1 = s_Output1Start; // Set output one to be on
    }
    else if (msgString == "CLOSE" || msgString == "0")
    {
      stateMachine1 = s_Output1Stop; // Set output one to be off
    }
  }
  else if (srtTopic.equals(subscribeCommandTopic2))
  {
    if (msgString == "OPEN" || msgString == "1")
    {
      stateMachine2 = s_Output2Start; // Set output two to be on
    }
    else if (msgString == "CLOSE" || msgString == "0")
    {
      stateMachine2 = s_Output2Stop; // Set output two to be off
    }
  }
  else if (srtTopic.equals(subscribeCommandTopic3))
  {
    if (msgString == "OPEN" || msgString == "1")
    {
      stateMachine3 = s_Output3Start; // Set output three to be on
    }
    else if (msgString == "CLOSE" || msgString == "0")
    {
      stateMachine3 = s_Output3Stop; // Set output three to be off
    }
  }
  else if (srtTopic.equals(subscribeCommandTopic4))
  {
    if (msgString == "OPEN" || msgString == "1")
    {
      stateMachine4 = s_Output4Start; // Set output four to be on
    }
    else if (msgString == "CLOSE" || msgString == "0")
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

  stateChanging = false; // Reset flag after state transition
  digitalWrite(CONTROLLINO_D2, LOW);

  Serial.println("  Completed mqttcallback() function");
  Serial.println("");
}

void controlOutputOne(bool state)
{
  if (state == true)
  {
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

bool checkWatchdog()
{
  // Helper lambda to know if any output is currently on
  auto anyOutputOn = []() {
    return outputOnePoweredStatus || outputTwoPoweredStatus || outputThreePoweredStatus || outputFourPoweredStatus;
  };

  // If no outputs are on, there's nothing to watch; keep timer as-is
  if (!anyOutputOn()) {
    return false;
  }

  // When outputs are on, check duration
  if (millis() - watchdogTimeStarted >= watchdogDurationTimeSetMillis) {
    Serial.println("checkWatchdog: duration exceeded");
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
    // Start watchdog duration timer.
    watchdogTimeStarted = millis();
    stateMachine1 = s_Output1On;
    break;

  case s_Output1On:
    // State is currently: On
    // Check if we need to stop, by checking for watchdog duration timer.
    if (checkWatchdog())
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
    // Start watchdog duration timer.
    watchdogTimeStarted = millis();
    stateMachine2 = s_Output2On;
    break;

  case s_Output2On:
    // State is currently: On
    // Check if we need to stop, by checking for watchdog duration timer.
    if (checkWatchdog())
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
    // Start watchdog duration timer.
    watchdogTimeStarted = millis();
    stateMachine3 = s_Output3On;
    break;

  case s_Output3On:
    // State is currently: On
    // Check if we need to stop, by checking for watchdog duration timer.
    if (checkWatchdog())
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
    // Start watchdog duration timer.
    watchdogTimeStarted = millis();
    stateMachine4 = s_Output4On;
    break;

  case s_Output4On:
    // State is currently: On
    // Check if we need to stop, by checking for watchdog duration timer.
    if (checkWatchdog())
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
  Initialize the NTP client.
*/
void setupNTP() {
  timeClient.begin();
  Serial.println("NTP client initialized.");
}

/*
  Update the NTP time once a day.
*/
void updateNTPTime() {
  if (!ntpUpdatedToday) {
    timeClient.update();
    Serial.print("Current NTP time: ");
    Serial.println(timeClient.getFormattedTime());
    ntpUpdatedToday = true;
  }

  // Reset the flag at midnight
  if (timeClient.getHours() == 0) {
    ntpUpdatedToday = false;
  }
}

/*
  Check if the system has been online for too long (7 days) and reboot during the night.
*/
void checkRebootCondition() {
  unsigned long currentMillis = millis();
  unsigned long uptimeMillis = currentMillis - systemStartTime;

  // Check if uptime exceeds 7 days
  if (uptimeMillis >= sevenDaysMillis) {
    timeClient.update(); // Ensure we have the latest time
    int currentHour = timeClient.getHours();

    // Check if the current time is within the reboot window
    if (currentHour >= rebootStartHour && currentHour < rebootEndHour) {
      Serial.println("Rebooting system after 7 days of uptime...");
      rebootArduino(); // Call the reboot function
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

  // Set MQTT settings
  Serial.println("Setup MQTT..");
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(mqttcallback);
  checkMqttConnection();
  Serial.println("  Setup MQTT complete");

  // Set startup debug LED #3
  digitalWrite(CONTROLLINO_D8, HIGH);
  delay(250);

  // Setup for this project.
  Serial.println("Start custom project setup..");
  customSetup();

  // Record system start time
  systemStartTime = millis();

  Serial.println("  Setup Complete");
  Serial.println("");
  Serial.println("");
}

// Main working loop
void loop()
{
  // Check connection to the MQTT server
  checkMqttConnection();

  // Publish MQTT
  mqttPublishStatusData(false); // Normal publish cycle

  // Loop for this project.
  customLoop();
}
