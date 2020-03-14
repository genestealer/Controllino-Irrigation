/***************************************************
  Irrigation and Lighting Controller
  Richard Huish 2017-2020
  Controllino Maxi Ethernet based with local home-assistant.io GUI,
    relay output for dual irrigation control via MQTT
    MQTT command message with 'on' payload command one of the outputs on.
    MQTT command message with 'off' payload command one of the outputs off.
  Note: My code is based on my other ESP8266 based projects, so there are may be some ESP nameing used.
  ----------
  Github: https://github.com/genestealer/Controllino-Irrigation
  ----------
  Key Libraries:
   ArduinoJson.h           https://bblanchon.github.io/ArduinoJson/ BUT ONLY UP TO VERSION 5!
   Updated arduinojson to Version 6
  ----------
  GUI: Locally hosted home assistant
  MQTT: Locally hosted broker https://mosquitto.org/
  OTA updates - not supported by the ATmega
  ----------
  The circuit:
    Controllino Maxi - ATmega 2560-16AU with W5100 ethetnet
    https://www.controllino.biz/wp-content/uploads/2018/10/CONTROLLINO-MAXI-Pinout.pdf
  Inputs:
    Analog Capacitive Soil Moisture Sensor V1.2 https://www.aliexpress.com/item/32832538686
    W5100 ethetnet (Built-In)
  Outputs:
    Relay one output - GPIO pin 28 (CONTROLLINO Relay 6)
    Relay two output - GPIO pin 29 (CONTROLLINO Relay 7)
    Multiple on-board LEDS
    ----------
  Notes:
    Multiple on-board LEDS toshow MQTT connection, ethernet connection, status, etc
    ----------
   Edits made to the PlatformIO Project Configuration File:
     platform = atmelavr = https://github.com/esp8266/Arduino/issues/2833 as the standard has an outdated Arduino Core for the ESP8266, ref http://docs.platformio.org/en/latest/platforms/espressif8266.html#over-the-air-ota-update
     build_flags = -DMQTT_MAX_PACKET_SIZE=512 = Overide max JSON size, until libary is updated to inclde this option https://github.com/knolleary/pubsubclient/issues/110#issuecomment-174953049
   ----------
   Sources:
   https://github.com/mertenats/open-home-automation/tree/master/ha_mqtt_sensor_dht22
   Create a JSON object
     Example https://github.com/mertenats/Open-Home-Automation/blob/master/ha_mqtt_sensor_dht22/ha_mqtt_sensor_dht22.ino
     Doc : https://github.com/bblanchon/ArduinoJson/wiki/API%20Reference
****************************************************/

// Note: Libaries are inluced in "Project Dependencies" file platformio.ini
#include <private.h>               // Passwords etc not for github
#include <PubSubClient.h>          // Arduino Client for MQTT https://github.com/knolleary/pubsubclient
#include <ArduinoJson.h>           // For sending MQTT JSON messages https://bblanchon.github.io/ArduinoJson/
#include <Arduino.h>
#include <Controllino.h>
#include <SPI.h>
#include <Ethernet.h>

// Ethernet parameters
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
const int noWifiConnectionCountLimit = 5;
int noEthernetConnectionCountRebootCount = 0;
const int noEthernetConnectionCountRebootLimit = 5;

// Initialize the Ethernet client library
EthernetClient ethClient;
PubSubClient mqttClient(ethClient);

// MQTT Settings
char message_buff[100];
long lastReconnectAttempt = 0; // Reconnecting MQTT - non-blocking https://github.com/knolleary/pubsubclient/blob/master/examples/mqtt_reconnect_nonblocking/mqtt_reconnect_nonblocking.ino
const char* mqtt_server = secret_mqtt_server; // E.G. 192.168.1.xx
const char* clientName = secret_clientName; // Client to report to MQTT
const char* mqtt_username = secret_mqtt_username; // MQTT Username
const char* mqtt_password = secret_mqtt_password; // MQTT Password
bool willRetain = true; // MQTT Last Will and Testament
const char* willMessage = "offline"; // MQTT Last Will and Testament Message
const int json_buffer_size = 256;
int noMqttConnectionCount = 0;
const int noMqttConnectionCountLimit = 5;
// MQTT Subscribe
const char* subscribeCommandTopic1 = secret_commandTopic1; // E.G. Home/Irrigation/Command1
const char* subscribeCommandTopic2 = secret_commandTopic2; // E.G. Home/Irrigation/Command2
// MQTT Publish
const char* publishLastWillTopic = secret_publishLastWillTopic;              // MQTT last will
const char* publishNodeStatusJsonTopic = secret_publishNodeStatusJsonTopic;  // State of the node
const char* publishNodeHealthJsonTopic = secret_publishNodeHealthJsonTopic;  // Health of the node
// MQTT publish frequency
unsigned long previousMillis = 0;
const long publishInterval = 6000; // Publish frequency in milliseconds 60000 = 1 min

// LED output parameters
const int DIGITAL_PIN_LED_POWER_STATUS = CONTROLLINO_D0;
const int DIGITAL_PIN_LED_NETWORK_CONNECTED = CONTROLLINO_D1;
const int DIGITAL_PIN_LED_MQTT_CONNECTED = CONTROLLINO_D2;
const int DIGITAL_PIN_LED_MQTT_FLASH = CONTROLLINO_D3;

// Relay output pins
const int DIGITAL_PIN_RELAY_ONE = CONTROLLINO_R1; // Define relay output one
const int DIGITAL_PIN_RELAY_TWO = CONTROLLINO_R2; // Define relay output two

// Output powered status
bool outputOnePoweredStatus = false;
bool outputTwoPoweredStatus = false;

// Sensor Inputs
const int ANALOGUE_PIN_ONE = CONTROLLINO_A0; // Define analogue input one

// Define state machine states
typedef enum {
  s_idle1 = 0,          // state idle
  s_Output1Start = 1,  // state start
  s_Output1On = 2,     // state on
  s_Output1Stop = 3,   // state stop
} e_state1;
int stateMachine1 = 0;

typedef enum {
  s_idle2 = 0,          // state idle
  s_Output2Start = 1,  // state start
  s_Output2On = 2,     // state on
  s_Output2Stop = 3,   // state stop
} e_state2;
int stateMachine2 = 0;

typedef enum {
  outputOne = 0,
  outputTwo = 1,
} irrigationOutputs;

// Watchdog duration timer, to set maximum duration in milliseconds keep outputs on. (In case of network/server connection break)
float watchdogDurationTimeSetMillis = 3600000; //60 mins = 3600000 millis
float watchdogTimeStarted;


/*
Setup the ethernet connection. 
Normally called only once from setup.
Also called if the MQTT connection failes after 5 re-tries.
*/
bool setup_ethernet() {
  // Serial.println("Inside setup_ethernet() function");
  // Serial.println("Initialize Ethernet with DHCP:");
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware.");
      // while (true) {
      //   delay(1); // do nothing, no point running without Ethernet hardware
      return false;
    }
  } else {
    Serial.print("  DHCP assigned IP ");
    Serial.println(Ethernet.localIP());
    digitalWrite(DIGITAL_PIN_LED_NETWORK_CONNECTED, HIGH); // Lights on HIGH.
    return true;
  }
  return false; // Catch all
}

void checkEthernetConnection() {
  Serial.println("Inside checkEthernetConnection() function");
  switch(Ethernet.maintain()) {
   case 0:   Serial.println("DHCP: Nothing happened"); break;
   case 1:   Serial.println("DHCP: Renew failed"); break;
   case 2:   Serial.println("DHCP: Renew success"); break;
   case 3:   Serial.println("DHCP: Rebind fail"); break;
   case 4:   Serial.println("DHCP: Rebind success"); break;
   default:  Serial.println("DHCP: Unexpected number"); break;
  }
  // The 2 lines below calse a crash for some reason.
  // print your local IP address:
  // Serial.println("Current IP address: " + Ethernet.localIP());
}


// Publish this nodes state via MQTT
void publishNodeHealth() {
  Serial.println("Inside publishNodeHealth() function");
  // Update status to online, retained = true - last will Message will drop in if we go offline
  mqttClient.publish(publishLastWillTopic, "online", true);

  // Gather data
  char bufIP[16]; // IP address
  // char bufMAC[6]; // MAC address
  sprintf(bufIP, "%d.%d.%d.%d", Ethernet.localIP()[0], Ethernet.localIP()[1], Ethernet.localIP()[2], Ethernet.localIP()[3]);
  // sprintf(bufMAC, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);  //This line calses a delayed crash, %x expects an unsigned int, whereas this supplying a char

  // Prepare and send the data in JSON to MQTT
  StaticJsonDocument<json_buffer_size> doc1;
  // INFO: the data must be converted into a string; a problem occurs when using floats...
  doc1["ClientName"] = String(clientName);
  doc1["IP"] = String(bufIP);
  doc1["MAC"] = "Find it from the router"; //String(bufMAC); 
  serializeJsonPretty(doc1, Serial);
  Serial.println(""); // Add new line as serializeJson() leaves the line open.
  char buffer[json_buffer_size];
  // Serialize to a temporary buffer
  serializeJson(doc1, buffer);
  if (!mqttClient.publish(publishNodeHealthJsonTopic, buffer, true))  //Retain data.
    Serial.println("Failed to publish JSON sensor data to [" + String(publishNodeHealthJsonTopic) + "]");
  else
    Serial.println("JSON Sensor data published to [" + String(publishNodeHealthJsonTopic) + "] ");

  Serial.println("Completed publishNodeHealth() function");
}

// Subscribe to MQTT topics
void mqttSubscribe() {
  mqttClient.subscribe(subscribeCommandTopic1);
  mqttClient.subscribe(subscribeCommandTopic2);
}

/*
  Non-Blocking mqtt reconnect.
  Called from checkMqttConnection.
  Based on example from 5ace47b Sep 7, 2015 https://github.com/knolleary/pubsubclient/blob/master/examples/mqtt_reconnect_nonblocking/mqtt_reconnect_nonblocking.ino
*/
boolean mqttReconnect() {
  Serial.println("Inside mqttReconnect() function");
  // Attempt to connect
  if (mqttClient.connect(clientName, mqtt_username, mqtt_password, publishLastWillTopic, 0, willRetain, willMessage)) {
    Serial.println("Attempting MQTT connection...");
    Serial.println("Call publishNodeHealth() from mqttReconnect()");
    // Publish node state data
    publishNodeHealth();
    Serial.println("Call mqttSubscribe() from mqttReconnect()");
    // Resubscribe to feeds
    mqttSubscribe();
    Serial.println("Connected to MQTT server");
  } else {
    Serial.println("Failed MQTT connection, rc=" + String(publishNodeStatusJsonTopic) + "] ");
  }
  Serial.println("Completed mqttReconnect() function");
  return mqttClient.connected(); // Return connection state
}

/*
  Checks if connection to the MQTT server is ok. Client connected
  using a non-blocking reconnect function. If the client loses
  its connection, it attempts to reconnect every 5 seconds
  without blocking the main loop.
  Called from main loop.
  If MQTT connection fails after x attempts it tries to reconnect wifi
  If wifi connections fails after x attempts it reboots the esp
*/
void checkMqttConnection() {
  if (!mqttClient.connected()) {
    // We are not connected. Turn off the wifi LED
    digitalWrite(DIGITAL_PIN_LED_MQTT_CONNECTED, LOW);
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (mqttReconnect()) {
        // We are connected.
        lastReconnectAttempt = 0;
        noMqttConnectionCount = 0;
        noEthernetConnectionCountRebootCount = 0;
        digitalWrite(DIGITAL_PIN_LED_MQTT_CONNECTED, HIGH);
      } else  {
        // Connection to MQTT failed.
        // If no connection after x attempts, then reconnect wifi, if no connection after x attempts reboot.
        noMqttConnectionCount = ++noMqttConnectionCount; //Increment the counter
        Serial.println("MQTT connection attempt number: " + String(noMqttConnectionCount));
        if (noMqttConnectionCount > noMqttConnectionCountLimit) {
          // Max MQTT connection attempts reached, reconnect ethernet.
          noMqttConnectionCount = 0; // Reset MQTT connection attempt counter.
          Serial.println("MQTT connection count limit reached, reconnecting ethernet");
          // Try to reconnect wifi, if this fails after x attemps then reboot.
          if (!setup_ethernet()) {
            noEthernetConnectionCountRebootCount = ++noEthernetConnectionCountRebootCount;
            Serial.println("Wifi connection attempt number: " + String(noEthernetConnectionCountRebootCount));
            if (noEthernetConnectionCountRebootCount > noEthernetConnectionCountRebootLimit) {
              Serial.println("Wifi re-connection count limit reached, reboot arduino");
              // Reboot
              // ESP.restart();
              // resetFunc(); //call reset 
            }
          }
        }
      }


    }
  } else {
    // Client connected: MQTT client loop processing
    mqttClient.loop();
  }
}

// Read soil sensor and return its value.
int readSoilSensor() {
    // Read Soil Sensor Capacitance
    int sensorValue = analogRead(ANALOGUE_PIN_ONE);
    Serial.println("Soil Moisture Capacitance: " + String(sensorValue));
    return sensorValue;
}

/*
MQTT Publish with normal or immediate option.
Serialize JSON document into an MQTT message.
*/
void mqttPublishStatusData(bool ignorePublishInterval) {
  // Only run when publishInterval in milliseconds expires or ignorePublishInterval == true
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= publishInterval || ignorePublishInterval == true) {
    previousMillis = currentMillis; // Save the last time this ran
    Serial.println("##############################################");
    Serial.println("Inside mqttPublishStatusData() function");
    digitalWrite(DIGITAL_PIN_LED_MQTT_FLASH, HIGH); // Light LED whilst in this fuction
    // Check ethernet connection
    checkEthernetConnection();
        
    // Check connection to MQTT server
    if (mqttClient.connected()) {

      // Publish node state data
      publishNodeHealth();

      // Prepare and send the data in JSON to MQTT
      StaticJsonDocument<json_buffer_size> doc;
      // INFO: the data must be converted into a string; a problem occurs when using floats...
      doc["Valve1"] = String(outputOnePoweredStatus);
      doc["Valve2"] = String(outputTwoPoweredStatus);
      doc["SoilCapacitance"] = String(readSoilSensor());
      // doc["SoilTemperature"] = String(soilSensorTemperature);
      serializeJsonPretty(doc, Serial);
      Serial.println(""); // Add new line as serializeJson() leaves the line open.
      char buffer[json_buffer_size];
      // Serialize to a temporary buffer
      serializeJson(doc, buffer);
      if (!mqttClient.publish(publishNodeStatusJsonTopic, buffer, true))  //Retain data.
        Serial.println("Failed to publish JSON sensor data to [" + String(publishNodeStatusJsonTopic) + "]");
      else
        Serial.println("JSON Sensor data published to [" + String(publishNodeStatusJsonTopic) + "] ");
    Serial.println("Complete mqttPublishStatusData() function");
    digitalWrite(DIGITAL_PIN_LED_MQTT_FLASH, LOW); // Turn off LED
    }
  }
}

// MQTT payload
void mqttcallback(char* topic, byte* payload, unsigned int length) {
  //If you want to publish a message from within the message callback function, it is necessary to make a copy of the topic and payload values as the client uses the same internal buffer for inbound and outbound messages:
  //http://www.hivemq.com/blog/mqtt-client-library-encyclopedia-arduino-pubsubclient/
  Serial.println("Inside mqttcallback() function");
  digitalWrite(CONTROLLINO_D2, HIGH); 

  Serial.println("Message arrived [" + String(topic) + "] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  // Create character buffer with ending null terminator (string)
  int i = 0;
  for (i = 0; i < length; i++) {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';
  // Check the value of the message
  String msgString = String(message_buff);
  Serial.println(msgString);
  // Check the message topic
  String srtTopic = topic;

  if (srtTopic.equals(subscribeCommandTopic1)) {
    if (msgString == "1")
      stateMachine1 = s_Output1Start; // Set output one to be on
    else if (msgString == "0")
      stateMachine1 = s_Output1Stop; // Set output one to be off
    }
  else if (srtTopic.equals(subscribeCommandTopic2)) {
    if (msgString == "1")
      stateMachine2 = s_Output2Start; // Set output one to be on
     else if (msgString == "0")
      stateMachine2 = s_Output2Stop; // Set output one to be off
    }
  digitalWrite(CONTROLLINO_D2, LOW); 
  Serial.println("Completeed mqttcallback() function");
}

void controlOutputOne(bool state) {
  if (state == true) {
    // Command the output on.
    Serial.println("controlOutputOne state true");
    digitalWrite(DIGITAL_PIN_RELAY_ONE, HIGH);
    outputOnePoweredStatus = true;
  } else {
    // Command the output off.
    Serial.println("controlOutputOne state false");
    digitalWrite(DIGITAL_PIN_RELAY_ONE, LOW);
    outputOnePoweredStatus = false;
  }
}

void controlOutputTwo(bool state) {
  if (state == true) {
    // Command the output on.
    Serial.println("controlOutputTwo state true");
    digitalWrite(DIGITAL_PIN_RELAY_TWO, HIGH);
    outputTwoPoweredStatus = true;
  } else {
    // Command the output off.
    Serial.println("controlOutputTwo state false");
    digitalWrite(DIGITAL_PIN_RELAY_TWO, LOW);
    outputTwoPoweredStatus = false;
  }
}

bool checkWatchdog() {
  if (millis() - watchdogTimeStarted >= watchdogDurationTimeSetMillis) {
    // Stop, as we must have lost connection to the server and output has been on too long.
    Serial.println("checkWatchdog: duration exceeded");
    return true;
  }
  // Else
  return false;
}


// State machines for controller
// Output 1 State Machine
void checkState1() {
  switch (stateMachine1) {

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
void checkState2() {
  switch (stateMachine2) {

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

// Custom setup for this program.
void customSetup() {
  // Initialize pins
  pinMode(DIGITAL_PIN_RELAY_ONE, OUTPUT);
  pinMode(DIGITAL_PIN_RELAY_TWO, OUTPUT);
  pinMode(ANALOGUE_PIN_ONE, INPUT);

  // Initialize pin start values
  digitalWrite(DIGITAL_PIN_RELAY_ONE, LOW);
  digitalWrite(DIGITAL_PIN_RELAY_TWO, LOW);
}

// Custom loop for this program.
void customLoop() {
  // Check the status and do actions
  checkState1();
  checkState2();
}


void setup() {
  // Set serial speed
  Serial.begin(115200);
  Serial.println("Setup Starting");
  delay(250); // Give the IC chance to startup.

  // Initialize pins
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
  
  // Setup ethernet
  setup_ethernet();

// Set startup debug LED #2
  digitalWrite(CONTROLLINO_D7, HIGH); 
  delay(250);

  // Set MQTT settings
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(mqttcallback);

  checkMqttConnection();

  // Set startup debug LED #3
  digitalWrite(CONTROLLINO_D8, HIGH); 
  delay(250);

  // Setup for this project.
  customSetup();

  Serial.println("Setup Complete");
}

// Main working loop
void loop() {
  // Check connection to the MQTT server
  checkMqttConnection();

  // Publish MQTT
  mqttPublishStatusData(false); // Normal publish cycle

  // Loop for this project.
  customLoop();
}
