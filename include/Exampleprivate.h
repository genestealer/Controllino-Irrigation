// Exampleprivate.h - Template for Private.h (do not include real secrets here)
// Copy this file to include/Private.h and fill in your values.
// Switch controllers using IRRIGATION_CONTROLLER in platformio.ini:
//   build_flags = -DIRRIGATION_CONTROLLER=1  ; Front
//   build_flags = -DIRRIGATION_CONTROLLER=2  ; Back

#pragma once

#ifndef IRRIGATION_CONTROLLER
#define IRRIGATION_CONTROLLER 2
#endif

// Common settings
#define secret_mqtt_server "192.168.0.2" // <-- your MQTT broker IP/host
#define ntp_server "pool.ntp.org"        // NTP server for time synchronization

// Legacy MQTT (optional): set to 1 to also publish the old combined status
// JSON (Valve1-4 + WatchdogMinutes) to secret_publishNodeStatusJsonTopic for
// manually-configured Home Assistant setups. Not needed with MQTT
// auto-discovery. Defaults to disabled (0) when left undefined.
// #define ENABLE_LEGACY_MQTT_STATUS_JSON 1

#if (IRRIGATION_CONTROLLER != 1) && (IRRIGATION_CONTROLLER != 2)
#error "IRRIGATION_CONTROLLER must be 1 (Front) or 2 (Back)"
#endif

// Front Garden (1)
#if IRRIGATION_CONTROLLER == 1

#define secret_clientName "IrrigationControllinoMaxi1"
#define secret_ha_deviceName "IrrigationControllinoMaxi1"
#define secret_mqtt_username "IrrigationController1"
#define secret_mqtt_password "REPLACE_ME"

// MAC address (must be unique on your network)
#define secret_byte {0xDE, 0xA1, 0xAD, 0xEF, 0xFE, 0xED}

#define secret_commandTopic1 "HOME/Irrigation1/OutputOne/Command"
#define secret_stateTopic1 "HOME/Irrigation1/OutputOne/State"
#define secret_valveName1 "Front Valve 1"

#define secret_commandTopic2 "HOME/Irrigation1/OutputTwo/Command"
#define secret_stateTopic2 "HOME/Irrigation1/OutputTwo/State"
#define secret_valveName2 "Front Valve 2"

#define secret_commandTopic3 "HOME/Irrigation1/OutputThree/Command"
#define secret_stateTopic3 "HOME/Irrigation1/OutputThree/State"
#define secret_valveName3 "Front Valve 3"

#define secret_commandTopic4 "HOME/Irrigation1/OutputFour/Command"
#define secret_stateTopic4 "HOME/Irrigation1/OutputFour/State"
#define secret_valveName4 "Front Valve 4"

#define secret_publishLastWillTopic "HOME/Irrigation1/Status"

#define secret_publishNodeStatusJsonTopic "HOME/Irrigation1/StatusJSON"
#define secret_publishNodeHealthJsonTopic "HOME/Irrigation1/NodeHealthJSON"

// Back Garden (2)
#elif IRRIGATION_CONTROLLER == 2

#define secret_clientName "IrrigationControllinoMaxi2"
#define secret_ha_deviceName "IrrigationControllinoMaxi2"
#define secret_mqtt_username "IrrigationController2"
#define secret_mqtt_password "REPLACE_ME"

// MAC address (must be unique on your network)
#define secret_byte {0xDE, 0xA1, 0xAD, 0xEF, 0xED, 0xFE}

#define secret_commandTopic1 "HOME/Irrigation2/OutputOne/Command"
#define secret_stateTopic1 "HOME/Irrigation2/OutputOne/State"
#define secret_valveName1 "Back Valve 1"

#define secret_commandTopic2 "HOME/Irrigation2/OutputTwo/Command"
#define secret_stateTopic2 "HOME/Irrigation2/OutputTwo/State"
#define secret_valveName2 "Back Valve 2"

#define secret_commandTopic3 "HOME/Irrigation2/OutputThree/Command"
#define secret_stateTopic3 "HOME/Irrigation2/OutputThree/State"
#define secret_valveName3 "Back Valve 3"

#define secret_commandTopic4 "HOME/Irrigation2/OutputFour/Command"
#define secret_stateTopic4 "HOME/Irrigation2/OutputFour/State"
#define secret_valveName4 "Back Valve 4"

#define secret_publishLastWillTopic "HOME/Irrigation2/Status"

#define secret_publishNodeStatusJsonTopic "HOME/Irrigation2/StatusJSON"
#define secret_publishNodeHealthJsonTopic "HOME/Irrigation2/NodeHealthJSON"

#endif // IRRIGATION_CONTROLLER
