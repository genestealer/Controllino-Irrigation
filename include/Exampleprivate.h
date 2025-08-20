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
#define secret_mqtt_server "192.168.0.2"     // <-- your MQTT broker IP/host
#define ntp_server         "pool.ntp.org"    // NTP server for time synchronization

#if (IRRIGATION_CONTROLLER != 1) && (IRRIGATION_CONTROLLER != 2)
#error "IRRIGATION_CONTROLLER must be 1 (Front) or 2 (Back)"
#endif

// Front Garden (1)
#if IRRIGATION_CONTROLLER == 1

#define secret_clientName    "IrrigationControllinoMaxi1"
#define secret_mqtt_username "IrrigationController1"
#define secret_mqtt_password "REPLACE_ME"

// MAC address (must be unique on your network)
#define secret_byte { 0xDE, 0xA1, 0xAD, 0xEF, 0xFE, 0xED }

#define secret_commandTopic1 "HOME/Irrigation1/OutputOne/Command"
#define secret_stateTopic1   "HOME/Irrigation1/OutputOne/State"

#define secret_commandTopic2 "HOME/Irrigation1/OutputTwo/Command"
#define secret_stateTopic2   "HOME/Irrigation1/OutputTwo/State"

#define secret_commandTopic3 "HOME/Irrigation1/OutputThree/Command"
#define secret_stateTopic3   "HOME/Irrigation1/OutputThree/State"

#define secret_commandTopic4 "HOME/Irrigation1/OutputFour/Command"
#define secret_stateTopic4   "HOME/Irrigation1/OutputFour/State"

#define secret_publishLastWillTopic       "HOME/Irrigation1/Status"
#define secret_publishClientName          "HOME/Irrigation1/ClientName"
#define secret_publishIpAddress           "HOME/Irrigation1/IpAddress"
#define secret_publishSignalStrength      "HOME/Irrigation1/SignalStrength"
#define secret_publishHostName            "HOME/Irrigation1/HostName"
#define secret_publishSSID                "HOME/Irrigation1/SSID"

#define secret_publishNodeStatusJsonTopic "HOME/Irrigation1/StatusJSON"
#define secret_publishNodeHealthJsonTopic "HOME/Irrigation1/NodeHealthJSON"

// Back Garden (2)
#elif IRRIGATION_CONTROLLER == 2

#define secret_clientName    "IrrigationControllinoMaxi2"
#define secret_mqtt_username "IrrigationController2"
#define secret_mqtt_password "REPLACE_ME"

// MAC address (must be unique on your network)
#define secret_byte { 0xDE, 0xA1, 0xAD, 0xEF, 0xED, 0xFE }

#define secret_commandTopic1 "HOME/Irrigation2/OutputOne/Command"
#define secret_stateTopic1   "HOME/Irrigation2/OutputOne/State"

#define secret_commandTopic2 "HOME/Irrigation2/OutputTwo/Command"
#define secret_stateTopic2   "HOME/Irrigation2/OutputTwo/State"

#define secret_commandTopic3 "HOME/Irrigation2/OutputThree/Command"
#define secret_stateTopic3   "HOME/Irrigation2/OutputThree/State"

#define secret_commandTopic4 "HOME/Irrigation2/OutputFour/Command"
#define secret_stateTopic4   "HOME/Irrigation2/OutputFour/State"

#define secret_publishLastWillTopic       "HOME/Irrigation2/Status"
#define secret_publishClientName          "HOME/Irrigation2/ClientName"
#define secret_publishIpAddress           "HOME/Irrigation2/IpAddress"
#define secret_publishSignalStrength      "HOME/Irrigation2/SignalStrength"
#define secret_publishHostName            "HOME/Irrigation2/HostName"
#define secret_publishSSID                "HOME/Irrigation2/SSID"

#define secret_publishNodeStatusJsonTopic "HOME/Irrigation2/StatusJSON"
#define secret_publishNodeHealthJsonTopic "HOME/Irrigation2/NodeHealthJSON"

#endif // IRRIGATION_CONTROLLER
// Private.h - Example configuration file for secrets and settings
// Replace the placeholder values with your actual credentials and settings.
// Do NOT commit your real secrets to version control.

#ifndef PRIVATE_H
#define PRIVATE_H




#endif // PRIVATE_H
