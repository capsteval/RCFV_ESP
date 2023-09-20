// Copyright (c) Microsoft Corporation. All rights reserved.
// SPDX-License-Identifier: MIT

/*
 * This is an Arduino-based Azure IoT Hub sample for ESPRESSIF ESP32 boards.
 * It uses our Azure Embedded SDK for C to help interact with Azure IoT.
 * For reference, please visit https://github.com/azure/azure-sdk-for-c.
 *
 * To connect and work with Azure IoT Hub you need an MQTT client, connecting, subscribing
 * and publishing to specific topics to use the messaging features of the hub.
 * Our azure-sdk-for-c is an MQTT client support library, helping composing and parsing the
 * MQTT topic names and messages exchanged with the Azure IoT Hub.
 *
 * This sample performs the following tasks:
 * - Synchronize the device clock with a NTP server;
 * - Initialize our "az_iot_hub_client" (struct for data, part of our azure-sdk-for-c);
 * - Initialize the MQTT client (here we use ESPRESSIF's esp_mqtt_client, which also handle the tcp
 * connection and TLS);
 * - Connect the MQTT client (using server-certificate validation, SAS-tokens for client
 * authentication);
 * - Periodically send telemetry data to the Azure IoT Hub.
 *
 * To properly connect to your Azure IoT Hub, please fill the information in the `iot_configs.h`
 * file.
 */

// C99 libraries
#include <cstdlib>
#include <string.h>
#include <time.h>

// Libraries for MQTT client and WiFi connection
#include <WiFi.h>
#include <mqtt_client.h>

// Azure IoT SDK for C includes
#include <az_core.h>
#include <az_iot.h>
#include <azure_ca.h>

// Additional sample headers
#include "AzIoTSasToken.h"
#include "SerialLogger.h"
#include "iot_configs.h"

//for json manipulating
#include <ArduinoJson.h>
//SERVO MOTOR LIBRARY
#include <ESP32Servo.h>


// When developing for your own Arduino-based platform,
// please follow the format '(ard;<platform>)'.
#define AZURE_SDK_CLIENT_USER_AGENT "c%2F" AZ_SDK_VERSION_STRING "(ard;esp32)"

// Utility macros and defines
#define sizeofarray(a) (sizeof(a) / sizeof(a[0]))
#define NTP_SERVERS "pool.ntp.org", "time.nist.gov"
#define MQTT_QOS1 1
#define DO_NOT_RETAIN_MSG 0
#define SAS_TOKEN_DURATION_IN_MINUTES 60
#define UNIX_TIME_NOV_13_2017 1510592825

#define PST_TIME_ZONE -8
#define PST_TIME_ZONE_DAYLIGHT_SAVINGS_DIFF 1

#define GMT_OFFSET_SECS (PST_TIME_ZONE * 3600)
#define GMT_OFFSET_SECS_DST ((PST_TIME_ZONE + PST_TIME_ZONE_DAYLIGHT_SAVINGS_DIFF) * 3600)

// Translate iot_configs.h defines into variables used by the sample
static const char* ssid = IOT_CONFIG_WIFI_SSID;
static const char* password = IOT_CONFIG_WIFI_PASSWORD;
static const char* host = IOT_CONFIG_IOTHUB_FQDN;
static const char* mqtt_broker_uri = "mqtts://" IOT_CONFIG_IOTHUB_FQDN;
static const char* device_id = IOT_CONFIG_DEVICE_ID;
static const int mqtt_port = AZ_IOT_DEFAULT_MQTT_CONNECT_PORT;

// Memory allocated for the sample's variables and structures.
static esp_mqtt_client_handle_t mqtt_client;
static az_iot_hub_client client;

static char mqtt_client_id[128];
static char mqtt_username[128];
static char mqtt_password[200];
static uint8_t sas_signature_buffer[256];
static unsigned long next_telemetry_send_time_ms = 0;
static char telemetry_topic[128];
static uint32_t telemetry_send_count = 0;
static String telemetry_payload = "{}";
static String sensor_payload = "{}";
StaticJsonDocument<512> telemetryDoc;

#define INCOMING_DATA_BUFFER_SIZE 128
static char incoming_data[INCOMING_DATA_BUFFER_SIZE];

HardwareSerial NodeMCU(1);  // Use HardwareSerial for ESP32 (Serial1: TX - GPIO4, RX - GPIO5)

//SERVO MOTOR DECLARATION SECTION
Servo myservo;
const int servoPin = 16;
int calibratedPosition = 0;
bool activatedSpinArm = false;
int pos=0;
//SERVO MOTOR DECLARATION SECTION END



//PUMP COMMAND DECLARATION SECTION
const int pump_relay = 4;  // the relay is connected to pin 2 of the Arduino board
//PUMP COMMAND DECLARATION SECTION END

//VEHICLE DIRECTION COMMAND DECLARATION SECTION
const int motorA1 = 12;  // Motor A direction pin 1 right
const int motorA2 = 13;  // Motor A direction pin 2
const int motorB1 = 14;  // Motor B direction pin 1 left
const int motorB2 = 15;  // Motor B direction pin 2
// const int motorForward = 4;   // Motor B direction pin 1 left
// const int motorBackward = 5;  // Motor B direction pin 2
//VEHICLE DIRECTION COMMAND DECLARATION SECTION


//VEHICLE SPEED COMMAND DECLARATION SECTION
// const int speedPinA = 6;  // Speed control pin for motor A
// const int speedPinB = 7;  // Speed control pin for motor B
const int speedPinAB = 0;
// Array of speed levels
const int speedLevels[] = { 0, 80, 115, 150, 185, 220, 255 };  // Seven speed levels 0,1,2,3,4,5,6
//VEHICLE SPEED COMMAND DECLARATION SECTION END



//CONNECTION STATUS DECLARATION SECTION
String connectionStatus = "Idle";
int connectionStatusCode = 0;
//CONNECTION STATUS DECLARATION SECTION END


//GLOBAL SERIAL COMMUNICATION VARIABLES DECLARATION SECTION
String GPSLocation = "";
String batteryPercentage = "";
String liquidLevelPerc = "";
String obstacleAhead = "";
//GLOBAL SERIAL COMMUNICATION VARIABLES DECLARATION SECTION END


// Auxiliary functions
#ifndef IOT_CONFIG_USE_X509_CERT
static AzIoTSasToken sasToken(
  &client,
  AZ_SPAN_FROM_STR(IOT_CONFIG_DEVICE_KEY),
  AZ_SPAN_FROM_BUFFER(sas_signature_buffer),
  AZ_SPAN_FROM_BUFFER(mqtt_password));
#endif  // IOT_CONFIG_USE_X509_CERT

static void connectToWiFi() {
  Serial.begin(115200);
  NodeMCU.begin(4800, SERIAL_8N1, 2);  // Specify RX and TX pins for HardwareSerial
  Logger.Info("Connecting to WIFI SSID " + String(ssid));

  WiFi.mode(WIFI_STA);
  // WiFi.disconnect();
  delay(100);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  connectionStatus = getWifiStatus(WiFi.status());
  connectionStatusCode = WiFi.status();
  Serial.println("");
  Logger.Info("WiFi connected, IP address: " + WiFi.localIP().toString());
}

static void initializeTime() {
  Logger.Info("Setting time using SNTP");

  configTime(GMT_OFFSET_SECS, GMT_OFFSET_SECS_DST, NTP_SERVERS);
  time_t now = time(NULL);
  while (now < UNIX_TIME_NOV_13_2017) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("");
  Logger.Info("Time initialized!");
}

void receivedCallback(char* topic, byte* payload, unsigned int length) {
  Logger.Info("Received [");
  Logger.Info(topic);
  Logger.Info("]: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println("");
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event) {
  switch (event->event_id) {
    int i, r;

    case MQTT_EVENT_ERROR:
      Logger.Info("MQTT event MQTT_EVENT_ERROR");
      break;
    case MQTT_EVENT_CONNECTED:
      Logger.Info("MQTT event MQTT_EVENT_CONNECTED");

      r = esp_mqtt_client_subscribe(mqtt_client, AZ_IOT_HUB_CLIENT_C2D_SUBSCRIBE_TOPIC, 1);
      if (r == -1) {
        Logger.Error("Could not subscribe for cloud-to-device messages.");
      } else {
        Logger.Info("Subscribed for cloud-to-device messages; message id:" + String(r));
      }

      break;
    case MQTT_EVENT_DISCONNECTED:
      Logger.Info("MQTT event MQTT_EVENT_DISCONNECTED");
      break;
    case MQTT_EVENT_SUBSCRIBED:
      Logger.Info("MQTT event MQTT_EVENT_SUBSCRIBED");
      break;
    case MQTT_EVENT_UNSUBSCRIBED:
      Logger.Info("MQTT event MQTT_EVENT_UNSUBSCRIBED");
      break;
    case MQTT_EVENT_PUBLISHED:
      Logger.Info("MQTT event MQTT_EVENT_PUBLISHED");
      break;
    case MQTT_EVENT_DATA:
      Logger.Info("MQTT event MQTT_EVENT_DATA");

      for (i = 0; i < (INCOMING_DATA_BUFFER_SIZE - 1) && i < event->topic_len; i++) {
        incoming_data[i] = event->topic[i];
      }
      incoming_data[i] = '\0';
      Logger.Info("Topic: " + String(incoming_data));

      for (i = 0; i < (INCOMING_DATA_BUFFER_SIZE - 1) && i < event->data_len; i++) {
        incoming_data[i] = event->data[i];
      }
      incoming_data[i] = '\0';
      Logger.Info("Data: " + String(incoming_data));
      Logger.Info("Stephen: ");
      processJson(String(incoming_data));

      break;
    case MQTT_EVENT_BEFORE_CONNECT:
      Logger.Info("MQTT event MQTT_EVENT_BEFORE_CONNECT");
      break;
    default:
      Logger.Error("MQTT event UNKNOWN");
      break;
  }

  return ESP_OK;
}

static void initializeIoTHubClient() {
  az_iot_hub_client_options options = az_iot_hub_client_options_default();
  options.user_agent = AZ_SPAN_FROM_STR(AZURE_SDK_CLIENT_USER_AGENT);

  if (az_result_failed(az_iot_hub_client_init(
        &client,
        az_span_create((uint8_t*)host, strlen(host)),
        az_span_create((uint8_t*)device_id, strlen(device_id)),
        &options))) {
    Logger.Error("Failed initializing Azure IoT Hub client");
    return;
  }

  size_t client_id_length;
  if (az_result_failed(az_iot_hub_client_get_client_id(&client, mqtt_client_id, sizeof(mqtt_client_id) - 1, &client_id_length))) {
    Logger.Error("Failed getting client id");
    return;
  }

  if (az_result_failed(az_iot_hub_client_get_user_name(
        &client, mqtt_username, sizeofarray(mqtt_username), NULL))) {
    Logger.Error("Failed to get MQTT clientId, return code");
    return;
  }

  Logger.Info("Client ID: " + String(mqtt_client_id));
  Logger.Info("Username: " + String(mqtt_username));
}

static int initializeMqttClient() {
#ifndef IOT_CONFIG_USE_X509_CERT
  if (sasToken.Generate(SAS_TOKEN_DURATION_IN_MINUTES) != 0) {
    Logger.Error("Failed generating SAS token");
    return 1;
  }
#endif

  esp_mqtt_client_config_t mqtt_config;
  memset(&mqtt_config, 0, sizeof(mqtt_config));
  mqtt_config.uri = mqtt_broker_uri;
  mqtt_config.port = mqtt_port;
  mqtt_config.client_id = mqtt_client_id;
  mqtt_config.username = mqtt_username;

#ifdef IOT_CONFIG_USE_X509_CERT
  Logger.Info("MQTT client using X509 Certificate authentication");
  mqtt_config.client_cert_pem = IOT_CONFIG_DEVICE_CERT;
  mqtt_config.client_key_pem = IOT_CONFIG_DEVICE_CERT_PRIVATE_KEY;
#else  // Using SAS key
  mqtt_config.password = (const char*)az_span_ptr(sasToken.Get());
#endif

  mqtt_config.keepalive = 30;
  mqtt_config.disable_clean_session = 0;
  mqtt_config.disable_auto_reconnect = false;
  mqtt_config.event_handle = mqtt_event_handler;
  mqtt_config.user_context = NULL;
  mqtt_config.cert_pem = (const char*)ca_pem;

  mqtt_client = esp_mqtt_client_init(&mqtt_config);

  if (mqtt_client == NULL) {
    Logger.Error("Failed creating mqtt client");
    return 1;
  }

  esp_err_t start_result = esp_mqtt_client_start(mqtt_client);

  if (start_result != ESP_OK) {
    Logger.Error("Could not start mqtt client; error code:" + start_result);
    return 1;
  } else {
    Logger.Info("MQTT client started");
    return 0;
  }
}

/*
 * @brief           Gets the number of seconds since UNIX epoch until now.
 * @return uint32_t Number of seconds.
 */
static uint32_t getEpochTimeInSecs() {
  return (uint32_t)time(NULL);
}

static void establishConnection() {
  connectToWiFi();
  initializeTime();
  initializeIoTHubClient();
  (void)initializeMqttClient();
  mySetup();
}

static void generateTelemetryPayload() {
  // You can generate the JSON using any lib you want. Here we're showing how to do it manually, for simplicity.
  // This sample shows how to generate the payload using a syntax closer to regular delevelopment for Arduino, with
  // String type instead of az_span as it might be done in other samples. Using az_span has the advantage of reusing the
  // same char buffer instead of dynamically allocating memory each time, as it is done by using the String type below.
  // telemetry_payload = "{ \"msgCount\": " + String(telemetry_send_count++) + " }";

  sensor_payload = "";
  serializeJson(telemetryDoc, sensor_payload);
  telemetry_payload = "{\"sensorValues\":" + sensor_payload + ",\"connection\":{\"connectionStatus\":\"" + connectionStatus + "\",\"connectionStatusCode\":" + String(connectionStatusCode) + "}}";
}

static void sendTelemetry() {
  Logger.Info("Sending telemetry ...");

  // The topic could be obtained just once during setup,
  // however if properties are used the topic need to be generated again to reflect the
  // current values of the properties.
  if (az_result_failed(az_iot_hub_client_telemetry_get_publish_topic(
        &client, NULL, telemetry_topic, sizeof(telemetry_topic), NULL))) {
    Logger.Error("Failed az_iot_hub_client_telemetry_get_publish_topic");
    return;
  }
  connectToSerialCommunication();
  generateTelemetryPayload();

  if (esp_mqtt_client_publish(
        mqtt_client,
        telemetry_topic,
        (const char*)telemetry_payload.c_str(),
        telemetry_payload.length(),
        MQTT_QOS1,
        DO_NOT_RETAIN_MSG)
      == 0) {
    Logger.Error("Failed publishing");
  } else {
    Logger.Info("Message published successfully");
  }
}

// Arduino setup and loop main functions.

void setup() {
   	myservo.setPeriodHertz(50);    // standard 50 hz servo
  	myservo.attach(servoPin, 1000, 2000); // attaches the servo on pin 18 to the servo object
  establishConnection();
}
void mySetup() {
  // Initialize motor control pins
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  //Initialize motor speed control pins
  pinMode(speedPinAB, OUTPUT);

  // // Initialize pump control pin
  pinMode(pump_relay, OUTPUT);
  // // Logger.Info("Before servo motor initialisation");
  // //   // //Intialize servo motor pin
  //   	myservo.setPeriodHertz(50);    // standard 50 hz servo
  // 	myservo.attach(servoPin, 1000, 2000); // attaches the servo on pin 18 to the servo object
  // //   //  // servoMotor.write(calibratedPosition);
  // //   Logger.Info("End of servo motor initialisation");
}
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  }
#ifndef IOT_CONFIG_USE_X509_CERT
  else if (sasToken.IsExpired()) {
    Logger.Info("SAS token expired; reconnecting with a new one.");
    (void)esp_mqtt_client_destroy(mqtt_client);
    initializeMqttClient();
  }
#endif
  else if (millis() > next_telemetry_send_time_ms) {
    ArmSpinner();
    sendTelemetry();
    next_telemetry_send_time_ms = millis() + TELEMETRY_FREQUENCY_MILLISECS;
  }
}
void processJson(String json) {
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, json);

  if (!error) {
    String component = doc["Component"];
    String command = doc["Command"];

    Serial.print("Received Component: ");
    Serial.println(component);
    Serial.print("Received Command: ");
    Serial.println(command);

    // Implement logic based on the received component and command
    if (component == "arm") {
      HandleArmSpin(command);
      Serial.println("HandleArmSpin");
    } else if (component == "wheel") {

      HandleVehicleMovement(command);
      Serial.println("HandleVehicleMovement");
    } else if (component == "pump") {
      HandlePump(command);
      Serial.println("HandlePump");
    }
  } else {
    Logger.Info("Error parsing JSON: ");
    Logger.Info(error.c_str());
  }
}


void HandleArmSpin(String armcommand) {
  Logger.Info("armcommand");
  Logger.Info(armcommand);
  if (armcommand == "spinarm") {
    Serial.println("spinning arm");
    activatedSpinArm = true;
  } else {
    myservo.write(calibratedPosition);
    activatedSpinArm = false;
  }
}

void ArmSpinner() {
  if (activatedSpinArm == true) {
    for (pos = 0; pos <= 180; pos += 1) {  // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      myservo.write(pos);  // tell servo to go to position in variable 'pos'
      delay(3);               // waits 15 ms for the servo to reach the position
    }
    for (pos = 180; pos >= 0; pos -= 1) {  // goes from 180 degrees to 0 degrees
      myservo.write(pos);               // tell servo to go to position in variable 'pos'
      delay(2);                            // waits 15 ms for the servo to reach the position
    }
  }
}


void HandlePump(String pumpcommand) {
  Logger.Info("pumpcommand(");
  Logger.Info(pumpcommand);
  if (pumpcommand == "startspray") {
    Serial.println("spraying");
    digitalWrite(pump_relay, HIGH);
  } else {
    Serial.println("spraying");
    digitalWrite(pump_relay, LOW);
  }
}

void MoveForward(int speed) {
  Logger.Info(String(speed));
  Serial.println("Moving Forward");
  setMotorSpeed(speedPinAB, speed);
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}

void MoveBackward(int speed) {
  Serial.println("Moving Backward");
  setMotorSpeed(speedPinAB, speed);
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
}

void stopMotors() {
  setMotorSpeed(speedPinAB, 0);
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
}
void MoveLeft(int speed) {
  Serial.println("Moving Left");
  setMotorSpeed(speedPinAB, speed / 2);
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
}

void MoveRight(int speed) {
  Serial.println("Moving Right");
  setMotorSpeed(speedPinAB, speed / 2);
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}

void setMotorSpeed(int speedPin, int speed) {
  Serial.print(speedPin);
  Serial.print(speed);
  Serial.print("speed");
  Serial.println(speedLevels[speed]);
  analogWrite(speedPin, speedLevels[speed]);  // Set PWM speed
}

void HandleVehicleMovement(String vehiclecommand) {
  StaticJsonDocument<200> vehicledoc;
  DeserializationError error = deserializeJson(vehicledoc, vehiclecommand);
  String dir = vehicledoc["Direction"];
  String speed = vehicledoc["Speed"];
  if (dir == "f") {
    MoveForward(speed.toInt());
  } else if (dir == "b") {
    MoveBackward(speed.toInt());
  } else if (dir == "l") {
    MoveLeft(speed.toInt());
  } else if (dir == "r") {
    MoveRight(speed.toInt());
  } else {
    stopMotors();
  }
}


String getWifiStatus(int status) {
  switch (status) {
    case WL_IDLE_STATUS:
      return "Idle";
    case WL_SCAN_COMPLETED:
      return "Scan Complete";
    case WL_NO_SSID_AVAIL:
      return "No network";
    case WL_CONNECT_FAILED:
      return "Connection failed";
    case WL_CONNECTION_LOST:
      return "Connection lost";
    case WL_CONNECTED:
      return "Connected";
    case WL_DISCONNECTED:
      return "Disconnected";
  }
}
static void connectToSerialCommunication() {

  CreateDefaultTelemetryDoc();
  if (NodeMCU.available() > 0) {
    GPSLocation = NodeMCU.readString();
    if (NodeMCU.read() == '\n') {
      Logger.Info("S_location: " + String(GPSLocation));
      Serial.print("S_location: ");
      Serial.println(GPSLocation);
      telemetryDoc["loc"] = String(GPSLocation);
    }

    batteryPercentage = NodeMCU.readString();
    if (NodeMCU.read() == '\n') {
      Logger.Info("S_Battery Percentage: " + String(batteryPercentage));
      Serial.print("S_Battery Percentage: ");
      Serial.println(batteryPercentage);
      telemetryDoc["bp"] = String(batteryPercentage);
    }

    liquidLevelPerc = NodeMCU.readString();
    if (NodeMCU.read() == '\n') {
      Logger.Info("S_Liquid level Percentage: " + String(liquidLevelPerc));
      Serial.print("S_Liquid level Percentage: ");
      Serial.println(liquidLevelPerc);
      telemetryDoc["lp"] = String(liquidLevelPerc);
    }

    obstacleAhead = NodeMCU.readString();
    if (NodeMCU.read() == '\n') {
      Logger.Info("S_Obstacle: " + String(obstacleAhead));
      Serial.print("S_Obstacle: ");
      Serial.println(obstacleAhead);
      telemetryDoc["oa"] = String(obstacleAhead);
    }
  }



  // if (NodeMCU.available() > 0) {
  //   sensor_payload = NodeMCU.readString();
  //   if (NodeMCU.read() == '\n') {
  //     Serial.print("sensor payload");
  //     Serial.println(sensor_payload);
  //   }
  //   String command = NodeMCU.readString();
  //   if (NodeMCU.read() == '\n') {
  //     if (command == "stop") {
  //       stopMotors();
  //     }
  //   }
  // }
}

void CreateDefaultTelemetryDoc() {
  // telemetryDoc["loc"]= String("78.32323");
  // telemetryDoc["bp"]= String(39);
  // telemetryDoc["lp"]= String(90);
  // telemetryDoc["oa"]= String("stop");
}