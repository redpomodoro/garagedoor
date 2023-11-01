#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <ArduinoMqttClient.h>
#include "arduino_secrets.h"

// Hardware specific
const int relay_open_pin        = 1;            // the number of the relay 1 pin
const int relay_close_pin       = 2;            // the number of the relay 2 pin
const int contact_pin           = A1;           // the number of the contact sensor pin - door closed     

// Generic
const long config_update        = 60UL * 1000L; // interval for sending config and availability update
const long relay_delay          = 100;          // time period to active door open/close relay [ms]
const long execution_delay      = 20L * 1000L;  // delay to wait for the door to fully open or close
const long sensor_state_delay   = 2L * 1000L;   // delay to ignore contact sensor when contact sensor state changed

const char ssid[] = SECRET_SSID;        // your network SSID (name)
const char pass[] = SECRET_PASS;        // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;            // the Wifi radio's status

const char broker[]             = SECRET_MQTT_SERVER;
const int  port                 = 1883; 
const String topic_root         = "homeassistant";
const String device_prefix      = "gdctrl_";

unsigned long last_config_update = 0;           // when last config was send
unsigned long now = 0;                          // current millis
unsigned long change_start = 0;                 // start time when change has been triggered

int contact_state;              // current state of door sensor fully closed
int last_contact_state;         // previous state of door sensor fully closed

String mac_address = "";
String topic_action = "";
String topic_state = "";
String topic_config = "";
String topic_availability = "";
String config = "";
String state = "";
String last_state = "";
int delay_loop = 0;

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

void setup() {
  // initialize contact sensor pin:
  pinMode(contact_pin, INPUT_PULLUP);
  // initialize relay pins:
  pinMode(relay_open_pin, OUTPUT);
  pinMode(relay_close_pin, OUTPUT);

  state = get_door_state();
  last_state = state;

  Serial.begin(9600);
  Serial.println("Init...");

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println(F("Communication with WiFi module failed!"));
    // don't continue
    while (true);
  }
  
  // check WiFi firmware
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println(F("Please upgrade the firmware"));
  }
  
  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
    if (status != WL_CONNECTED) {
      // wait 10 seconds for next attempt:
      delay(1000);
    }
  }

  // Get device id and set MQTT topics
  byte mac[6];
  WiFi.macAddress(mac);
  String mac_address = mac2String(mac);

  String device_id = device_prefix + mac_address;
  topic_action = topic_root + "/cover/" + device_id + "/set";
  topic_state = topic_root + "/cover/" + device_id + "/state";
  topic_config = topic_root + "/cover/" + device_id + "/config";
  topic_availability = topic_root + "/cover/" + device_id + "/availability";

  config = "{\"device_class\":\"garage\",\"suggested_area\":\"garage\",\"name\":\"" + device_id + "\",\"unique_id\":\"" + device_id + "\",\"object_id\":\"" + device_id + "\",\"command_topic\":\"" + topic_action + "\",\"state_topic\": \"" + topic_state + + "\",\"availability_topic\":\"" + topic_availability + "\"}";

  mqttClient.setId(device_id);
  mqttClient.setUsernamePassword(SECRET_MQTT_USER, SECRET_MQTT_PASS);
}

void loop() {  
  delay_loop = 0;
  last_state = state;
  last_contact_state = contact_state;
  now = millis();

  // Check for active WiFi/MQTT connection
  if(!mqttClient.connected()){
    Serial.println("MQTT disconnected");
    if(WiFi.status() != WL_CONNECTED){
      Serial.print("Attempting to connect to WPA SSID: ");
      Serial.println(ssid);
      status = WiFi.begin(ssid, pass);
      if (status != WL_CONNECTED) {
        // wait 10 seconds for next attempt:
        delay(1000);
      }
    }
    if(WiFi.status() == WL_CONNECTED){
      if(!attemptReconnect()){
        mqttClient.subscribe(topic_action);
        Serial.println("MQTT connected");
        send_mqtt_config();
      }
    }
  }

  // Listen for action from MQTT
  int messageSize = mqttClient.parseMessage();
  if (messageSize) {
    // we received a message, print out the topic and contents
    Serial.print("Received a message with topic '");
    Serial.print(mqttClient.messageTopic());
    Serial.print("', length ");
    Serial.print(messageSize);
    Serial.println(" bytes:");

    String action;
    char c;

    // use the Stream interface to print the contents
    while (mqttClient.available()) {
      c = mqttClient.read();
      action = String(action + c);
    }
    action.toLowerCase();
    Serial.println("action: " + action);
    if(action == "open") {
      trigger_relay(relay_open_pin, "opening");
      delay_loop = sensor_state_delay;
    } else if(action == "close") {
      trigger_relay(relay_close_pin, "closing");
      delay_loop = sensor_state_delay;
    } else if(action == "stop") {
      if (state == "opening") {
        trigger_relay(relay_close_pin, "stopping");
      } else if (state == "closing") {
        trigger_relay(relay_open_pin, "stopping");
      }
    }
  } else {
    // read new contact state
    contact_state  = digitalRead(contact_pin); 
    if (last_contact_state == LOW && contact_state == HIGH) { // state change: LOW -> HIGH
      if (change_start==0) {
          // triggered by external button or remote
          Serial.println("The door-opening detected by contact sensor");
          state = "opening";
          change_start = now;
      } else if (last_state != "opening") {
        Serial.println("Unexpected last state with door open. last state:" + last_state);
      }
    } else if (last_contact_state == HIGH && contact_state == LOW) { // state change: HIGH -> LOW
        Serial.println("The door-closed is detected");
        state = "closed";
        delay_loop = sensor_state_delay;
        change_start = 0;
    } else if (last_state == "stopping") {
        state = get_door_state();
        Serial.println(last_state + " expired. state set to: " + state);
        change_start = 0;
    } else if (last_state == "opening" || last_state == "closing") {
      if (now > change_start) {
        if ((unsigned long)(now - change_start) >= execution_delay) {
          state = get_door_state();
          Serial.println(last_state + " expired. state set to: " + state);
          change_start = 0;
        }
      } else {
        // overflow given more time
        change_start = now;
        Serial.println("change execution overflow");
      }
    }
  }
  
  if (state != last_state) {
    sendMqttMessage(topic_state, state);
    if (delay_loop > 0) {
      Serial.println("waiting " + String(delay_loop));
      delay(delay_loop);    
    }
  }

  // Send Config
  if (now > last_config_update) {
    if ((unsigned long)(now - last_config_update) >= config_update) {
      send_mqtt_config();
    }
  } else { 
     // handle overflow
    last_config_update = now;
  }
}

void send_mqtt_config() {
  sendMqttMessage(topic_config, config, true);
  sendMqttMessage(topic_availability, "online", true);
  sendMqttMessage(topic_state, state);
  last_config_update = millis();
}

String get_door_state() {
  int contact_state = digitalRead(contact_pin); // read new state
  String s = "";
  if (contact_state == LOW) {
    s = "closed";
  } else {
    s = "open";
  }
  return s;
}

void trigger_relay(int relay_id, String new_state) {
    digitalWrite(relay_id, HIGH);
    delay(relay_delay); 
    digitalWrite(relay_id, LOW);
    state = new_state;
    change_start = now;
}

String mac2String(byte ar[]) {
  String s;
  for (byte i = 0; i < 6; ++i)
  {
    char buf[3];
    sprintf(buf, "%02X", ar[i]); // J-M-L: slight modification, added the 0 in the format for padding 
    s += buf;
  }
  return s;
}

void sendMqttMessage(String topic, String text) {
    sendMqttMessage(topic, text, false);
}

void sendMqttMessage(String topic, String text, bool retain) {
    if (text.length() > 256) {
      mqttClient.beginMessage(topic, (unsigned long)text.length(), retain);
      mqttClient.print(text.substring(0,255));
      mqttClient.print(text.substring(255));
    } else {
      mqttClient.beginMessage(topic, retain);
      mqttClient.print(text);
    }
    mqttClient.endMessage();
    Serial.print("mqtt: ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(text);
}

int attemptReconnect(){
  if (!mqttClient.connect(broker, port)) {
      Serial.print("MQTT connection failed! Error code = ");
      Serial.println(mqttClient.connectError());
  }
  return mqttClient.connectError();
}
