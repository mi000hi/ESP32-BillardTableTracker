// WiFi Connection and MQTT
#include "WiFi.h"
#include "PubSubClient.h"

// for mac change
#include <esp_wifi.h>
#include <esp_now.h>

// for HTTP request
#include <HTTPClient.h>

// MPU6050 sensor
#include "Wire.h"
#include "MPU6050.h"

// to parse json
#include <ArduinoJson.h>

#include "arduino_secrets.h"

// #define USE_WIFI_EDUROAM 0
#define USE_CUSTOM_MAC 0
#define FIRST_WIFI_NETWORK 0
#define ENABLE_SLEEP 1
#define ENABLE_MEASUREMENTS 1
#define ENABLE_LED_BLINKS 0

#define HOUR_MORNING_START 8
#define HOUR_EVENING_END 17

// deep sleep variables
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define SECONDS_TO_SLEEP_DEFAULT 60*5;
long seconds_to_sleep = SECONDS_TO_SLEEP_DEFAULT;
RTC_DATA_ATTR int bootCount = 0;

// Function declarations
void broker_callback(char *topic, byte *payload, unsigned int length);


// general variables
uint16_t loop_delay_milliseconds = 100;

// MPU6050 @0x68
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
float acceleration_x, acceleration_y, acceleration_z;
float max_acc_x, max_acc_y, max_acc_z;

float acc_threshold = 0.015;
int values_over_threshold = 0;
int nr_measurements_per_period = 100;
int nr_measurements_taken = 0;
int period_counter = 0;
int nr_periods_before_sleep = 5;
bool go_to_sleepmode = false;
int deep_sleep_minute_interval = 5;

RTC_DATA_ATTR int last_threshold_ratios_index = 0;
#define LAST_THRESHOLD_RATIOS_LENGTH 20
RTC_DATA_ATTR float last_threshold_ratios[LAST_THRESHOLD_RATIOS_LENGTH];

// WiFi Connection Information
WiFiClient wifiClient;
// char wifi_ssid_home[] = SECRET_SSID_HOME;    // your network SSID (name)
// char wifi_pass_home[] = SECRET_PASS_HOME;    // your network password (use for WPA, or use as key for WEP)
// char wifi_ssid_eth[] = SECRET_SSID_ETH;  // your WPA2 enterprise network SSID (name)
// char wifi_user_eth[] = SECRET_USER_ETH;  // your WPA2 enterprise username
// char wifi_pass_eth[] = SECRET_PASS_ETH;  // your WPA2 enterprise password
uint8_t wifi_network = FIRST_WIFI_NETWORK; // selects the network to connect to
uint8_t masterCustomMac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33}; //Custom mac address

HTTPClient httpClient;
#define TIMESERVER_URL "https://www.timeapi.io/api/Time/current/zone?timeZone=Europe/Zurich"

// ESP32 lite pin declaration
#define MPU_PIN 16
#define MPU_ON 1
#define MPU_OFF 0
#define LED_PIN 22
#define LED_ON 0
#define LED_OFF 1
int8_t led_state = LED_OFF;

#define SERIAL_BAUDRATE 115200
#define SDA_PIN 0
#define SCL_PIN 4

// MQTT
#define MSG_BUFFER_SIZE	50
char msg[MSG_BUFFER_SIZE];

#define BROKER_CLIENT_ID "ESP32 Lolin Lite"
const char broker[] = BROKER_URL;
int        broker_port    = BROKER_PORT;
const char topic_loop_delay[] = "esp32/loop/delay";
const char topic_led[]    = "esp32/led";
const char topic_acc_x[]  = "esp32/acc/x";
const char topic_acc_y[]  = "esp32/acc/y";
const char topic_acc_z[]  = "esp32/acc/z";
const char topic_test_int[] = "esp32/test/int";
const char topic_acc_threshold[] = "esp32/acc/threshold";
const char topic_acc_over_threshold[] = "esp32/acc/over_threshold";
const char topic_sleep_seconds[] = "esp32/sleep/seconds";
const char topic_sleep_until[] = "esp32/sleep/until";
const char topic_acc_last_over_threshold[] = "esp32/acc/last_over_threshold";
const char topic_request[] = "esp32/request";
const char topic_info[]   = "esp32/info";
PubSubClient mqttClient(broker, broker_port, broker_callback, wifiClient);

// other variables
int some_variable = 0;

/*
 * FUNCTIONS
 */
void broker_callback(char *topic, byte *payload, unsigned int length) {

  payload[length] = 0;

  Serial.printf("\nMessage arrived in topic: %s\n", topic);
  Serial.printf("Message: %s", (char*) payload);
  Serial.printf("\n************\n");

  // set LED state
  if(strcmp(topic, topic_led) == 0) {
    if (strcmp((char*) payload, "on") == 0) {
      digitalWrite(LED_PIN, LED_ON);
    } else {
      digitalWrite(LED_PIN, LED_OFF);
    }
    return;
  }

  // acceleration threshold
  if(strcmp(topic, topic_acc_threshold) == 0) {
    // convert value to float
    acc_threshold = atof((char*) payload);
    Serial.printf("New acc_threshold: %f\n", acc_threshold);
  }

  // some_variable
  if(strcmp(topic, topic_test_int) == 0) {
    // convert value to int
    some_variable = atoi((char*) payload);
    Serial.printf("New value of some_variable: %d\n", some_variable);
  }

  // sleep seconds
  if(strcmp(topic, topic_sleep_seconds) == 0) {
    // convert value to long
    long seconds = (long) atoi((char*) payload);
    go_to_deep_sleep(seconds);
  }

  // request topic
  if(strcmp(topic, topic_request) == 0) {
    if(strcmp((char*) payload, "last_over_threshold") == 0) {
      // requested last_over_threshold
      Serial.printf("Last over threshold ratios requested...\n");
      mqtt_publish_last_threshold_ratios();
    }
  }

}

void initialize_wifi() {

  char* wifi_ssid; // = wifi_ssid_home;
  char* wifi_pass;

  // char* wifi_user;
  // char* wifi_pass = wifi_pass_home;
  // if (USE_WIFI_EDUROAM == 1) {
  //   wifi_ssid = wifi_ssid_eth;
  //   wifi_user = wifi_user_eth;
  //   wifi_pass = wifi_pass_eth;
  // }

  while (WiFi.status() != WL_CONNECTED) {

    // initialize WiFi device
    switch(wifi_network) {
      case 0: wifi_ssid = SECRET_SSID_ETH;    wifi_pass = SECRET_PASS_ETH;    break;
      case 1: wifi_ssid = SECRET_SSID_ZH;     wifi_pass = SECRET_PASS_ZH;     break;
      case 2: wifi_ssid = SECRET_SSID_PHONE;  wifi_pass = SECRET_PASS_PHONE;  break;
      case 3: wifi_ssid = SECRET_SSID_SG;     wifi_pass = SECRET_PASS_SG;     break;    

      default: Serial.printf("ERROR: could not select a wifi network: wifi_network=%d\n", wifi_network);
    }

    Serial.printf("\nConnecting to WIFI SSID: %s ", wifi_ssid);
    WiFi.disconnect(true);  //disconnect form wifi to set new wifi connection
    WiFi.mode(WIFI_STA);

    // set custom MAC address
    if(USE_CUSTOM_MAC == 1 && esp_wifi_set_mac(WIFI_IF_STA, &masterCustomMac[0]) != 0) {
      Serial.println("\nERROR: could not change MAC address.");
    }

    // connect to network
    if(wifi_network == 0) {
      WiFi.begin(wifi_ssid, WPA2_AUTH_PEAP, SECRET_USER_ETH, SECRET_USER_ETH, wifi_pass);
    } else {
      WiFi.begin(wifi_ssid, wifi_pass);
    }
    for (int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++) {
      Serial.printf(".");
      delay(1000);
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf(" CONNECTED\n");
      Serial.printf("IP: ");
      Serial.println(WiFi.localIP());
      break;
    }

    Serial.printf(" TIMEOUT\n");
    wifi_network = (wifi_network + 1) % 4;
  }
}

void initialize_i2c_devices() {

  // I2C setup
  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
  delay(200);

  // initialize MPU6050
  do {
    Serial.print("\nInitializing MPU6050 ...");
    accelgyro.initialize();
    delay(200);
    accelgyro.setFullScaleAccelRange(MPU6050_IMU::MPU6050_ACCEL_FS_2);

    // verify device connections
  } while(!accelgyro.testConnection());
  Serial.println(accelgyro.testConnection() ? "SUCCESS" : "FAILED");

  Serial.printf("Calibrating MPU6050 ...");
  accelgyro.CalibrateAccel();

  // verify device connections
  Serial.println(accelgyro.testConnection() ? "SUCCESS" : "FAILED");
}

void mqtt_publish_acc_data() {

  // publish acceleration in x y and z  
  snprintf (msg, MSG_BUFFER_SIZE, "%f", acceleration_x);
  mqttClient.publish(topic_acc_x, msg);
  snprintf (msg, MSG_BUFFER_SIZE, "%f", acceleration_y);
  mqttClient.publish(topic_acc_y, msg);
  snprintf (msg, MSG_BUFFER_SIZE, "%f", acceleration_z-1);
  mqttClient.publish(topic_acc_z, msg);
}

void mqtt_publish_max_acceleration() {

  // publish max acceleration in x y and z  
  snprintf (msg, MSG_BUFFER_SIZE, "%f", max_acc_x);
  mqttClient.publish(topic_acc_x, msg);
  snprintf (msg, MSG_BUFFER_SIZE, "%f", max_acc_y);
  mqttClient.publish(topic_acc_y, msg);
  snprintf (msg, MSG_BUFFER_SIZE, "%f", max_acc_z-1);
  mqttClient.publish(topic_acc_z, msg);

  Serial.printf("PUB: esp32/acc: x: %.4f,\t y: %.4f,\t z: %.4f\n", max_acc_x, max_acc_y, max_acc_z);
}

void mqtt_publish_acc_threshold_measurement() {

  double ratio = (float) values_over_threshold/nr_measurements_taken;
  snprintf(msg, MSG_BUFFER_SIZE, "%.6f", ratio);
  Serial.printf("PUB: esp32/acc/over_threshold: %s\n", msg);
  mqttClient.publish(topic_acc_over_threshold, msg);
}

void mqtt_publish_sleep_until(long seconds) {

  snprintf(msg, MSG_BUFFER_SIZE, "%ld", seconds);
  Serial.printf("PUB: esp32/sleep/until: %s\n", msg);
  mqttClient.publish(topic_sleep_until, msg);
}

void mqtt_publish_last_threshold_ratios() {
  for(int i = 0; i < LAST_THRESHOLD_RATIOS_LENGTH; i++) {
    snprintf(msg, MSG_BUFFER_SIZE, "%.6f", last_threshold_ratios[(last_threshold_ratios_index + i)%LAST_THRESHOLD_RATIOS_LENGTH]);
    Serial.printf("PUB: esp32/acc/last_over_threshold: %s\n", msg);
    mqttClient.publish(topic_acc_last_over_threshold, msg);
    delay(200);
  }
}

void mqtt_publish(const char* topic, const char* payload) {
  Serial.printf("PUB: %s: %s\n", topic, payload);
  mqttClient.publish(topic, payload);
}

void go_to_deep_sleep(long seconds) {

  mqtt_publish_sleep_until(seconds);
  mqtt_publish(topic_info, "offline");

   // go into deep sleep
  esp_sleep_enable_timer_wakeup(seconds * uS_TO_S_FACTOR);

  // shutdown MPU
  digitalWrite(MPU_PIN, MPU_OFF);
  // rtc_gpio_isolate((gpio_num_t) MPU_PIN);
  // gpio_hold_en((gpio_num_t) MPU_PIN);
	gpio_deep_sleep_hold_en(); 

  Serial.printf("Going to sleep now for %ld seconds...\n", seconds);
  Serial.flush(); 
  while(1) {
    esp_deep_sleep_start();
    delay(1000);
  }
}

void initialize_mqtt() {

  randomSeed(micros());
  mqttClient.setServer(broker, broker_port);
  mqttClient.setCallback(broker_callback);

  while (!mqttClient.connected()) {

    if(WiFi.status() != WL_CONNECTED) {
      Serial.printf("\nERROR: WiFi has been disconnected...");     
      initialize_wifi();
      if(WiFi.status() != WL_CONNECTED) {
        go_to_deep_sleep(60);
      }
    }

    // generate a random client name
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);

    Serial.printf("\nConnecting to MQTT broker %s as %s ... ", broker, clientId.c_str());

    // Attempt to connect
    if (!mqttClient.connect(clientId.c_str())) {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
  Serial.printf("CONNECTED\n");

  // subscribing to to topics
  mqttClient.subscribe(topic_led);
  mqttClient.subscribe(topic_test_int);
  mqttClient.subscribe(topic_acc_threshold);
  mqttClient.subscribe(topic_sleep_seconds);
  mqttClient.subscribe(topic_request);
}

void acceleration_raw_to_floats() {
  int lsb_per_g = 16384; // for +-2g range
  acceleration_x = (float) ax / lsb_per_g;
  acceleration_y = (float) ay / lsb_per_g;
  acceleration_z = (float) az / lsb_per_g;
}

void print_acceleration() {

  // floats
  Serial.printf("x: %.4f,\t y: %.4f,\t z: %.4f\n", acceleration_x, acceleration_y, acceleration_z);
}

void setup() {

  // initialize serial communication
  Serial.begin(SERIAL_BAUDRATE);

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  // configure Arduino LED pin for output
  // gpio_hold_dis((gpio_num_t) MPU_PIN);
	// gpio_deep_sleep_hold_dis(); 
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF); // turn LED off
  led_state = LED_OFF;

  // power up the MPU
  pinMode(MPU_PIN, OUTPUT);
  digitalWrite(MPU_PIN, MPU_OFF);
  delay(200);
  digitalWrite(MPU_PIN, MPU_ON);
  delay(200);
  
  initialize_i2c_devices();
  initialize_wifi();
  initialize_mqtt();

  mqtt_publish(topic_info, "online");

  period_counter = 0;

  // toggle led few times
  Serial.printf("Setup complete...\n");
  for(int i = 0; ENABLE_LED_BLINKS == 1 && i < 20; i++) {
    led_state = (led_state+1)%2;
    digitalWrite(LED_PIN, led_state);
    delay(200);
  }

  // run loop for billiard measurements
  acceleration_billiard_program();
  // this is never reached
}

void acceleration_billiard_program() {

float total_over_threshold_ratio = 0;

  if(ENABLE_MEASUREMENTS) {
    for (int j = 0; ENABLE_SLEEP == 0 || j < nr_periods_before_sleep; j++) {

      // read raw accel/gyro measurements from device
      // use this loop to be still able to use MQTT
      for(int i = 0; i < nr_measurements_per_period; i++) {
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        acceleration_raw_to_floats();

        // check if values are over the threshold
        if(abs(acceleration_x) > acc_threshold || abs(acceleration_y) > acc_threshold || abs(acceleration_z-1) > acc_threshold) {
          values_over_threshold += 1;      
        }
        nr_measurements_taken += 1;

        if(abs(acceleration_x) > max_acc_x) {
          max_acc_x = abs(acceleration_x);
        }
        if(abs(acceleration_y) > max_acc_y) {
          max_acc_y = abs(acceleration_y);
        }
        if(abs(acceleration_z) > max_acc_z) {
          max_acc_z = abs(acceleration_z);
        }

        mqttClient.loop();
        delay(loop_delay_milliseconds);
      }

      if(nr_measurements_taken >= nr_measurements_per_period) {
        mqtt_publish_acc_threshold_measurement();
        total_over_threshold_ratio += (float) values_over_threshold/nr_measurements_taken;
        nr_measurements_taken = 0;
        values_over_threshold = 0;

        mqtt_publish_max_acceleration();
        max_acc_x = 0;
        max_acc_y = 0;
        max_acc_z = 0;

        // accelgyro.CalibrateAccel();
      }

      // display tab-separated accel/gyro x/y/z values
      // print_acceleration();

      // send values via mqtt
      // mqtt_publish_acc_data();

      // mqtt client loop
      mqttClient.loop();
    }
  }

  // save current over threshold ratio
  last_threshold_ratios[last_threshold_ratios_index] = total_over_threshold_ratio / nr_periods_before_sleep;
  last_threshold_ratios_index = (last_threshold_ratios_index + 1)%LAST_THRESHOLD_RATIOS_LENGTH;

  // GOTO deep sleep

  // determine sleep time

  // start connection and send HTTP header
  // Serial.printf("Sending HTTP time request...\n");
  httpClient.begin(TIMESERVER_URL); // HTTP request
  int httpCode = httpClient.GET();
  String payload_json;
  if(httpCode > 0) { // negative code is an error
    // HTTP header has been send and Server response header has been handled
    // Serial.printf("[HTTP] GET... code: %d: %s\n", httpCode, httpClient.errorToString(httpCode).c_str());
 
    // file found at server
    if(httpCode == HTTP_CODE_OK) {
        payload_json = httpClient.getString();
        // Serial.println(payload_json);
        // Serial.println("Length: " + payload_json.length());
    }
  } else {
    Serial.printf("[HTTP] GET... failed, error: %s\n", httpClient.errorToString(httpCode).c_str());
  }

  // parse json payload
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, payload_json);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  // save time
  int hours = doc["hour"];
  int minutes = doc["minute"];
  int seconds = doc["seconds"];

  // Serial.printf("Current Time: %d:%d:%d\n", hours, minutes, seconds);

  // seconds until next 5min
  seconds_to_sleep =  60 - seconds + 60*( (deep_sleep_minute_interval - 1) - minutes%deep_sleep_minute_interval );

  // sleep overnight
  if (hours >= HOUR_EVENING_END || hours < HOUR_MORNING_START) {    
    seconds_to_sleep = (60-seconds) + 60*(60-1-minutes) + 60*60*(HOUR_MORNING_START-1-hours);
    // Serial.printf("DEBUG: seconds_to_sleep=%d\n", seconds_to_sleep);
    if (seconds_to_sleep < 0) {
      seconds_to_sleep = (60-seconds) + 60*(60-1-minutes) + 60*60*(HOUR_MORNING_START-1 + 24-hours);
    // Serial.printf("DEBUG: seconds_to_sleep=%d\n", seconds_to_sleep);
    }
  }

  // make sure seconds are positive
  if (seconds_to_sleep < 5) {
    seconds_to_sleep = SECONDS_TO_SLEEP_DEFAULT;
  }

  // go into deep sleep
  go_to_deep_sleep(seconds_to_sleep);
  // this code is not reached
}

void loop() {
  // should not be called
  Serial.printf("ERROR: Should not be in main loop\n");
  delay(1000);

  go_to_deep_sleep(60);

  // mqtt client loop -- just in case
  mqttClient.loop();
}
