# ESP32-BillardTableTracker
---

This project consists of a `ESP32 Lolin Lite` microcontroller with an attached `MPU6050` accelerometer board. Its purpose is to measure the activity an a Billiard Table to see if it is used at the moment. The measured data is sent to a `MQTT` broker for easy access.

## Software

The project is written in the Arduino IDE. In addition to installing the included libraries, you need to create a `arduino_secrets.h` file including `ssid`, `username` and `password` to the WiFi network as well as the `url` and `port` to the `MQTT` broker.

## Hardware

The `MPU6050` is connected to the `ESP32` over the pins `SDA_PIN 0` and `SCL_PIN 4`. The power pin is `MPU_PIN 16`. This pin is toggled `off` when the `ESP32` goes into deep sleep.

---

## Functionality

...
