#!/bin/bash

# Compile the code and provide external path to library
arduino-cli compile \
  --fqbn Seeeduino:mbed:xiaonRF52840Sense \
  --libraries /home/jetson/Arduino/libraries/Seeed_Arduino_LSM6DS3 \
  /home/jetson/projects/esp32-loc/IMUcode

# Upload the code to the board
arduino-cli upload \
  -p /dev/ttyACM0 \
  --fqbn Seeeduino:mbed:xiaonRF52840Sense \
  /home/jetson/projects/esp32-loc/IMUcode

