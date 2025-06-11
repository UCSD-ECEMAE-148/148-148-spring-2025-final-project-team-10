/*****************************************************************************/
//  HighLevelExample.ino
//  Hardware:      Grove - 6-Axis Accelerometer&Gyroscope
//	Arduino IDE:   Arduino-1.65
//	Author:	       Lambor
//	Date: 	       Oct,2015
//	Version:       v1.0
//
//  Modified by:
//  Data:
//  Description:
//
//	by www.seeedstudio.com
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
/*******************************************************************************/

#include <Arduino.h>
#include "LSM6DS3.h"
//#include <SparkFunLSM6DS3.h> 
#include "Wire.h"

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A
float yaw = 0.0;          // Yaw angle (in degrees)
unsigned long prevTime = 0;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    yaw = 0.0;  
    while (!Serial);
    //Call .begin() to configure the IMUs
    if (myIMU.begin() != 0) {
        Serial.println("Device error");
    } else {
        Serial.println("Device OK!");
    }

}

void loop() {
 
   if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); // Remove whitespace

    if (cmd == "RESET") {
      Serial.println("Resetting...");
      delay(100);  // Give time to send response
      // NVIC_SystemReset();  // Perform software reset
      yaw = 0.0;
    }
  }
  
    float ax = myIMU.readFloatAccelX();
    float ay = myIMU.readFloatAccelY();
    float az = myIMU.readFloatAccelZ();
    float gx = myIMU.readFloatGyroX();
    float gy = myIMU.readFloatGyroY();
    float gz = myIMU.readFloatGyroZ();

    /*
    Serial.print("gx:"); Serial.print(gx); Serial.print("\t");
    Serial.print("gy:"); Serial.print(gy); Serial.print("\t");
    Serial.print("gz:"); Serial.println(gz);
    */
    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0;
    prevTime = currentTime;
    if(gz < 1 && gz > -1) gz = 0;

    // Integrate gyroZ over time to get yaw angle
    yaw += gz * dt;

     // Normalize yaw to -180 to 180 degrees
     if (yaw > 180.0) yaw -= 360.0;
     if (yaw < -180.0) yaw += 360.0;


    // Convert to 0–360 range
    if (yaw < 0) yaw += 360.0;

    // Serial.print("gz:"); Serial.print(gz);
    // Serial.print("Yaw (Δθ): "); Serial.println(yaw, 2);  // Print yaw with 2 decimal places
    Serial.println(yaw, 2);
    delay(50); 
}



