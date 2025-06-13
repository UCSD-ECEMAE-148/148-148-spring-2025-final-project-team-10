# Autonomous Driving with Ultra-Wide Band (UWB) and Aruco Marker

<div align="center">
  <a href="https://jacobsschool.ucsd.edu/">
    <img src="media/images/UCSD_logo.png" alt="Logo" width="400" height="100">
  </a>
  <h3>ECE/MAE-148 Final Project</h3>
  <p>
    Spring 2025 - Team 10
  </p>
  <a>
    <img src="media/images/car.jpg" alt="Car View" height="335">
    <img src="media/images/team.jpeg" alt="Team-10 Group Photo" width="400" />

  </a>
</div>

<details>
  <summary>Table of Contents</summary>

  - [Team Members](#team-members)
  - [Abstract](#abstract)
  - [Goals](#goals)
    - [Must Have](#must-have)
    - [Nice to Have](#nice-to-have)
  - [Accomplishments](#accomplishments)
  - [Challenges Faced](#challenges-faced)
    - [Triangulation with ESP32](#triangulation-with-esp32)
    - [Driving with ESP32](#driving-with-esp32)
    - [Aruco Marker Detection](#aruco-marker-detection)
  - [Parts and Hardware](#parts-and-hardware)
    - [Wiring](#wiring)
  - [Acknowledgements](#acknowledgements)
  - [Contacts](#contacts)

  </details>


## Team Members
- **Efe Erturk** | ECE: BS in Computer Engineering 
- **Etka Uzun** | MAE: BS in Aerospace Engineering
- **Junran "Jason" Wang** | ECE: BS in Computer Engineering
- **Pushkal Mishra** | ECE: PhD in Wireless Sensing and ML
  
## Abstract
In this project, we developed an autonomous car following system using Ultra-Wide Band (UWB) technology and Aruco markers. The system is designed to enable a car to autonomously follow a UWB-enabled ESP32 tag and identify the tag with an Aruco marker. Additionally, the car can also follow the aruco marker once it is in the field-of-view. The project integrates various hardware components: NVIDIA Jetson Nano, OAK-D Lite camera, 3 ESP32 microcontroller with UWB support and IMU sensor, to achieve real-time distance measurement and control.

The aim of the project is to detect an ESP32 tag and autonomously drive towards it using triangulation and Aruco marker detection. The car is equipped with a camera for visual recognition and two ESP32 tags for UWB communication. The system is designed to be robust, allowing the car to follow the tag even in complex environments with non line-of-sight.

## Goals
### Must Have
- Accurate position measurement using triangulation with UWB.
- IMU sensor for orientation data.
- Aruco marker detection for visual identification and driving.
- Autonomous driving towards the detected tag.

### Nice to Have
- Integrate both ESP32 and Aruco marker detection for robust following.
- Real-time following of the tag.
- LiDAR integration for obstacle avoidance.

## Accomplishments
- Successfully implemented UWB triangulation for accurate distance measurement.

https://github.com/user-attachments/assets/99eae474-88a1-4db3-a036-837bd43ad0e0

In this video, we are plotting the location of the ESP32 tag predicted in real-time using the UWB triangulation. The car is placed on a desk and the tag is continuously moved around the environment and the location of the tag is plotted in real-time from the jetson connected to a monitor. In the plot, you can see two red dots representing the tags on the car and the blue dot representing the predicted location of the tag. The car is able to accurately predict the location of the tag even when it is moved around.

- IMU sensor integration for orientation data to correct for yaw of the car.

https://github.com/user-attachments/assets/ff7656d6-d6ba-4912-a2eb-ccfd36a7f4ab

In this video, we are demonstrating the robustness of location prediction using sensor fusion with IMU and UWB triangulation. The car is placed on a stand and the tag is kept stationary. As the car is rotated around, the predicted location of the tag remains stable, demonstrating the robustness of measured angular deviation using the IMU sensor. This is crucial for accurate navigation as we need to calculate the angle that the car has rotated to accurately predict the location of the tag. 

- Car can autonomously drive to a UWB-enabled ESP32 tag even in non line-of-sight condition.

https://github.com/user-attachments/assets/731a74e7-a74d-4a91-b878-ca56320b25f8

In this video, the car autonomously detects, generates waypoints and drives towards the ESP32 tag kept on the ground. This is a line-of-sight condition where there are no obstacles in the way between the car and the tag, and the car can safely drive towards the tag. 

https://github.com/user-attachments/assets/dc13d5db-3512-4963-a193-890831c27531

In this video also the car detects the tag and drives towards it. However, this is a non line-of-sight condition where there is an obstacle in between the car and the tag, and the car generates curved waypoints around the obstacle to reach the tag.

- Aruco marker detection implemented for visual identification.

https://github.com/user-attachments/assets/7d87b73c-9c4e-48f1-9f4e-f30a59d7324a

In this video, the car detects an Aruco marker, reads the ID of the marker to confirm weather its a valid user to track, determines the location and orientation of the marker and correspoindingly drive towards the marker. The steering and rotation of the car is controlled in real-time to ensure that the car is always facing the marker and is at-least 30cm away from the marker. Also the car is able to follow the marker even when it is moving around. 

## Challenges Faced
- We were limited by low range (~15m) and sometimes we wouldn't get position data due to small antenna of the DW3000 chip. 
- Even though our individual parts worked well, our location estimation varied a lot due to added uncertanities from all of them.
- We faced issues in integrating the transition from driving with ESP32 localization and driving with Aruco marker in the donkeycar framework.
- When following the Aruco marker, ROS2 was very laggy, we faced this issue in our lane-following assignment also. Might need to use an updated Jetson or larger SD card.

## Code Documentation
### Step-by-Step Running Instructions

### Driving with ESP32
The directory `driving-with-esp32` contains all the code needed to drive the car. This part of the project is divided into three parts:
1. **ESP32 Distancing**: 
  - Before we can get to estimating coordinates of the tag, we first need to measure distance between two tags. 
  - Below image shows the parameters and timing measurements we need to take care of while measuring distance:
    <div align="center">
      <img src="media/images/ESP32/esp32-distancing.png" width="500" alt="ESP32 Distance Measurement">
    </div>
  - In the above image, the anchor sends a packet to the tag, which then sends a response back to the anchor. 
  - The anchor measures the time taken for the round trip and calculates the distance using the speed of light.

2. **Communication between ESP32**: 
  - For determining position of the tag, we need two tags on the car which calls for developing a communication protocol between the two ESP32 tags. 
  - The protocol is summarized in the below image:
    <div align="center">
      <img src="media/images/ESP32/esp32-comm-protocol.png" width="500" alt="ESP32 Communication Protocol">
    </div>
    This can be summarized as follows:
  - First the tag T1 sends a request packet for distancing to the anchor P.
  - The anchor P then sends a response packet to T1. T1 measures the time taken for the response and calculates the distance to the anchor.
  - T1 sends a clearance packet to T2, which is the second tag on the car.
  - T2 then sends a request packet to the anchor P.
  - The anchor P then sends a response packet to T2. T2 measures the time taken for the response and calculates the distance to the anchor.
  - T2 then sends a clearance packet to T1, which is the first tag on the car.
  - The code for Part 1 and 2 are written through arduino and can be found in the directory: `driving-with-esp32/esp32_code`

3. **ESP32 Triangulation**: 
  - We mounted two ESP32 tags on the car which will measure the distance to a thrid ESP32 anchor placed in the environment. 
  - The image below shows the triangulation algorithm used to determine the coordinates of the tag:
    <div align="center">
      <img src="media/images/ESP32/esp32-localization.png" width="500" alt="Triangulation Algorithm">
    </div>
  - Using this formula, we can calculate the coordinates of the tag using the distances measured by the two ESP32 tags on the car. 
  - The code for this part is in `driving-with-esp32/location_tag.py`.

4. **ESP32-IMU Fusion**: 
  - As the orientation of the car changes when it drives towards the tag, the reference coordinate for distance and angle measurements also changes. 
  - This effect can be seen in the below image, where the left diagram is the case when the car does not rotate and the right diagram is the case when the car rotates:
    <div align="center">
      <img src="media/images/ESP32/esp32-imu-fusion.png" width="500" alt="ESP32 IMU Fusion">
    </div>
  - To cancel out this effect, we used an IMU sensor to compute the yaw of the car and correct the coordinates of the tag accordingly.
  - The code for this part is written through arduino and can be found in the directory: `driving-with-esp32/IMUcode`.

5. **Driving with ESP32**: 
  - Once we have the accurate coordinates of the tag, we used the GPS Donkeycar framework to drive the car towards the tag.
  - First we generate the waypoints for the car to the tag, and then pass this to the GPS Donkeycar framework to drive the car.
  - The code for this part is present in: `driving-with-esp32/manage.py`.
  - Further, we replaced the positions dumped by the GPS with the positions predicted by the ESP32 triangulation algorithm. This part of the code is present in `driving-with-esp32/vehicle.py`. Please note that this code should be put in the `donkeycar` directory of the donkeycar framework.

### Driving with Aruco Marker
> **TODO**: Jason please add the code explanation here.

## Parts and Hardware

| Part | Preview <img width="5"/> |  File |
|------|--------------------------|----------|
| **Mounting Board** | <img src="media/images/3D-Printed/board.png" width="300" alt="Mounting-board render"> | [DXF](media/stl-files/Electronics%20Mount%20Plate%20v1.dxf) |
| **NVIDIA Jetson Nano Case** | <img src="media/images/3D-Printed/jetsoncase.webp" width="300" alt="Jetson Nano case"> | [Bottom STL](media/stl-files/NanoBox_Bottom_v21.stl) • [Top STL](media/stl-files/NanoBox_Top_v22.stl) |
| **GPS Mount** | <img src="media/images/3D-Printed/gpsmount1.png" width="300" alt="GPS mount"> | [STL](media/stl-files/gps_mount.stl) |
| **Oak-D Lite Camera Mount** | <img src="media/images/3D-Printed/cameram2.png" width="300" alt="Camera mount view 1"><br><img src="media/images/3D-Printed/cameram3.png" width="300" alt="Camera mount view 2"> | [Mount STL](media/stl-files/camera_mount.stl) • [Adapter 3MF](media/stl-files/OAK-DLiteAdapter.3mf) |
| **ESP32 Mount** | <img src="media/images/3D-Printed/ae1.png" width="300" alt="ESP32 arm"> | |
| **ESP32 Mount – Arm** | <img src="media/images/3D-Printed/arm2.png" width="300" alt="ESP32 arm"> | [STL](media/stl-files/arm.stl) |
| **ESP32 Mount – Extension** | <img src="media/images/3D-Printed/e1.png" width="300" alt="ESP32 extension"> | [STL](media/stl-files/extension.stl) |
| **ESP32 Case** | <img src="media/images/3D-Printed/esp2.png" width="300" alt="ESP32 case"> | [STL](media/stl-files/esp_case_2.stl) |
| **Power-Bank & ESP32 Case** | <img src="media/images/3D-Printed/pbc1.png" width="300" alt="Power-bank case"> | [Bottom STL](media/stl-files/power_bank_case_1.stl) • [Top STL](media/stl-files/power_bank_case_2.stl) |
| **ESP32 with UWB** | <img src="media/images/Parts/esp32.jpg" width="300" alt="ESP-32"> | [Product Link](https://www.makerfabs.com/esp32-uwb-dw3000.html?srsltid=AfmBOorf-qO5e5mbXH0PJVKdmqxx--euOzTTMy_9mlpG27S6Im0cnn02) |
| **OAK-D Camera** | <img src="media/images/Parts/oakd.webp" width="300" alt="ESP-32"> | [Product Link](https://shop.luxonis.com/products/oak-d-lite-1?variant=42583102456031) |
| **IMU** | <img src="media/images/Parts/imu.webp" width="300" alt="ESP-32"> | [Product Link](https://wiki.seeedstudio.com/XIAO_BLE/) |

### Wiring
<div align="center">
  <img src="media/images/mae148wiring.png" width="500" alt="Wiring">
</div>

## Gantt Chart
<div align="center">
  <img src="media/images/gantt-chart.png" width="500" alt="Gantt Chart">
</div>

## Acknowledgements
Special thanks to Professor Jack Silberman and our TAs, Alex and Winston for the continous support and guidance throughout the quarter.

## Contacts
- Pushkal Mishra (pumishra@ucsd.edu)
- Efe Erturk (eerturk@ucsd.edu)
- Jason Junran (juw070@ucsd.edu)
- Etka Uzun (muzun@ucsd.edu)
