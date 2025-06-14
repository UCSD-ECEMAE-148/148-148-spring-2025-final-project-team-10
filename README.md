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
  - [Step-by-Step Running Instructions](#step-by-step-running-instructions)
    - [Hardware Setup](#hardware-setup)
    - [Software Setup](#software-setup)
    - [Plotting Live Location from ESP32](#plotting-live-location-from-esp32)
    - [Driving the Car with ESP32](#driving-the-car-with-esp32)
    - [Driving the Car with Aruco Marker](#driving-the-car-with-aruco-marker)
  - [Code Documentation](#code-documentation)
    - [Driving with ESP32](#driving-with-esp32)
    - [Driving with Aruco Marker](#driving-with-aruco-marker)
  - [Hardware Used](#hardware-used)
    - [Parts](#parts)
    - [Wiring](#wiring)
  - [Gantt Chart](#gantt-chart)
  - [Acknowledgements](#acknowledgements)
  - [Contacts](#contacts)

  </details>


## Team Members
- **Efe Erturk** | ECE: BS in Electrical Engineering 
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

## Step-by-Step Running Instructions
### Hardware Setup
- Use the provided 3D-printed parts to mount the NVIDIA Jetson Nano, OAK-D Lite camera, ESP32 tags, and IMU sensor on the car.
- Connect the two ESP32 tags, OAK-D Lite Camera, and IMU sensor to the Jetson using cables.
- Ensure that the ESP32 tags are securely mounted and connected to Jetson.
- Connect the dongle for the controller to the Jetson Nano.

### Software Setup:
- First you need to install Arduino CLI, DW3000 library and IMU on the jetson nano. This can be found in the links: [Arduino CLI](https://arduino.github.io/arduino-cli/0.35/installation/), [DW3000 Library](https://github.com/Makerfabs/Makerfabs-ESP32-UWB-DW3000) and [IMU Library](https://wiki.seeedstudio.com/XIAO_BLE/)
- Clone this repository to your local machine:
  ```bash
  git clone https://github.com/UCSD-ECEMAE-148/148-spring-2025-final-project-team-10.git
  ```
- To write the localization code to the ESP32, you can open each of the `.ino` code in `driving-with-esp32/esp32_code` directory using Arduino IDE and upload it to the ESP32 tags. To use the more reliable localization, follow the instructions [here](https://github.com/kk9six/dw3000) to upload the code present in `driving-with-esp32/esp32_code_v2` directory.
- To write the code to IMU, you can simly run the following command in Jetson's terminal:
  ```bash
  cd ./driving-with-esp32/IMUcode/
  bash compile_imu.sh
  ```
- Since we are replacing the GPS coordinates with ESP32 coordinates, we need to replace the `vehicle.py` file in the donkeycar framework with the one present in `driving-with-esp32/vehicle.py`. Use the following command to do so:
  ```bash
  cp ./driving-with-esp32/vehicle.py /home/jetson/projects/donkeycar/donkeycar/vehicle.py
  ```
### Plotting Live Location from ESP32:
- As seen in the video above, we can plot the live location of the ESP32 tag using the code present in `driving-with-esp32/location_tag.py`. 
- To run this code, use the following command:
  ```bash
  cd ./driving-with-esp32/
  python3 location_tag.py
  ```
- Make sure that the thrid ESP32 anchor is connected to a power bank.
### Driving the Car with ESP32:
- To drive the car using ESP32 triangulation, you can simply run the GPS code. Run the following command in the Jetson Terminal:
  ```bash
  cd ./driving-with-esp32/
  python3 manage.py drive
  ```
- Steps to run the car:
  - First, press the "B" button to reset the IMU readings.
  - Then, press "Y" to generate the waypoints to the ESP32 tag.
  - Then, press "X" to load the path.
  - Finally press the "Start" button to start driving the car towards the ESP32 tag.
- The above key mappings can be found in `driving-with-esp32/myconfig.py`.

### Driving the Car with Aruco Marker:
- Follow the [ECE/MAE 148 document](https://docs.google.com/document/d/1Onft0sIWhEd9UH7fItJ0atC1hKnxpTxKKmUFHtqC-sA/edit?tab=t.0) (Section 4) to pull the Robocar Docker Image and set it up with ROS2

- Now create and launch Docker container:
```bash
robocar_docker test_container
docker start test_container
docker exec -it test_container bash
```
- Create ROS2 Package for Aruco
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python aruco_marker --dependencies rclpy sensor_msgs geometry_msgs std_msgs
```
- Then clone your Python node scripts:
```bash
cd aruco_marker
mkdir aruco_marker
cd aruco_marker

git clone https://github.com/UCSD-ECEMAE-148/148-spring-2025-final-project-team-10/blob/main/aruco_marker/aruco_marker/oakd_aruco_node.py oakd_aruco_node.py
git clone https://github.com/UCSD-ECEMAE-148/148-spring-2025-final-project-team-10/blob/main/aruco_marker/aruco_marker/vesc_twist_node.py esc_twist_node.py
git clone https://github.com/UCSD-ECEMAE-148/148-spring-2025-final-project-team-10/tree/main/aruco_marker/aruco_marker/vesc_submodule vesc_submodule
```
- Make sure to copy the setup and launch files from the repository to the correct directories:
  - The setup.py file is located here: [https://github.com/UCSD-ECEMAE-148/148-spring-2025-final-project-team-10/blob/main/aruco_marker/setup.py](https://github.com/UCSD-ECEMAE-148/148-spring-2025-final-project-team-10/blob/main/aruco_marker/setup.py)

  - The Launch File for both nodes are present here: [https://github.com/UCSD-ECEMAE-148/148-spring-2025-final-project-team-10/blob/main/aruco_marker/launch/duo_nodes.launch.py](https://github.com/UCSD-ECEMAE-148/148-spring-2025-final-project-team-10/blob/main/aruco_marker/launch/duo_nodes.launch.py)

- Run the following commands to build, source, and launch (from inside docker):
```bash
source_ros2
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch aruco_marker duo_node_.launch.py
```
- You can also use the bash script to compile and run the ROS2 nodes for faster prototyping located [here](https://github.com/UCSD-ECEMAE-148/148-spring-2025-final-project-team-10/blob/main/aruco_marker/run_ros2.sh):
```bash
bash run_ros2.sh
```

## Code Documentation
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
  - This can be summarized as follows:
    - First the tag T1 sends a request packet for distancing to the anchor P.
    - The anchor P then sends a response packet to T1. T1 measures the time taken for the response and calculates the distance to the anchor.
    - T1 sends a clearance packet to T2, which is the second tag on the car.
    - T2 then sends a request packet to the anchor P.
    - The anchor P then sends a response packet to T2. T2 measures the time taken for the response and calculates the distance to the anchor.
    - T2 then sends a clearance packet to T1, which is the first tag on the car.
  - The code for Part 1 and 2 are written through arduino and can be found in the directory: `driving-with-esp32/esp32_code_v2`.
  - We were facing a lot of issues with the precision of the distance measurements, so we used the code from this repository: [https://github.com/kk9six/dw3000](https://github.com/kk9six/dw3000). We made a few modifications for our setup, which can be found in `driving-with-esp32/`.

3. **ESP32 Triangulation**: 
  - We mounted two ESP32 tags on the car which will measure the distance to a thrid ESP32 anchor placed in the environment. 
  - The image below shows the triangulation algorithm used to determine the coordinates of the tag:
    <div align="center">
      <img src="media/images/ESP32/esp32-localization.png" width="500" alt="Triangulation Algorithm">
    </div>
  - Using this formula, we can calculate the coordinates of the tag using the distances measured by the two ESP32 tags on the car. 
  - The code for this part is in `driving-with-esp32/location_tag.py`.
    
4. **IMU Integration**:
  - We are using nRF52840 Sense IMU with 6 DOF, which supports acceleration and gyroscope data in x, y, z axes.
  - This figure shows the change in angular acceleration of as the IMU is rotated:
    <div align="center">
      <img src="media/images/IMU_figure2.png" width="500" alt="IMU Integation">
    </div>
  - The Z-axis gyroscope data (gz) is used to estimate the yaw angle (rotation around the vertical axis) by integrating angular velocity over time.
  - The yaw is normalized to a 0–360° range and transmitted to the jetson via serial monitor every 50 ms.
    
5. **ESP32-IMU Fusion**: 
  - As the orientation of the car changes when it drives towards the tag, the reference coordinate for distance and angle measurements also changes. 
  - This effect can be seen in the below image, where the left diagram is the case when the car does not rotate and the right diagram is the case when the car rotates:
    <div align="center">
      <img src="media/images/ESP32/esp32-imu-fusion.png" width="500" alt="ESP32 IMU Fusion">
    </div>
  - To cancel out this effect, we used an IMU sensor to compute the yaw of the car and correct the coordinates of the tag accordingly.
  - The code for this part is written through arduino and can be found in the directory: `driving-with-esp32/IMUcode`.

6. **Waypoint Generation**:
  - For the car to drive towards the tag, we need to generate waypoints for the car to follow.
  - We have generated two types of paths, one is linear and the other is a curved path. These can be seen in the below image:
    <div align="center">
      <img src="media/images/ESP32/esp32-waypoint-gen.png" width="500" alt="ESP32 Waypoints">
    </div>
  - The curved path is basically a quadratic equation in such a manner that it passes through the tag and the car's initial position. The curvature of this path can be controlled by the parameter `PATH_CURVE_FACTOR` in `driving-with-esp32/myconfig.py`.
  - A higher path curve factor will result in a lesser curvature.

7. **Driving with ESP32**: 
  - Once we have the accurate coordinates of the tag, we used the GPS Donkeycar framework to drive the car towards the tag.
  - First we generate the waypoints for the car to the tag, and then pass this to the GPS Donkeycar framework to drive the car.
  - The code for this part is present in: `driving-with-esp32/manage.py`.
  - Further, we replaced the positions dumped by the GPS with the positions predicted by the ESP32 triangulation algorithm. This part of the code is present in `driving-with-esp32/vehicle.py`. Please note that this code should be put in the `donkeycar` directory of the donkeycar framework.

### Driving with Aruco Marker
The directory `home/projects/ros2_ws/src/aruco_marker` contains all the code needed to drive the car autnomously using aruco marker. This part of the project is divided into two parts:
1. **oakd_aruco_node.py**
- This node uses an OAK-D camera to detect Aruco markers and estimate their 3D position and orientation.
- This node file allow evey aruco marker to be considered as valid marker.
- You can generate your own marker using the OpenCV Aruco library.
- Aruco marker size is known (14 cm), allowing `cv2.aruco.estimatePoseSingleMarkers()` to calculate the translation vector (tvec) and rotation vector (rvec).
- Make sure the printed marker size is near 14cm by 14cm.
  <div align="center">
    <img src="media/images/aruco_marker_23.png" width="500" alt="Aruco Marker Image">
  </div>
- From the translation vector, we extract the following parameters from the aruco marker detection:
  - z = forward distance (used to determine rpm)
  - x = horizontal offset (used to compute steering value)
- The node converts yaw angle in radians into angular.z  and forward distance into linear.x, then publishes them in a Twist Interface
- Publishes Twist messages to ```/cmd_vel``` topic so that other node files can subscribe the motot value in this topic.
- Stops publishing if the detected marker is too close.
- Publishing 0.5 steering (center) and 0 rpm if not detecting any markers, 

2. **Running the oakd_aruco_node.py**
- run it with ```bash run_ros2.sh```
- Then on the terminal there should be distance and angle message printing. Show the aruco marker in front of the oakd camera, and the value should be changing.
- The maximum range line of sight of the camera is roughly -20 to +20 degrees, but it is kind of unstable when the marker get close to 20 degree offset.

3. **Checking if the message is published to the topic**
- run ```ros2 topic echo /cmd_vel``` on a separate terminal, then you should see linear.x and angular.z being published to the topic.

4. **vesc_twist_node.py**
- This node subscribes to /cmd_vel and controls the VESC motor and steering via the vesc_submodule:
- Converts linear.x (distance) into throttle (RPM), clamped between minimum and maximum values.
- Converts angular.z (yaw angle) from radians to degrees, and remaps to a steering servo PWM value between 0.0 and 1.0.
- Applies polarity and remapping parameters (tunable via ROS parameters).
- Commands are sent to the VESC over UART.

5. **Common issue**
- If launching the node files result in "could not connect to vesc" please re-run the bash script to debug. Vesc sometimes break down magically and re-running it can fix the issue.
- Check if VESC and Camera is plugeged directly into jetson.
- Check battery voltage level, this can cause any issue.
- If you feel like running the bash script does not make a change to the behavior, check the memory storage usage on the SD card.
- If docker break down such as xeyes not responding, quit the docker, stop the docker, then exit jetson, reboot jetson, re-ssh, restart docker, try xeyes again. If this does not solve the issue, turn off jetson and give it a couple minutes then continue working later.
- If VESC behave weirdly, please use the VESC tool to re-tune the VESC.

6. **Future potential**
- PID implementation for aruco marker autonomous driving.
- Improved pre-collision braking system using LiDar by creating another node file to publish more up-to-time distance.
- Modify ```vesc_twist_node.py``` to include the pre-collison braking algorithm.


## Hardware Used
### Parts
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
| **Logitech Controller** | <img src="media/images/controller.webp" width="300" alt="ESP-32"> | [Product Link](https://www.logitechg.com/en-us/products/gamepads/f710-wireless-gamepad.940-000117.html?gclid=Cj0KCQjwz7uRBhDRARIsAFqjulkuKo41BBWfzpkCoHsG_LRd6WOvTYzetC7DRXhKwdPnabcZy49XMJ0aAhT_EALw_wcB) |


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
