import time
import math
import serial
import argparse
import threading
import numpy as np
from collections import deque
import matplotlib.pyplot as plt

def triangle_angles(a, b, c):
    # Check for triangle validity
    if a + b <= c or a + c <= b or b + c <= a:
        # raise ValueError(f"Invalid triangle sides: a = {a}, b = {b}, c = {c}")
        return 0.0, 0.0, 0.0
    
    # Law of Cosines
    angle_A = math.acos((b**2 + c**2 - a**2) / (2 * b * c))
    angle_B = math.acos((a**2 + c**2 - b**2) / (2 * a * c))
    angle_C = math.acos((a**2 + b**2 - c**2) / (2 * a * b))

    return angle_A, angle_B, angle_C

def triangulation(d1, d2, d = 0.33, anchor_angle = 0.0):
    """
    Triangulate the position of point A given the distances to points B and C.

    Parameters
    ----------
    d1 : float
        Distance from point A to point B in meters.
    d2 : float
        Distance from point A to point C in meters.
    d   : float
        Distance between anchors in meters.
    anchor_angle : float
        Deviated angle of the anchors in degrees.
    
    Returns
    -------
    tuple
        Coordinates of point A (x, y).
    """
    a, b, c = d, d2, d1
    angle_A, angle_B, angle_C = triangle_angles(a, b, c)
    # anchor_angle = 0.0
    print(d1, d2, anchor_angle)
    
    if angle_A == 0.0 and angle_B == 0.0 and angle_C == 0.0:
        return -100.0, -100.0
    
    anchor_angle_radian = math.radians(anchor_angle)
    # anchor_angle_radian = 0.0
    alpha = math.atan2(2 * b * math.sin(angle_A) - a * math.sin(angle_B), 2 * b * math.cos(angle_A) + a * math.cos(angle_B))
    
    theta = angle_B + alpha + anchor_angle_radian
    r = b * math.sin(angle_C) / math.sin(theta)
    
    x = r * math.cos(theta)
    y = r * math.sin(theta)
    
    return x, y

# Function to read and print data from a serial port
def read_serial_esp32(port_name, baudrate=115200, read_buffers = None):
    try:
        with serial.Serial(port_name, baudrate=baudrate, timeout=1) as ser:
            print(f"[INFO] ESP32 connected on {port_name}...")
            while True:
                if ser.in_waiting:
                    try:
                        line = ser.readline().decode('utf-8', errors='replace').strip()
                        val = float(line)
                        read_buffers[port_name].append(val)
                    except ValueError:
                        print(f"[WARN] Invalid data on {port_name}: {line}")
    except serial.SerialException as e:
        print(f"[ERROR] Could not open {port_name}: {e}")
    
def read_serial_imu(
    port_name, 
    baudrate=115200, 
    read_buffers = None
):
    reset_message = 'RESET\n'
    try:
        with serial.Serial(port_name, baudrate=baudrate, timeout=1) as ser:
            print(f"[INFO] IMU connected on {port_name}...")
            while True:
                if ser.in_waiting:
                    try:
                        if read_buffers["reset_imu"]:
                            ser.write(reset_message.encode('utf-8'))                            
                            read_buffers["reset_imu"] = False
                        else:
                            line = ser.readline().decode('utf-8', errors='replace').strip()
                            read_buffers[port_name] = float(line)
                    except ValueError:
                        print(f"[WARN] Invalid data on {port_name}: {line}")
    except serial.SerialException as e:
        print(f"[ERROR] Could not open {port_name}: {e}")

class ESP32_Tag_Loc:
    """
    Class to read the location of the tag from the ESP32s and the IMU.
    The ESP32s and IMU are read in a threaded manner.
    The location is calculated using the triangulation algorithm.
    The location is returned as a tuple of (x, y).
    """
    def __init__(self, cfg):
        self.read_buffers = {
            cfg.ESP32_PORT1: deque(maxlen = cfg.QUEUE_SIZE),
            cfg.ESP32_PORT2: deque(maxlen = cfg.QUEUE_SIZE),
            cfg.IMU_PORT: 0.0,
            "reset_imu": True
        }
        self.cfg = cfg
        self.ANCHOR_DISTANCE = cfg.ANCHOR_DISTANCE
        self.ANCHOR_LEFT = (-self.ANCHOR_DISTANCE / 2, 0)
        self.ANCHOR_RIGHT = (self.ANCHOR_DISTANCE / 2, 0)
        self.thread_esp32_port1 = None
        self.thread_esp32_port2 = None
        self.thread_imu = None
        self.tag_x_prev = 0.0
        self.tag_y_prev = 0.0
        self.distance_threshold = cfg.DISTANCE_THRESHOLD
    
    def start(self):
        self.thread_esp32_port1 = threading.Thread(
            target = read_serial_esp32, 
            args = (self.cfg.ESP32_PORT1, self.cfg.BAUD_RATE, self.read_buffers), 
            daemon=True
        )
        self.thread_esp32_port1.start()

        self.thread_esp32_port2 = threading.Thread(
            target = read_serial_esp32, 
            args = (self.cfg.ESP32_PORT2, self.cfg.BAUD_RATE, self.read_buffers), 
            daemon=True
        )
        self.thread_esp32_port2.start()

        self.thread_imu = threading.Thread(
            target = read_serial_imu,
            args = (self.cfg.IMU_PORT, self.cfg.BAUD_RATE, self.read_buffers),
            daemon=True
        )
        self.thread_imu.start()
    
    def get_location(self):
        while True:
            buf0 = self.read_buffers[self.cfg.ESP32_PORT1]
            buf1 = self.read_buffers[self.cfg.ESP32_PORT2]
            imu_angle = self.read_buffers[self.cfg.IMU_PORT]
            if len(buf0) == self.cfg.QUEUE_SIZE and len(buf1) == self.cfg.QUEUE_SIZE:
                d1 = sum(buf0) / self.cfg.QUEUE_SIZE
                d2 = sum(buf1) / self.cfg.QUEUE_SIZE
                tag_x, tag_y = triangulation(
                    d1 = d1, 
                    d2 = d2, 
                    d = self.ANCHOR_DISTANCE,
                    anchor_angle = imu_angle
                )
                if tag_x == -100.0 and tag_y == -100.0:
                    tag_x = self.tag_x_prev
                    tag_y = self.tag_y_prev
                elif np.sqrt(tag_x ** 2 + tag_y ** 2) > self.distance_threshold:
                    tag_x = self.tag_x_prev
                    tag_y = self.tag_y_prev
                else:
                    self.tag_x_prev = tag_x
                    self.tag_y_prev = tag_y
                
                return tag_x, tag_y
            else:
                time.sleep(self.cfg.REFRESH_RATE)

    def reset_origin(self):
        # reset the previous tag location and IMU angle
        self.tag_x_prev = 0.0
        self.tag_y_prev = 0.0
        self.read_buffers[self.cfg.IMU_PORT] = 0.0
        
        # Use the thread_imu to send the reset message
        self.read_buffers["reset_imu"] = True
    
    def stop(self):
        # Stop the threads
        if self.thread_esp32_port1:
            self.thread_esp32_port1.join()
        if self.thread_esp32_port2:
            self.thread_esp32_port2.join()
        if self.thread_imu:
            self.thread_imu.join()



def main(args):
    read_buffers = {
        args.esp32_port1: deque(maxlen = args.queue_size),
        args.esp32_port2: deque(maxlen = args.queue_size),
        args.imu_port: 0.0,
        "reset_imu": True,
        "at_start_flag": True
    }

    threading.Thread(
        target = read_serial_esp32, 
        args = (args.esp32_port1, args.baud_rate, read_buffers), 
        daemon=True
    ).start()

    threading.Thread(
        target = read_serial_esp32, 
        args = (args.esp32_port2, args.baud_rate, read_buffers), 
        daemon=True
    ).start()
    
    threading.Thread(
        target = read_serial_imu,
        args = (args.imu_port, args.baud_rate, read_buffers),
        daemon=True
    ).start()
    
    ANCHOR_DISTANCE = args.anchor_distance
    ANCHOR_LEFT = (-ANCHOR_DISTANCE / 2, 0)
    ANCHOR_RIGHT = (ANCHOR_DISTANCE / 2, 0)
    
    plt.ion()
    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    ax.set_xlim(-args.plot_limit_factor * ANCHOR_DISTANCE, args.plot_limit_factor * ANCHOR_DISTANCE)
    ax.set_ylim(-0.1, args.plot_limit_factor * ANCHOR_DISTANCE)
    ax.scatter([ANCHOR_LEFT[0], ANCHOR_RIGHT[0]], [ANCHOR_LEFT[1], ANCHOR_RIGHT[1]], color='red', s=100, label='Anchors')

    tag_plot, = ax.plot([], [], 'bo', label='Tag')  # Tag as blue dot
    line1, = ax.plot([], [], 'k--')  # Line to anchor 1
    line2, = ax.plot([], [], 'k--')  # Line to anchor 2
    ax.legend()
    
    try:
        while True:
            buf0 = read_buffers[args.esp32_port1]
            buf1 = read_buffers[args.esp32_port2]

            if len(buf0) == args.queue_size and len(buf1) == args.queue_size:
                d1 = sum(buf0) / args.queue_size
                d2 = sum(buf1) / args.queue_size

                try:
                    tag_x, tag_y = triangulation(
                        d1 = d1, 
                        d2 = d2, 
                        anchor_angle = read_buffers[args.imu_port]
                    )

                    # Update tag position
                    tag_plot.set_data([tag_x], [tag_y])
                    line1.set_data([ANCHOR_LEFT[0], tag_x], [ANCHOR_LEFT[1], tag_y])
                    line2.set_data([ANCHOR_RIGHT[0], tag_x], [ANCHOR_RIGHT[1], tag_y])
                    ax.set_title(f"Tag Location: X = {tag_x:.2f} m, Y = {tag_y:.2f} m\nIMU Angle = {read_buffers[args.imu_port]:.2f} deg")
                    fig.canvas.draw()
                    plt.pause(0.001)

    
                except ValueError as e:
                    print(f"[WARN] Invalid triangle: {e}")

            time.sleep(args.refresh_rate)
    except KeyboardInterrupt:
        print("\n[INFO] Exiting...")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--esp32_port1",
        type = str,
        default = "/dev/ttyUSB0",
        help = "Serial port for anchor 1"
    )
    parser.add_argument(
        "--esp32_port2",
        type = str,
        default = "/dev/ttyUSB1",
        help = "Serial port for anchor 2"
    )
    parser.add_argument(
        "--imu_port",
        type = str,
        default = "/dev/ttyACM0",
        help = "Serial port for IMU"
    )
    parser.add_argument(
        "--queue_size",
        type = int,
        default = 5,
        help = "Queue size for distance readings"
    )
    parser.add_argument(
        "--anchor_distance",
        type = float,
        default = 0.33,
        help = "Distance between anchors"
    )
    parser.add_argument(
        "--plot_limit_factor",
        type = float,
        default = 15.0,
        help = "Plot limit factor"
    )
    parser.add_argument(
        "--refresh_rate",
        type = float,
        default = 0.01,
        help = "Refresh rate for plot"
    )
    parser.add_argument(
        "--baud_rate",
        type = int,
        default = 115200,
        help = "Baud rate for serial communication"
    )
    args = parser.parse_args()

    main(args)
