import time
import math
import serial
import argparse
import threading
from collections import deque
import matplotlib.pyplot as plt

def triangle_angles(a, b, c):
    # Check for triangle validity
    if a + b <= c or a + c <= b or b + c <= a:
        raise ValueError("Invalid triangle sides")

    # Law of Cosines
    angle_A = math.acos((b**2 + c**2 - a**2) / (2 * b * c))
    angle_B = math.acos((a**2 + c**2 - b**2) / (2 * a * c))
    angle_C = math.acos((a**2 + b**2 - c**2) / (2 * a * b))

    return angle_A, angle_B, angle_C

def triangulation(d1, d2, d = 0.33):
    """
    Triangulate the position of point A given the distances to points B and C.

    Parameters
    ----------
    d1 : float
        Distance from point A to point B.
    d2 : float
        Distance from point A to point C.

    Returns
    -------
    tuple
        Coordinates of point A (x, y).
    """
    a, b, c = d, d2, d1
    angle_A, angle_B, angle_C = triangle_angles(a, b, c)
    alpha = math.atan2(2 * b * math.sin(angle_A) - a * math.sin(angle_B), 2 * b * math.cos(angle_A) + a * math.cos(angle_B))
    
    theta = angle_B + alpha
    r = b * math.sin(angle_C) / math.sin(theta)
    
    x = r * math.cos(theta)
    y = r * math.sin(theta)
    
    return x, y

# Function to read and print data from a serial port
def read_serial(port_name, baudrate=115200, distance_buffers = None):
    try:
        with serial.Serial(port_name, baudrate=baudrate, timeout=1) as ser:
            print(f"[INFO] Listening on {port_name}...")
            while True:
                if ser.in_waiting:
                    try:
                        line = ser.readline().decode('utf-8', errors='replace').strip()
                        val = float(line)
                        distance_buffers[port_name].append(val)
                    except ValueError:
                        print(f"[WARN] Invalid data on {port_name}: {line}")
    except serial.SerialException as e:
        print(f"[ERROR] Could not open {port_name}: {e}")
    
def main(args):
    distance_buffers = {
        args.port1: deque(maxlen = args.queue_size),
        args.port2: deque(maxlen = args.queue_size)
    }

    threading.Thread(
        target = read_serial, 
        args = (args.port1, args.baud_rate, distance_buffers), 
        daemon=True
    ).start()

    threading.Thread(
        target = read_serial, 
        args = (args.port2, args.baud_rate, distance_buffers), 
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
            buf0 = distance_buffers[args.port1]
            buf1 = distance_buffers[args.port2]

            if len(buf0) == args.queue_size and len(buf1) == args.queue_size:
                d1 = sum(buf0) / args.queue_size
                d2 = sum(buf1) / args.queue_size

                try:
                    tag_x, tag_y = triangulation(d1, d2)

                    # Update tag position
                    tag_plot.set_data([tag_x], [tag_y])
                    line1.set_data([ANCHOR_LEFT[0], tag_x], [ANCHOR_LEFT[1], tag_y])
                    line2.set_data([ANCHOR_RIGHT[0], tag_x], [ANCHOR_RIGHT[1], tag_y])
                    ax.set_title(f"Tag Location: X = {tag_x:.2f} cm, Y = {tag_y:.2f} cm")
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
        "--port1",
        type = str,
        default = "/dev/ttyUSB0",
        help = "Serial port for anchor 1"
    )
    parser.add_argument(
        "--port2",
        type = str,
        default = "/dev/ttyUSB1",
        help = "Serial port for anchor 2"
    )
    parser.add_argument(
        "--queue_size",
        type = int,
        default = 1,
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
        default = 10.0,
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