#!/usr/bin/env python3

import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import time
import cv2

# Ensure we can import Quanser libraries
sys.path.insert(0, '/home/nvidia/Documents/Quanser/0_libraries/python')
try:
    from pal.products.qcar import QCarCameras
    QUANSER_LIB_FOUND = True
except ImportError:
    QUANSER_LIB_FOUND = False

class SensorVerifier(Node):
    def __init__(self):
        super().__init__('sensor_verifier')
        self.lidar_found = False
        self.odom_found = False
        
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.get_logger().info('--- QCar 2 Sensor Verification ---')
        self.get_logger().info('Checking for /scan and /odom topics...')

    def lidar_callback(self, msg):
        self.lidar_found = True

    def odom_callback(self, msg):
        self.odom_found = True

def main():
    rclpy.init()
    node = SensorVerifier()
    
    # 1. Check ROS topics (Lidar/Odom)
    start_time = time.time()
    while time.time() - start_time < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.lidar_found and node.odom_found:
            break
            
    print(f"[Topic] /scan: {'✅ FOUND' if node.lidar_found else '❌ NOT FOUND'}")
    print(f"[Topic] /odom: {'✅ FOUND' if node.odom_found else '❌ NOT FOUND'}")

    # 2. Check CSI Cameras with "Working" Parameters (820x616 @ 80fps)
    print("\n[CSI] Initializing cameras with verified parameters (820x616 @ 80fps)...")
    if not QUANSER_LIB_FOUND:
        print("❌ Quanser libraries not found at /home/nvidia/Documents/Quanser/0_libraries/python")
    else:
        # Force headless/fixing EGL authorization as per user script
        os.environ.pop("DISPLAY", None)
        os.environ.pop("XAUTHORITY", None)
        
        cameras = None
        try:
            cameras = QCarCameras(
                frameWidth=820,
                frameHeight=616,
                frameRate=80,
                enableRight=True,
                enableBack=True,
                enableFront=True,
                enableLeft=True
            )
            print("[CSI] Cameras initialized ✅")
            
            # Read a few frames to verify
            for _ in range(10):
                cameras.readAll()
                time.sleep(0.1)
            
            names = {0: 'Right', 1: 'Rear', 2: 'Front', 3: 'Left'}
            for i, name in names.items():
                img = cameras.csi[i].imageData
                max_val = img.max() if img is not None else 0
                status = "✅ WORKING" if max_val > 10 else "❌ BLANK"
                print(f"  Camera {i} ({name:5s}): {status} (max_px={max_val})")
            
        except Exception as e:
            print(f"[CSI ERROR] {e}")
        finally:
            if cameras is not None:
                cameras.terminate()
                print("[CSI] Terminated safely.")

    print("\n--- Summary ---")
    if node.lidar_found and node.odom_found:
        print("🚀 ESSENTIALS READY: You can start SLAM and Nav2 now!")
    else:
        print("⚠️ ACTION REQUIRED: Ensure your QCar 2 drivers are running to provide Lidar and Odom.")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
