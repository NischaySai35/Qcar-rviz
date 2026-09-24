#!/usr/bin/env python3

import os
import sys
import rclpy
from rclpy.node import Node
import time
import cv2
import numpy as np

# Ensure we can import Quanser libraries
sys.path.insert(0, '/home/nvidia/Documents/Quanser/0_libraries/python')
try:
    from pal.products.qcar import QCarCameras
    QUANSER_LIB_FOUND = True
except ImportError:
    QUANSER_LIB_FOUND = False

class CSIHub(Node):
    def __init__(self):
        super().__init__('csi_hub')
        
        # Parameters for your specific car as verified
        self.width = 820
        self.height = 616
        self.fps = 80
        
        # Display size (scaled down for the dashboard)
        self.display_size = (410, 308) 
        
        if not QUANSER_LIB_FOUND:
            self.get_logger().error('Quanser libraries not found! Cannot start CSI Hub.')
            return

        # Fix EGL issues as per user script
        os.environ.pop("DISPLAY", None)
        os.environ.pop("XAUTHORITY", None)

        self.get_logger().info(f'Initializing 4 CSI cameras @ {self.width}x{self.height} ({self.fps}fps)...')
        
        try:
            self.cameras = QCarCameras(
                frameWidth=self.width,
                frameHeight=self.height,
                frameRate=self.fps,
                enableRight=True,
                enableBack=True,
                enableFront=True,
                enableLeft=True
            )
            self.get_logger().info('CSI cameras initialized successfully ✅')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize cameras: {e}')
            self.cameras = None

        self.timer = self.create_timer(0.05, self.timer_callback) # ~20fps display rate

    def timer_callback(self):
        if self.cameras is None:
            return

        try:
            self.cameras.readAll()
            
            # Stitch 4 cameras into a 2x2 grid
            # Right=0, Rear=1, Front=2, Left=3
            frames = []
            names = ['Right', 'Rear', 'Front', 'Left']
            
            for i in range(4):
                img = self.cameras.csi[i].imageData
                if img is None:
                    img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
                
                # Resize for dashboard
                img_small = cv2.resize(img, self.display_size)
                
                # Add label
                cv2.putText(img_small, names[i], (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                frames.append(img_small)
            
            # Combine into 2x2
            top = np.hstack([frames[2], frames[0]]) # Front, Right
            bottom = np.hstack([frames[3], frames[1]]) # Left, Rear
            dashboard = np.vstack([top, bottom])
            
            cv2.imshow('QCar 2 CSI Cluster', dashboard)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().warn(f'Capture error: {e}')

    def __del__(self):
        if hasattr(self, 'cameras') and self.cameras is not None:
            self.cameras.terminate()

def main(args=None):
    rclpy.init(args=args)
    node = CSIHub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
