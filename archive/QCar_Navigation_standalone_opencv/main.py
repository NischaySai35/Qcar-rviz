import numpy as np
import cv2
import time

try:
    from pal.products.qcar import QCar, QCarLidar
    HARDWARE_OK = True
except ImportError:
    print("Warning: Quanser QCar API not found. Running in simulation/mock mode.")
    HARDWARE_OK = False

from mapping import GlobalMap
from planner import PathPlanner
from navigator import ReactiveNavigator

# Global goal parameter updated by GUI
GOAL_NODE_WORLD = None

def mouse_callback(event, x, y, flags, param):
    global GOAL_NODE_WORLD
    if event == cv2.EVENT_LBUTTONDBLCLK:
        # Received a double click on the Map window
        # Converts pixel (x,y) to world meters
        # Param carries the map object
        my_map = param
        world_x, world_y = my_map.pixel_to_world(x, y) # Notice OpenCV x,y vs map px,py
        GOAL_NODE_WORLD = (world_x, world_y)
        print(f"New Destination Set: ({world_x:.2f}, {world_y:.2f})")

def main():
    global GOAL_NODE_WORLD
    
    # Initialize Core Components
    qmap = GlobalMap(resolution_m=0.08, size_m=60.0)
    planner = PathPlanner(qmap)
    navigator = ReactiveNavigator(lookahead_distance=1.0, max_steering=0.52, safe_dist_m=0.6)
    
    # Initialize Hardware
    if HARDWARE_OK:
        myCar = QCar(readMode=0)
        myLidar = QCarLidar(numMeasurements=1000, rangingDistanceMode=2, interpolationMode=0)
    
    cv2.namedWindow('QCar Live SLAM Map')
    cv2.setMouseCallback('QCar Live SLAM Map', mouse_callback, param=qmap)
    
    print("Navigation Setup Complete. Double-click on the map to set a destination.")
    
    sampleTime = 1.0 / 30.0
    last_loop_time = time.time()
    last_plan_time = 0.0
    
    # Drive states
    path = []
    throttle, steering = 0.0, 0.0
    
    try:
        while True:
            start_time = time.time()
            dt = start_time - last_loop_time
            last_loop_time = start_time
            
            # --- 1. SENSOR INGESTION & LOCALIZATION ---
            if HARDWARE_OK:
                myLidar.read()
                
                # Wheel odometry logic
                # Calibrated Odometry for QCar 2
                # motorTach is in rad/s (motor-side)
                # linear_vel = motorTach * PIN_TO_SPUR_RATIO * WHEEL_RADIUS
                wheel_rad = 0.033
                gear_ratio = 0.0954
                wheelbase = 0.256
                
                linear_vel = np.mean(myCar.motorTach) * gear_ratio * wheel_rad
                
                # Yaw rate for Ackermann steering: v * tan(delta) / L
                yaw_rate = (linear_vel * np.tan(steering)) / wheelbase
                
            else:
                linear_vel, yaw_rate, myLidar = 0.0, 0.0, type('mock', (), {'distances': np.zeros(10), 'angles': np.zeros(10)})
                
            # Odometry update (critical for mapping halls with curves!)
            qmap.update_pose_odometry(linear_vel, yaw_rate, dt)
            
            if HARDWARE_OK and len(myLidar.distances) > 0:
                 # Flip angles to standard convention
                 anglesBody = myLidar.angles * -1 + np.pi
                 
                 # Refine pose via scan matching to existing obstacles
                 if np.sum(qmap.get_obstacle_mask()) > 50:
                     qmap.scan_match(myLidar.distances, anglesBody)
                     
                 qmap.add_lidar_scan(myLidar.distances, anglesBody)
            else:
                 pass

            # --- 2. GLOBAL PATH PLANNING ---
            if GOAL_NODE_WORLD is not None:
                # DWA swerves automatically. Only Replan if path is empty, off-track, or periodic
                replan = False
                if len(path) == 0:
                    replan = True
                elif len(path) > 0 and (start_time - last_plan_time > 0.5):
                    # Check distance to path (off-track detection)
                    pos = np.array([qmap.pose[0], qmap.pose[1]])
                    min_dist = np.min(np.linalg.norm(np.array(path) - pos, axis=1))
                    if min_dist > 0.8: # Severely off track due to massive swerve or reverse!
                        replan = True
                        
                if (start_time - last_plan_time > 2.0): # Relax general replan frequency to 2.0s
                    replan = True
                    
                if replan:
                    new_path = planner.compute_path(GOAL_NODE_WORLD)
                    if len(new_path) > 0:
                        path = new_path
                    else:
                        print("No valid path found. Obstacles blocking?")
                    last_plan_time = start_time
            
            # --- 3. HARDWARE CONTROL & REVISION ---
            new_throttle, new_steering = 0.0, 0.0
            
            if GOAL_NODE_WORLD is not None and path:
                if HARDWARE_OK and len(myLidar.distances) > 0:
                    dist_array = myLidar.distances
                    ang_array = anglesBody
                else:
                    dist_array, ang_array = np.array([]), np.array([])
                    
                new_throttle, new_steering, reached = navigator.compute_control(qmap.pose, path, dist_array, ang_array)
                
                if reached:
                    print("Destination Reached.")
                    GOAL_NODE_WORLD = None
                    path = []
                    new_throttle, new_steering = 0.0, 0.0
                elif abs(new_throttle) < 0.01:
                    print("Completely blocked in! Waiting for clear path.")

            # Update our state tracking variables
            throttle, steering = new_throttle, new_steering

            # Execute commanded velocities natively
            if HARDWARE_OK:
                # LEDs logic (Indicator lights)
                leds = np.zeros(8)
                if steering > 0.1: leds[0], leds[2] = 1,1
                elif steering < -0.1: leds[1], leds[3] = 1,1
                
                myCar.read_write_std(throttle=throttle, steering=steering, LEDs=leds)
            
            # --- 4. VISUALIZATION GUI ---
            display_map = qmap.get_map_image()
            
            # Draw Path on map
            for p in path:
                px, py = qmap.world_to_pixel(p[0], p[1])
                cv2.circle(display_map, (px, py), 1, (0, 255, 0), -1)
                
            # Draw Goal
            if GOAL_NODE_WORLD:
                gpx, gpy = qmap.world_to_pixel(GOAL_NODE_WORLD[0], GOAL_NODE_WORLD[1])
                cv2.drawMarker(display_map, (gpx, gpy), (255, 0, 0), cv2.MARKER_CROSS, 10, 2)
            
            cv2.imshow('QCar Live SLAM Map', display_map)
            
            # Loop delay
            key = cv2.waitKey(max(1, int(sampleTime * 1000 - (time.time()-start_time)*1000)))
            if key == 27: # ESC
                break

    except KeyboardInterrupt:
        print("Interrupted! Terminating...")
    
    finally:
        print("Saving final map states to disk...")
        # Save raw log-odds probabilities scaled to 0-255 image
        raw_prob_img = ((np.clip(qmap.grid_logic, qmap.l_min, qmap.l_max) - qmap.l_min) / (qmap.l_max - qmap.l_min) * 255).astype(np.uint8)
        cv2.imwrite("saved_raw_map.png", raw_prob_img)
        cv2.imwrite("saved_display_map.png", qmap.get_map_image())
        
        if HARDWARE_OK:
            myCar.terminate()
            myLidar.terminate()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
