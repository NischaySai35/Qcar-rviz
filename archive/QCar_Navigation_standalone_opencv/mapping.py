import numpy as np
import cv2
from scipy.ndimage import distance_transform_edt

class GlobalMap:
    def __init__(self, resolution_m=0.08, size_m=30.0):
        self.res = resolution_m
        self.size = size_m
        self.pixels = int(self.size / self.res)
        # Center of the map
        self.cx = self.pixels // 2
        self.cy = self.pixels // 2
        
        # Log-odds grid: 0 is unknown, >0 is occupied, <0 is free
        self.grid_logic = np.zeros((self.pixels, self.pixels), dtype=np.float32)
        # Thresholds for occupancy (Log-odds)
        self.l_occ = 1.0
        self.l_free = 1.2
        self.l_max = 4.0
        self.l_min = -5.0
        
        # Global pose (X, Y, Theta) in meters and rad
        self.pose = np.array([0.0, 0.0, 0.0])
        
    def get_obstacle_mask(self):
        """ Returns binary mask of occupied cells """
        return self.grid_logic > 1.0
        
    def update_pose_odometry(self, linear_vel, angular_vel, dt):
        """ Integrate wheel/IMU velocity for odometry (Initial guess) """
        dx = linear_vel * np.cos(self.pose[2]) * dt
        dy = linear_vel * np.sin(self.pose[2]) * dt
        dth = angular_vel * dt
        self.pose += np.array([dx, dy, dth])
        self.pose[2] = np.arctan2(np.sin(self.pose[2]), np.cos(self.pose[2]))
        
    def scan_match(self, distances, angles_body):
        """ Refine self.pose using distance field matching """
        obs_mask = self.get_obstacle_mask()
        if np.sum(obs_mask) < 20: return # Not enough map to match
        
        # Compute distance to nearest obstacle
        dist_field = distance_transform_edt(~obs_mask)
        
        valid = (distances > 0.45) & (distances < 8.0)
        dist = distances[valid]
        ang = angles_body[valid]
        
        best_pose = self.pose.copy()
        best_score = 1e9 # Minimum distance sum
        
        # Search for pose that minimizes distance to nearest obstacles
        for dx in np.arange(-0.06, 0.061, 0.02):
            for dy in np.arange(-0.06, 0.061, 0.02):
                for dth in np.arange(-0.04, 0.041, 0.02):
                    test_pose = best_pose + np.array([dx, dy, dth])
                    g_ang = ang + test_pose[2]
                    gx = test_pose[0] + dist * np.cos(g_ang)
                    gy = test_pose[1] + dist * np.sin(g_ang)
                    
                    px = (self.cy - gy / self.res).astype(np.int32)
                    py = (self.cx - gx / self.res).astype(np.int32)
                    
                    in_b = (px >= 0) & (px < self.pixels) & (py >= 0) & (py < self.pixels)
                    if np.sum(in_b) < 10: continue
                    
                    score = np.sum(dist_field[py[in_b], px[in_b]])
                    if score < best_score:
                        best_score = score
                        best_pose = test_pose
        
        self.pose = best_pose

    def add_lidar_scan(self, distances, angles_in_body, max_dist=12.0):
        """ Probabilistic update of the occupancy grid """
        valid = (distances > 0.45) & (distances < max_dist)
        dist = distances[valid]
        ang = angles_in_body[valid]
        
        if len(dist) == 0: return
        
        # 1. Update Free Space along rays
        car_px = int(self.cy - self.pose[1] / self.res)
        car_py = int(self.cx - self.pose[0] / self.res)
        
        # Draw free space using polygons (faster than individual rays)
        free_poly = []
        g_ang = ang + self.pose[2]
        px = (self.cy - (self.pose[1] + dist * np.sin(g_ang)) / self.res).astype(np.int32)
        py = (self.cx - (self.pose[0] + dist * np.cos(g_ang)) / self.res).astype(np.int32)
        
        # Sort angles for polygon
        sort_idx = np.argsort(ang)
        px, py = px[sort_idx], py[sort_idx]
        
        poly = np.vstack(([car_px, car_py], np.vstack((px, py)).T)).astype(np.int32)
        mask_free = np.zeros_like(self.grid_logic, dtype=np.uint8)
        cv2.fillPoly(mask_free, [poly], 1)
        
        # 2. Update Occupied cells
        mask_occ = np.zeros_like(self.grid_logic, dtype=np.uint8)
        in_b = (px >= 0) & (px < self.pixels) & (py >= 0) & (py < self.pixels)
        mask_occ[py[in_b], px[in_b]] = 1
        
        # Apply Log-Odds updates
        self.grid_logic[mask_free == 1] -= self.l_free
        self.grid_logic[mask_occ == 1] += self.l_occ + self.l_free # Counteract free update at endpoints
        
        # Clip to bounds
        self.grid_logic = np.clip(self.grid_logic, self.l_min, self.l_max)
        
    def get_map_image(self):
        """ Returns BGR image for GUI based on log-odds """
        # Convert log-odds to 0-255 image: 0 unknown (127), positive (occupied=255), negative (free=0)
        display = np.ones_like(self.grid_logic, dtype=np.uint8) * 127
        display[self.grid_logic > 0.5] = 255
        display[self.grid_logic < -0.5] = 0
        
        color_map = cv2.applyColorMap(display, cv2.COLORMAP_BONE)
        # Overlay car position
        car_px = int(self.cy - self.pose[1] / self.res)
        car_py = int(self.cx - self.pose[0] / self.res)
        cv2.circle(color_map, (car_px, car_py), 3, (0, 0, 255), -1)
        
        # Arrow for orientation
        dir_px = int(car_px - 10 * np.sin(self.pose[2]))
        dir_py = int(car_py - 10 * np.cos(self.pose[2]))
        cv2.line(color_map, (car_px, car_py), (dir_px, dir_py), (0, 0, 255), 2)
        
        return color_map

    def world_to_pixel(self, x_m, y_m):
        py = int(self.cx - x_m / self.res)
        px = int(self.cy - y_m / self.res)
        return (px, py)

    def pixel_to_world(self, px, py):
        x_m = (self.cx - py) * self.res
        y_m = (self.cy - px) * self.res
        return x_m, y_m
