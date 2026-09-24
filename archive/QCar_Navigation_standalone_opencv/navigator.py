import numpy as np

class ReactiveNavigator:
    def __init__(self, lookahead_distance=1.0, max_steering=0.52, safe_dist_m=0.5, car_half_width=0.2):
        self.lookahead = lookahead_distance
        self.max_steering = max_steering
        self.safe_dist = safe_dist_m
        self.wheelbase = 0.256
        self.car_half_width = car_half_width
        
    def get_closest_target(self, pose, path):
        """ Find target point on path based on lookahead distance """
        pos = np.array([pose[0], pose[1]])
        distances = np.linalg.norm(np.array(path) - pos, axis=1)
        closest_idx = np.argmin(distances)
        
        target_idx = closest_idx
        for i in range(closest_idx, len(path)):
            if np.linalg.norm(np.array(path[i]) - pos) >= self.lookahead:
                target_idx = i
                break
                
        is_end = (target_idx == len(path) - 1)
        return path[target_idx], is_end

    def check_arc_collision(self, steering, distances, angles, is_reverse=False):
        """ Returns the minimum distance to an obstacle along a given steering arc. np.inf if safe. """
        wrapped_angles = np.arctan2(np.sin(angles), np.cos(angles))
        x = distances * np.cos(wrapped_angles)
        y = distances * np.sin(wrapped_angles)
        
        # Predicted lateral position based on steering curvature
        if abs(steering) > 0.05:
            R = self.wheelbase / np.tan(steering)
            y_expected = (x**2) / (2 * R)
            y_relative = y - y_expected
        else:
            y_relative = y

        if not is_reverse:
            # Forward footprint safe area check
            mask = (x > 0.15) & (x < self.safe_dist) & (np.abs(y_relative) < self.car_half_width)
        else:
            # Reverse footprint (look backwards!) Ensure it ignores the car chassis center
            mask = (x < -0.15) & (x > -self.safe_dist) & (np.abs(y_relative) < self.car_half_width)

        masked_dists = np.abs(distances[mask])
        if len(masked_dists) > 0:
            return np.min(masked_dists)
        return np.inf

    def compute_control(self, pose, path, distances, angles):
        """ Calculate steering and throttle using Reactive Swerving and Reverse """
        if not path:
            return 0.0, 0.0, True # Stop if no path
            
        target_pt, is_end = self.get_closest_target(pose, path)
        dist_to_goal = np.linalg.norm([pose[0]-path[-1][0], pose[1]-path[-1][1]])
        if is_end and dist_to_goal < 0.3:
            return 0.0, 0.0, True
            
        # Mathematical Ideal Pure Pursuit Steering
        target_angle = np.arctan2(target_pt[1] - pose[1], target_pt[0] - pose[0])
        alpha = target_angle - pose[2]
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))
        ideal_steering = np.arctan(2.0 * self.wheelbase * np.sin(alpha) / self.lookahead)
        ideal_steering = np.clip(ideal_steering, -self.max_steering, self.max_steering)
        
        # 1. Primary Arc Check
        primary_dist = self.check_arc_collision(ideal_steering, distances, angles, is_reverse=False)
        if primary_dist == np.inf:
            throttle = 0.15 if abs(alpha) < 0.3 else 0.10
            return throttle, ideal_steering, False
            
        # 2. DWA Swerving (Primary path is blocked!)
        best_steering = None
        min_cost = np.inf
        
        # Evaluate 11 candidate arcs across the sweep of the steering rack
        for candidate_steer in np.linspace(-self.max_steering, self.max_steering, 11):
            if abs(candidate_steer - ideal_steering) < 0.05:
                continue # Roughly the same as primary, which we know is blocked
                
            dist_obs = self.check_arc_collision(candidate_steer, distances, angles, is_reverse=False)
            if dist_obs == np.inf: # Safe swerve vector!
                # Prioritize arcs closest to our ideal target
                cost = abs(candidate_steer - ideal_steering)
                if cost < min_cost:
                    min_cost = cost
                    best_steering = candidate_steer
                    
        if best_steering is not None:
            # Proceed with swerve, reducing speed slightly for safety
            return 0.08, best_steering, False
            
        # 3. All forward trajectories blocked (Dead End) -> REVERSE GEAR
        rear_safe = self.check_arc_collision(0.0, distances, angles, is_reverse=True) == np.inf
        
        if rear_safe:
            # Back up while reverse steering to realign
            return -0.15, -ideal_steering, False 
            
        # 4. Completely boxed in (Fatal)
        return 0.0, 0.0, False
