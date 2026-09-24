import numpy as np
import heapq
import cv2
from scipy.ndimage import distance_transform_edt

class PathPlanner:
    def __init__(self, globals_map):
        self.map_obj = globals_map
        
    def inflate_obstacles(self, inflation_radius_meters=0.4):
        """ Create a costmap where obstacles are inflated """
        obstacle_mask = self.map_obj.get_obstacle_mask()
        
        if not np.any(obstacle_mask):
             return np.zeros_like(self.map_obj.grid_logic, dtype=np.float32)

        # Distance transform (distance to nearest obstacle in pixels)
        dist_to_obs = distance_transform_edt(~obstacle_mask)
        
        # Convert pixels to meters
        dist_m = dist_to_obs * self.map_obj.res
        
        # Create costmap (higher cost closer to obstacles)
        costmap = np.zeros_like(self.map_obj.grid_logic, dtype=np.float32)
        mask = dist_m < inflation_radius_meters
        # Max cost is nearest obstacle, tapering off
        costmap[mask] = (inflation_radius_meters - dist_m[mask]) / inflation_radius_meters * 100.0
        
        # Hard restriction on walls
        costmap[obstacle_mask] = np.inf
        # Also avoid unknown spots if strict
        # costmap[self.map_obj.grid_logic < 0] = np.inf
        
        return costmap

    def heuristic(self, a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    def a_star_search(self, start_px, goal_px):
        """ Perform A* on the inflated costmap """
        costmap = self.costmap
        h, w = costmap.shape
        
        # Valid checks
        if not (0 <= start_px[0] < w and 0 <= start_px[1] < h): return []
        if not (0 <= goal_px[0] < w and 0 <= goal_px[1] < h): return []
        
        # Unblock start point if car is temporarily inside an obstacle layer
        if costmap[start_px[1], start_px[0]] == np.inf:
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    ny, nx = start_px[1] + dy, start_px[0] + dx
                    if 0 <= nx < w and 0 <= ny < h:
                        costmap[ny, nx] = 0.0
                        
        if costmap[goal_px[1], goal_px[0]] == np.inf: return []
        
        frontier = []
        heapq.heappush(frontier, (0, start_px))
        
        came_from = {start_px: None}
        cost_so_far = {start_px: 0}
        
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        
        while frontier:
            current_priority, current = heapq.heappop(frontier)
            
            # Reached goal
            if np.linalg.norm(np.array(current) - np.array(goal_px)) < 2.0:
                goal_px = current
                break
                
            for dx, dy in directions:
                nx, ny = current[0] + dx, current[1] + dy
                next_node = (nx, ny)
                
                if 0 <= nx < w and 0 <= ny < h:
                    c = costmap[ny, nx]
                    if c == np.inf:
                        continue
                        
                    # Base distance cost (1 for orth, 1.4 for diag)
                    move_cost = np.linalg.norm([dx, dy])
                    new_cost = cost_so_far[current] + move_cost + (c * 0.5)
                    
                    if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                        cost_so_far[next_node] = new_cost
                        priority = new_cost + self.heuristic(next_node, goal_px)
                        heapq.heappush(frontier, (priority, next_node))
                        came_from[next_node] = current
                        
        if goal_px not in came_from:
            return []
            
        # Reconstruct path
        path = []
        curr = goal_px
        while curr != start_px:
            path.append(curr)
            curr = came_from[curr]
        path.append(start_px)
        path.reverse()
        
        # Convert path pixel coords back to meters
        world_path = [self.map_obj.pixel_to_world(p[0], p[1]) for p in path]
        world_path = self.smooth_path(world_path)
        return world_path

    def smooth_path(self, path, weight_data=0.1, weight_smooth=0.4, tolerance=0.01, max_iter=50):
        """ Gradient Descent B-Spline relaxation to turn grid movements into fluid driving arcs """
        if len(path) <= 2: return path
        new_path = [list(p) for p in path]
        change = tolerance
        iters = 0
        while change >= tolerance and iters < max_iter:
            change = 0.0
            for i in range(1, len(path)-1):
                for j in range(2):
                    aux = new_path[i][j]
                    new_path[i][j] += weight_data * (path[i][j] - new_path[i][j]) + \
                                      weight_smooth * (new_path[i-1][j] + new_path[i+1][j] - 2.0 * new_path[i][j])
                    change += abs(aux - new_path[i][j])
            iters += 1
        return [tuple(p) for p in new_path]

    def compute_path(self, goal_m):
        # Update costmap
        self.costmap = self.inflate_obstacles()
        
        # Get start px (current pose)
        start_px = self.map_obj.world_to_pixel(self.map_obj.pose[0], self.map_obj.pose[1])
        goal_px = self.map_obj.world_to_pixel(goal_m[0], goal_m[1])
        
        return self.a_star_search(start_px, goal_px)
