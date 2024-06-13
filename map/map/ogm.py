#!/usr/bin/env python3

#pose and laser scan datas are available
#laser scan angle-> -103.13240039172833917 to 103.13240039172833917

#the map is static
#probability of a cell occupied is not dependent on other cells

import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import math

GRID_CORNERS = [(-5, -5), (-5, 5), (5, 5), (5, -5), (-5, -5)]
GRID_SIZE_X = 50
GRID_SIZE_Y = 50

class Map_cell:
    def __init__(self):
        self.log_odds = 0  # Initialize with a default log odds, which corresponds to a 0.5 probability

class Robot:
    def __init__(self):
        self.x = None
        self.y = None
        self.angle = None

class Map(Node):
    def __init__(self):
        super().__init__("Map")
        self.my_robot = Robot()
        self.my_map = [[Map_cell() for _ in range(GRID_SIZE_Y)] for _ in range(GRID_SIZE_X)]
        self.create_subscription(LaserScan,"/scan",self.scan_callback,100) #2 hz
        self.create_subscription(Pose2D,"/pose",self.pose_callback,100) #10 hz
        self.og_pub = self.create_publisher(OccupancyGrid, "/occupancy_grid",10)
        self.create_timer(1,self.publish_map)
    
    def publish_map(self):
        og_publish = OccupancyGrid()
        og_publish.header.stamp = self.get_clock().now().to_msg()
        og_publish.header.frame_id = "world"
        og_publish.info.resolution = 0.2
        og_publish.info.width = GRID_SIZE_X
        og_publish.info.height = GRID_SIZE_Y
        og_publish.info.origin.position.x = -5.0
        og_publish.info.origin.position.y = -5.0
        og_publish.info.origin.position.z = 0.0
        og_publish.info.origin.orientation.x = 0.7071068
        og_publish.info.origin.orientation.y = 0.7071068
        og_publish.info.origin.orientation.z = 0.0
        og_publish.info.origin.orientation.w = 0.0

        og_array = []
        for i in range(GRID_SIZE_X):
            for j in range(GRID_SIZE_Y):
                log_odds = max(min(self.my_map[i][j].log_odds, 100), -100)
                prob = 1.0 / (1.0 + math.exp(-log_odds))  # Convert from log odds to probability
                og_array.append(int(prob * 100))

        og_publish.data = og_array
        self.og_pub.publish(og_publish)

    def pose_callback(self, msg: Pose2D):
        self.my_robot.x = msg.x
        self.my_robot.y = msg.y
        self.my_robot.angle = (msg.theta + math.pi) % (2 * math.pi) - math.pi

    def scan_callback(self, msg: LaserScan):
        min_angle = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        robot_loc = [self.my_robot.x, self.my_robot.y]
        angle = min_angle
        for ray in ranges:
            if not math.isnan(ray):
                ray_angle = self.my_robot.angle + angle
                if math.isinf(ray):
                    #continue
                    # Infinite rays should update cells along the ray as free
                    end_x = robot_loc[0] + math.cos(ray_angle) * 8
                    end_y = robot_loc[1] + math.sin(ray_angle) * 8
                   
                else:
                    end_x = robot_loc[0] + math.cos(ray_angle) * ray
                    end_y = robot_loc[1] + math.sin(ray_angle) * ray

                start = self.pose_to_grid(robot_loc[0], robot_loc[1])
                end = self.pose_to_grid(end_x, end_y)
                grid_list = self.bresenham(start[0], start[1], end[0], end[1])
                self.update_grid(grid_list, math.isinf(ray))

            angle += angle_increment

    def update_grid(self, grids, is_infinite):
        
        free_log_odds = math.log(0.25 / 0.75)  # Example values for free space
        occ_log_odds = math.log(0.92 / 0.08)   # Example values for occupied space
        for i, grid in enumerate(grids):
            if i == len(grids) - 1 and not is_infinite:
                self.my_map[grid[0]][grid[1]].log_odds += occ_log_odds
            else:
                self.my_map[grid[0]][grid[1]].log_odds += free_log_odds


    def pose_to_grid(self, x, y):
        grid_x = int((x - GRID_CORNERS[0][0]) / (GRID_CORNERS[2][0] - GRID_CORNERS[0][0]) * (GRID_SIZE_X - 1))
        grid_y = int((y - GRID_CORNERS[0][1]) / (GRID_CORNERS[2][1] - GRID_CORNERS[0][1]) * (GRID_SIZE_Y - 1))
        return max(0, min(grid_x, GRID_SIZE_X - 1)), max(0, min(grid_y, GRID_SIZE_Y - 1))

    def bresenham(self, x1, y1, x2, y2):
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy
        x, y = x1, y1

        line_points = []
        while True:
            line_points.append((x, y))
            if x == x2 and y == y2:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return line_points

def save_occupancy_grid(occupancy_grid):
    # Convert occupancy grid to numpy array
    occupancy_array = np.zeros((GRID_SIZE_X, GRID_SIZE_Y))
    for i in range(GRID_SIZE_X):
        for j in range(GRID_SIZE_Y):
            log_odds = max(min(occupancy_grid[i][j].log_odds, 100), -100)
            prob = 1.0 - (1.0 / (1.0 + math.exp(-log_odds)))
            if prob>=0.6:
                occupancy_array[i][j] = 1
            else:
                occupancy_array[i][j] = 0
    
    np.save("env11.npy", occupancy_array)
    print("Occupancy grid saved as occupancy_grid.npy.")

def main(args=None):
    rclpy.init(args=args)
    node = Map()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Save occupancy grid when KeyboardInterrupt occurs
        save_occupancy_grid(node.my_map)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()