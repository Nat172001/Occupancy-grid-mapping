This Python script implements a basic occupancy grid mapping algorithm using the Robot Operating System 2 (ROS 2) framework. It creates a ROS 2 node called Map, which subscribes to laser scan and pose data topics, and publishes the occupancy grid as a message. The occupancy grid serves as a probabilistic representation of the environment, aiding in mapping and localization tasks for mobile robots.

**Classes**
The Map_cell class represents a single cell in the occupancy grid, initialized with a default log-odds value. The Robot class represents the robot's state, including its position (x, y) and orientation (angle). The main class, Map, inherits from Node and manages the robot's state, processes sensor data, and updates the occupancy grid. Additionally, it handles publishing the occupancy grid as a ROS 2 message.

**Subscriptions and Publications**
The script sets up subscriptions to /scan (LaserScan) to receive laser scan data at 2 Hz and /pose (Pose2D) to receive robot pose data at 10 Hz. It publishes the occupancy grid on the /occupancy_grid (OccupancyGrid) topic at 1 Hz.

**Methods**
The publish_map method converts the internal occupancy grid representation into an OccupancyGrid message and publishes it. The pose_callback method updates the robot's pose based on incoming Pose2D messages. The scan_callback method processes laser scan data to update the occupancy grid, utilizing the Bresenham algorithm to determine the cells affected by each laser scan ray. The update_grid method updates the log-odds values of the cells in the grid, indicating whether they are occupied or free. The pose_to_grid method converts real-world coordinates to grid indices, while the bresenham method implements the Bresenham line algorithm to determine which grid cells a line intersects.

**Saving the Occupancy Grid**
The save_occupancy_grid function is called when a KeyboardInterrupt occurs (e.g., when the user stops the program). It converts the occupancy grid to a numpy array and saves it to a file (env11.npy). This allows for later analysis and use of the generated map.

![](https://github.com/Nat172001/Occupancy-grid-mapping/blob/main/recordings/env1.gif)        ![](https://github.com/Nat172001/Occupancy-grid-mapping/blob/main/recordings/env3.gif)

Usage
To use this script, ensure you have ROS 2 and the necessary dependencies installed. You can run the script using Python 3. The node will continuously update the occupancy grid based on the incoming sensor data and publish it as a ROS 2 message. When you stop the program (e.g., by pressing Ctrl+C), the current state of the occupancy grid is saved to a file for later use.

**Conclusion**
By processing laser scan and pose data, it builds a probabilistic map of the environment, which is essential for many robotic applications such as navigation and path planning.
