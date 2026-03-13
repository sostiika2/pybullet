import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
import time
import csv

class TurtleBotEnv(gym.Env):
    metadata = {"render_modes": ["human"]}

    def __init__(self):
        super().__init__()

        # Initialize ROS
        rclpy.init()
        self.node = Node("turtlebot_nav_env")

        # Collision
        self.collision = False
        self.node.create_subscription(Bool, "collision", self.collision_callback, 10)

        # Publishers
        self.cmd_pub = self.node.create_publisher(Twist, "cmd_vel", 10)

        # Subscribers
        self.node.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.node.create_subscription(LaserScan, "scan", self.lidar_callback, 10)

        # Reset service
        self.reset_client = self.node.create_client(Trigger, "reset_robot")

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
    

        # Goal
        self.goal = np.array([0, 1] )

        # Step counter
        self.step_count = 0
      

        # Action space (continuous)
        self.linear_limit = (0.0, 0.6)
        self.angular_limit = (-1.0, 1.0)
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0]),
            high=np.array([1.0, 1.0]),
            dtype=np.float32
        )

        # Observation space (6 lidar sectors + dx, dy + yaw)
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(8,),
            dtype=np.float32
        )

        # Goal tolerance
        self.goal_tolerance = 0.1
        self.prev_distance = None

        # Default lidar values
        self.lidar_range = 5.0
        self.lidar_ranges = np.full(24, self.lidar_range, dtype=np.float32)

    # ---------------------- Callbacks ---------------------- #
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = np.arctan2(siny, cosy)

    def lidar_callback(self, msg):
        self.lidar_ranges = np.array([r if np.isfinite(r) else self.lidar_range for r in msg.ranges[:24]], dtype=np.float32)

    def collision_callback(self, msg):
        self.collision = msg.data

    # ---------------------- Helpers ---------------------- #
    def lidar_to_sectors(self, scan):
        if len(scan) != 24:
            raise ValueError("Expected 24 LiDAR rays")
        sectors = [
            slice(0, 4),    # Front-left
            slice(4, 8),    # Front
            slice(8, 12),   # Front-right
            slice(12, 16),  # Left
            slice(16, 20),  # Right
            slice(20, 24)   # Back
        ]
        sector_min = np.array([np.min(scan[s]) for s in sectors], dtype=np.float32)
        return sector_min
    
    def get_obs(self):
        # 1. Lidar sectors
        scan = self.lidar_ranges
        sectors = self.lidar_to_sectors(scan)  # shape (6,)
        
        # 2. Relative position to goal
        dx = self.goal[0] - self.x
        dy = self.goal[1] - self.y
        
        # Distance to goal (scalar)
        distance_to_goal = np.linalg.norm([dx, dy])
        
        # Angle to goal relative to robot's heading
        angle_to_goal = np.arctan2(dy, dx) - self.yaw
        # Normalize to [-pi, pi]
        angle_to_goal = (angle_to_goal + np.pi) % (2 * np.pi) - np.pi
        
        # 3. Combine observation
        obs = np.concatenate([sectors, [distance_to_goal, angle_to_goal]], axis=0)
        
        return obs.astype(np.float32)

  

    def scale_action(self, action):
        linear_norm = (action[0] + 1) / 2.0
        linear_vel = linear_norm * (self.linear_limit[1] - self.linear_limit[0]) + self.linear_limit[0]
        angular_vel = action[1] * self.angular_limit[1]
        return linear_vel, angular_vel

    def apply_action(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_pub.publish(cmd)
        rclpy.spin_once(self.node, timeout_sec=0.01)

    # ---------------------- Reward ---------------------- #
    def calculate_reward(self, obs, linear, angular):

        # 1. Current distance to goal
        dx = self.goal[0] - self.x
        dy = self.goal[1] - self.y
        distance = np.linalg.norm([dx, dy])
        
        # 2. Progress reward: how much closer we got since last step
        if self.prev_distance is None:
            progress = 0.0
        else:
            progress = self.prev_distance - distance
        self.prev_distance = distance
        reward = 10.0 * progress  # stronger incentive for moving forward

        # 3. Angle to goal: penalize facing away
        angle_to_goal = np.arctan2(dy, dx) - self.yaw
        angle_to_goal = (angle_to_goal + np.pi) % (2 * np.pi) - np.pi
        reward -= 2.0 * abs(angle_to_goal)  # stronger penalty if not facing goal

        # 4. Spinning penalty: discourage unnecessary rotation
        reward -= 0.05 * abs(angular)

        # 5. Small time penalty to encourage faster goal reaching
        reward -= 0.01

        # 6. Collision penalty: heavy penalty and terminate
        if self.collision:
            reward = -50.0

        # 7. Goal reached: big reward
        if distance < self.goal_tolerance:
            reward = 100.0

        return float(reward)

    # ---------------------- Step & Reset ---------------------- #
    def step(self, action):
        linear, angular = self.scale_action(action)
        self.apply_action(linear, angular)
        obs = self.get_obs()
        reward = self.calculate_reward(obs, linear, angular)
        print(f"Step {self.step_count}: Reward={reward:.2f}, Distance to Goal={np.linalg.norm(self.goal - np.array([self.x, self.y])):.2f}, reward={reward:.2f}  linear={linear:.2f} angular={angular:.2f}")

        done = False
        if np.linalg.norm(self.goal - np.array([self.x, self.y])) < self.goal_tolerance:
            done = True
        if self.collision:
            done = True
        self.step_count += 1
        self.log_data(self.step_count, self.x, self.y, np.linalg.norm(self.goal - np.array([self.x, self.y])), linear, angular, reward)
        return obs, reward, done, False, {}

    def reset(self, seed=None, options=None):
        if seed is not None:
            np.random.seed(seed)
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            print("Waiting for reset service...")
        req = Trigger.Request()
        future = self.reset_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        time.sleep(1.0)
        self.step_count = 0
        self.prev_distance = None
        self.collision = False
        rclpy.spin_once(self.node, timeout_sec=0.1)
        obs = self.get_obs()
        return obs, {}

    # ---------------------- Logging & Close ---------------------- #
    def log_data(self, step, x, y, distance, linear, angular, reward):
        with open("training_log.csv", "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([step, x, y, distance, linear, angular, reward])

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        rclpy.spin_once(self.node, timeout_sec=0.1)
        print(f"Robot stopped at ({self.x:.2f}, {self.y:.2f})")

    def close(self):
        self.stop_robot()
        self.node.destroy_node()
        rclpy.shutdown()