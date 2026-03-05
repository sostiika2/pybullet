import gymnasium as gym
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger


class TurtlebotEnv(gym.Env):

    def __init__(self):

        super().__init__()

        # Initializpie ROS
        rclpy.init()

        self.node = rclpy.create_node("turtlebot_rl_env")

        # Publisher: action -> robot
        self.cmd_pub = self.node.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Subscriber: lidar observation
        self.scan_sub = self.node.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        # Reset service
        self.reset_client = self.node.create_client(
            Trigger,
            'reset_robot'
        )

        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Waiting for reset service...")

        # Store latest lidar
        self.scan = None

        # Episode step counter
        self.step_count = 0
        self.max_steps = 500

        # Action space
        # [linear_velocity , angular_velocity]
        self.action_space = gym.spaces.Box(
            low=np.array([0.0, -1.0]),
            high=np.array([0.5, 1.0]),
            dtype=np.float32
        )

        # Observation space (36 lidar beams)
        self.observation_space = gym.spaces.Box(
            low=0.0,
            high=5.0,
            shape=(36,),
            dtype=np.float32
        )

    # ---------------------------------------
    # Lidar Callback
    # ---------------------------------------

    def scan_callback(self, msg):

        scan = np.array(msg.ranges)

        # Replace inf values
        scan[np.isinf(scan)] = 5.0

        self.scan = scan


    # ---------------------------------------
    # Reset Environment
    # ---------------------------------------

    def reset(self, seed=None, options=None):

        self.step_count = 0

        # Call reset service
        req = Trigger.Request()
        future = self.reset_client.call_async(req)

        rclpy.spin_until_future_complete(self.node, future)

        # Wait until lidar arrives
        while self.scan is None:
            rclpy.spin_once(self.node)

        observation = self.scan

        return observation, {}


    # ---------------------------------------
    # Step Function
    # ---------------------------------------

    def step(self, action):

        self.step_count += 1

        linear = float(action[0])
        angular = float(action[1])

        # Publish cmd_vel
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular

        self.cmd_pub.publish(cmd)

        # Wait for new lidar
        rclpy.spin_once(self.node)

        observation = self.scan

        reward = self.compute_reward(observation, linear, angular)

        done = self.check_collision(observation)

        truncated = False

        if self.step_count >= self.max_steps:
            truncated = True

        return observation, reward, done, truncated, {}


    # ---------------------------------------
    # Reward Function
    # ---------------------------------------

    def compute_reward(self, scan, linear, angular):

        min_dist = np.min(scan)

        # Collision
        if min_dist < 0.18:
            return -10.0

        reward = 0.0

        # Encourage forward motion
        reward += linear * 1.0

        # Penalize turning
        reward -= abs(angular) * 0.1

        # Penalize getting close to obstacles
        if min_dist < 0.5:
            reward -= 0.5

        return reward

 
    # ---------------------------------------
    # Collision Detection
    # ---------------------------------------

    def check_collision(self, scan):

        if np.min(scan) < 0.18:
            return True

        return False


    # ---------------------------------------
    # Close Environment
    # ---------------------------------------

    def close(self):

        self.node.destroy_node()
        rclpy.shutdown()