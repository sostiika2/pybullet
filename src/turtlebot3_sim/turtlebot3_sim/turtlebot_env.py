import gymnasium as gym
import numpy as np
import rclpy
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from rclpy.node import Node


class TurtlebotNavEnv(gym.Env):

    def __init__(self):

        super().__init__()

        rclpy.init()

        
        self.node = Node("turtlebot_nav_env")

        # Publishers
        self.cmd_pub = self.node.create_publisher(Twist, "cmd_vel", 10)

        # Subscribers
        self.node.create_subscription(Odometry, "odom", self.odom_callback, 10)

        # Reset service
        self.reset_client = self.node.create_client(Trigger, "reset_robot")

        # Robot state
        self.x = 0
        self.y = 0
        self.yaw = 0

        # Goal
        self.goal = np.array([-3.0, 0.0])

        # Gym spaces
        self.action_space = gym.spaces.Box(
            low=np.array([-0.3, -1.0]),
            high=np.array([0.3, 1.0]),
            dtype=np.float32
        )

        self.observation_space = gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(4,),
            dtype=np.float32
        )

        self.step_count = 0

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)

        self.yaw = np.arctan2(siny, cosy)

    def get_obs(self):

        dist = np.linalg.norm(self.goal - np.array([self.x, self.y]))

        return np.array([
            self.x,
            self.y,
            self.yaw,
            dist
        ], dtype=np.float32)

    def step(self, action):

        linear = float(action[0])
        angular = float(action[1])

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular

        self.cmd_pub.publish(cmd)

        rclpy.spin_once(self.node, timeout_sec=0.05)

        obs = self.get_obs()

        dist = obs[3]

        reward = -dist

        done = False

        if dist < 0.3:
            reward = 100
            done = True

        self.step_count += 1

        if self.step_count > 500:
            done = True

        return obs, reward, done, False, {}

    def reset(self, seed=None, options=None):

        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            print("Waiting for reset service...")

        req = Trigger.Request()
        future = self.reset_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        time.sleep(1)

        self.step_count = 0

        rclpy.spin_once(self.node, timeout_sec=0.1)

        obs = self.get_obs()

        return obs, {}

    def close(self):

        self.node.destroy_node()
        rclpy.shutdown()