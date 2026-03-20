# import gymnasium as gym
# from gymnasium import spaces
# import numpy as np
# import math
# import random
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, Pose2D, Point
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import LaserScan
# from std_msgs.msg import Bool


# class TurtleBotEnv(gym.Env):
#     metadata = {"render_modes": ["human"]}

#     # ---------------- Constants ----------------
#     MAX_STEPS      = 6000
#     GOAL_TOLERANCE = 0.3
#     LIDAR_RANGE    = 5.0
#     MAX_LINEAR_VEL = 0.8
#     MAX_ANGULAR_VEL = 1.52
#     MIN_LINEAR_VEL = 0.1
#     MIN_ANGULAR_VEL = -1.52


    # # ---------------- Goal & spawn positions ----------------
    # GOAL_LIST = [
    #     ( 1.5,  1.5), (-1.5,  1.5), ( 1.5, -1.5), (-1.5, -1.5),
    #     ( 0.0, 1.0), ( 0.0, -1.5), ( 1.0,  0.0), (-1.0,  0.0),(-1.5,0),
    #     ( 1.5,  1.5), (-1.5,  1.5), ( 1.5, -1.5), (-1.5, -1.5),(0,0),(0,0.5),(0.5,0)

    # ]
    # SPAWN_LIST = [
    #     ( 1.5, -1.5,  3*math.pi/4),
    #     (-1.5, -1.5,  math.pi/4),
    #     ( 1.5,  1.5, -3*math.pi/4),
    #     (-1.5,  1.5, -math.pi/4),
    #     ( 0.0, -1.5,  math.pi/2),
    #     ( 0.0,  1.5, -math.pi/2),
    #     ( 1.5,  0.0,  math.pi),
    #     (-1.5,  0.0,  0.0),
    #     ( 1.5, -1.5,  math.pi/2),
    #     (-1.5,  1.5, -math.pi/2),
    #     (0.0,0.0,0.0),
    #     (0.0,0.0,math.pi/2)
    # ]

from gymnasium import spaces
import gymnasium as gym
import numpy as np
import math
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class TurtleBotEnv(gym.Env):

    def __init__(self):
        super().__init__()

        # ---------- CONSTANTS ----------
        self.LIDAR_RANGE = 3.5
        self.MAX_LINEAR_VEL = 0.6
        self.MAX_ANGULAR_VEL = 1.57
        self.MAX_GOAL_DIST =  7.07

        self.GOAL_TOLERANCE = 0.1
        self.MAX_STEPS = 3000

        # ---------- ROS ----------
        rclpy.init()
        self.node = Node("turtlebot_env")

        self.node.create_subscription(Bool, "collision", self.collision_cb, 10)
        self.node.create_subscription(Odometry, "odom", self.odom_cb, 10)
        self.node.create_subscription(LaserScan, "scan", self.lidar_cb, 10)

        self.cmd_pub = self.node.create_publisher(Twist, "cmd_vel", 10)
        self.reset_pub = self.node.create_publisher(Pose2D, "/reset_pose", 10)
        self.goal_pub = self.node.create_publisher(Point, "/goal_position", 10)

        # ---------- STATE ----------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vel_linear = 0.0
        self.vel_angular = 0.0

        self.lidar = np.full(36, self.LIDAR_RANGE, dtype=np.float32)

        self.goal = np.array([0.0, 0.0])
        self.prev_distance = None
        self.collision = False
        self.step_count = 0

        # ---------- OBSERVATION SPACE ----------
        self.observation_space = spaces.Box(
            low=np.array(
                [0.0] +
                [-1.0, -1.0] +
                [0.0] +
                [-1.0] +
                [0.0]*36,
                dtype=np.float32
            ),
            high=np.array(
                [1.0] +
                [1.0, 1.0] +
                [1.0] +
                [1.0] +
                [1.0]*36,
                dtype=np.float32
            ),
            dtype=np.float32
        )

        # ---------- ACTION SPACE ----------
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
        )

    # ================= ROS CALLBACKS =================

    def collision_cb(self, msg):
        self.collision = msg.data

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny, cosy)

        self.vel_linear = msg.twist.twist.linear.x
        self.vel_angular = msg.twist.twist.angular.z

    def lidar_cb(self, msg):
        self.lidar = np.array(msg.ranges, dtype=np.float32)

    # ================= OBSERVATION =================

    def _get_obs(self):
        dx = self.goal[0] - self.x
        dy = self.goal[1] - self.y

        dist = math.hypot(dx, dy)
        dist_norm = np.clip(dist / self.MAX_GOAL_DIST, 0.0, 1.0)

        angle = math.atan2(dy, dx) - self.yaw
        angle = (angle + math.pi) % (2 * math.pi) - math.pi

        sin_angle = math.sin(angle)
        cos_angle = math.cos(angle)

        linear_norm = np.clip(self.vel_linear / self.MAX_LINEAR_VEL, 0.0, 1.0)
        angular_norm = np.clip(self.vel_angular / self.MAX_ANGULAR_VEL, -1.0, 1.0)

        lidar_norm = np.clip(self.lidar / self.LIDAR_RANGE, 0.0, 1.0)

        obs = np.concatenate([
            [dist_norm],
            [cos_angle],
            [sin_angle],
            [linear_norm],
            [angular_norm],
            lidar_norm
        ]).astype(np.float32)

        return obs

    # ================= ACTION =================

    def _apply_action(self, action):
        linear = ((action[0] + 1.0) / 2.0) * self.MAX_LINEAR_VEL
        angular = float(action[1]) * self.MAX_ANGULAR_VEL

        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)

        self.cmd_pub.publish(cmd)

        for _ in range(5):
            rclpy.spin_once(self.node, timeout_sec=0.02)

    # ================= STEP =================

    def step(self, action):
        self.step_count += 1
        self._apply_action(action)

        obs = self._get_obs()

        dist = float(obs[0])
        lidar_min = float(np.min(obs[5:]) * self.LIDAR_RANGE)

        progress = (self.prev_distance - dist) if self.prev_distance is not None else 0.0

        reward = 0.99 * progress

        SAFE_DIST = 0.3
        if lidar_min < SAFE_DIST:
            reward += -0.2 * (1.0 - lidar_min / SAFE_DIST) ** 2

        reward += -0.005

        if self.collision:
            reward = -20.0

        if dist < self.GOAL_TOLERANCE:
            reward = 20.0

        terminated = self.collision or (dist < self.GOAL_TOLERANCE)

        truncated = False
        if self.step_count >= self.MAX_STEPS:
            truncated = True
            reward += -2.0

        self.prev_distance = dist
        self.collision = False

        return obs, float(reward), terminated, truncated, {}

    # ================= RESET =================

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        self._stop_robot()

        # --- Ensure goal and spawn distance >= 1.5 ---
        while True:

            goal = np.array([
                random.uniform(-1.8, 1.8),
                random.uniform(-1.8, 1.8)
            ])

            spawn = [
                random.uniform(-1.8, 1.8),
                random.uniform(-1.8, 1.8),
                random.uniform(-math.pi, math.pi)
            ]

            dist = math.hypot(goal[0] - spawn[0], goal[1] - spawn[1])

            if dist >= 1.5:
                break

        self.goal = goal

        # publish goal
        goal_msg = Point()
        goal_msg.x = float(goal[0])
        goal_msg.y = float(goal[1])
        goal_msg.z = 0.01
        self.goal_pub.publish(goal_msg)

        # publish spawn
        pose = Pose2D()
        pose.x = float(spawn[0])
        pose.y = float(spawn[1])
        pose.theta = float(spawn[2])
        self.reset_pub.publish(pose)

        for _ in range(20):
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.step_count = 0
        self.collision = False

        obs = self._get_obs()
        self.prev_distance = float(obs[0])

        return obs, {}

    # ================= UTIL =================

    def _stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def close(self):
        self._stop_robot()
        self.node.destroy_node()
        rclpy.shutdown()