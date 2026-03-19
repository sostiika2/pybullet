
# # myown_env.py
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

#     MAX_STEPS      = 5000
#     GOAL_TOLERANCE = 0.2
#     LINEAR_MAX     = 0.9
#     ANGULAR_MAX    = 1.0
#     LIDAR_RANGE    = 3.5
#     LINEAR_MIN = 0.3
#     LIDAR_MAX = 5.0
#     LIDAR_MIN = 0.12
#     GOAL_LIST = [
#     ( 2.0,  2.0), (-2.0,  2.0), ( 2.0, -2.0), (-2.0, -2.0),  # corners
#     ( 0.0,  2.0), ( 0.0, -2.0), ( 2.0,  0.0), (-2.0,  0.0),  # middle edges
#     ( 1.5,  1.5), (-1.5,  1.5), ( 1.5, -1.5), (-1.5, -1.5),  # near pillars but not inside
#     ]
#     SPAWN_LIST = [
#         ( 2.0, -2.0,  3*math.pi/4),
#         (-2.0, -2.0,  math.pi/4),
#         ( 2.0,  2.0, -3*math.pi/4),
#         (-2.0,  2.0, -math.pi/4),
#         ( 0.0, -2.0,  math.pi/2),
#         ( 0.0,  2.0, -math.pi/2),
#         ( 2.0,  0.0,  math.pi),
#         (-2.0,  0.0,  0.0),
#         ( 1.5, -1.5,  math.pi/2),
#         (-1.5,  1.5, -math.pi/2),
#     ]


#     def __init__(self):
#         super().__init__()

#         rclpy.init()
#         self.node = Node("turtlebot_env")

#         self.node.create_subscription(Bool,      "collision", self.collision_cb, 10)
#         self.node.create_subscription(Odometry,  "odom",      self.odom_cb,      10)
#         self.node.create_subscription(LaserScan, "scan",      self.lidar_cb,     10)

#         self.cmd_pub   = self.node.create_publisher(Twist,  "cmd_vel",        10)
#         self.reset_pub = self.node.create_publisher(Pose2D, "/reset_pose",    10)
#         self.goal_pub  = self.node.create_publisher(Point,  "/goal_position", 10)

#         # Robot state
#         self.x         = 0.0
#         self.y         = 0.0
#         self.yaw       = 0.0
#         self.lidar     = np.full(24, self.LIDAR_RANGE, dtype=np.float32)
#         self.collision = False

#         # Episode state
#         self.step_count    = 0
#         self.prev_distance = 0.0
#         self.goal          = np.array([2.0, 2.0])

#         # 24 lidar + distance + angle = 26
#         self.observation_space = spaces.Box(
#             low  = np.array([0.0]*24 + [0, -math.pi], dtype=np.float32),
#             high = np.array([self.LIDAR_RANGE]*24 + [6.0, math.pi], dtype=np.float32),
#         )

#         # continuous [linear, angular] in [-1, 1]
#         self.action_space = spaces.Box(
#             low  = np.array([-1.0, -1.0], dtype=np.float32),
#             high = np.array([ 1.0,  1.0], dtype=np.float32),
#         )

#     # ── Callbacks ─────────────────────────────────────────────────────────
#     def collision_cb(self, msg):
#         self.collision = msg.data

#     def odom_cb(self, msg):
#         self.x = msg.pose.pose.position.x
#         self.y = msg.pose.pose.position.y
#         q    = msg.pose.pose.orientation
#         siny = 2.0 * (q.w * q.z + q.x * q.y)
#         cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#         self.yaw = float(np.arctan2(siny, cosy))

#     def lidar_cb(self, msg):
#         raw        = np.array(msg.ranges, dtype=np.float32)
#         raw        = np.where(np.isfinite(raw), raw, self.LIDAR_RANGE)
#         self.lidar = np.clip(raw[:24], 0.0, self.LIDAR_RANGE)


#     # ── Core ──────────────────────────────────────────────────────────────
#     def _get_obs(self):
#         dx    = self.goal[0] - self.x
#         dy    = self.goal[1] - self.y
#         dist  = float(math.hypot(dx, dy))
#         angle = float(math.atan2(dy, dx) - self.yaw)
#         angle = (angle + math.pi) % (2 * math.pi) - math.pi
#         return np.append(self.lidar, [dist, angle]).astype(np.float32)

#     def _apply_action(self, action):
#         linear  = float(((action[0] + 1.0) / 2.0) * self.LINEAR_MAX)  # [0, 0.6]
#         angular = float(action[1] * self.ANGULAR_MAX)                   # [-1.0, 1.0]
#         cmd = Twist()
#         cmd.linear.x  = linear
#         cmd.angular.z = angular
#         self.cmd_pub.publish(cmd)
#         for _ in range(3):
#             rclpy.spin_once(self.node, timeout_sec=0.05)

#     def _publish_goal(self, x, y):
#         msg   = Point()
#         msg.x = float(x)
#         msg.y = float(y)
#         msg.z = 0.01
#         self.goal_pub.publish(msg)

#     # ── Step ──────────────────────────────────────────────────────────────
#     def step(self, action):
#         self.step_count += 1

#         # Apply the continuous action
#         linear_vel, angular_vel = action
#         self._apply_action(action)  # your _apply_action must accept continuous values

#         obs = self._get_obs()
#         dx = self.goal[0] - self.x
#         dy = self.goal[1] - self.y
#         dist = math.hypot(dx, dy)

#         # Reward shaping
#         progress = self.prev_distance - dist
#         distance_reward = progress * 15.0

#         angle_to_goal = math.atan2(dy, dx)
#         heading = angle_to_goal - self.yaw
#         heading = (heading + math.pi) % (2*math.pi) - math.pi
#         heading_reward = math.cos(heading) * 5.0

#         sharp_turn_penalty = -5.0 * abs(angular_vel)
#         step_penalty = -0.01

#         reward = distance_reward + heading_reward + sharp_turn_penalty + step_penalty

#         # Terminal rewards
#         terminated = False
#         if self.collision:
#             reward = -200.0
#             terminated = True
#         elif dist < self.GOAL_TOLERANCE:
#             reward = 200.0
#             terminated = True

#         truncated = (not terminated) and (self.step_count >= self.MAX_STEPS)
#         self.prev_distance = dist
#         self.collision = False

#         return obs, float(reward), terminated, truncated, {}
#     # ── Reset ─────────────────────────────────────────────────────────────
#     def reset(self, seed=None, options=None):
#         super().reset(seed=seed)

#         while True:
#             self.goal = np.array(random.choice(self.GOAL_LIST))
#             spawn     = random.choice(self.SPAWN_LIST)
#             if math.hypot(self.goal[0] - spawn[0], self.goal[1] - spawn[1]) > 1.5:
#                 break

#         self._publish_goal(self.goal[0], self.goal[1])

#         msg       = Pose2D()
#         msg.x     = float(spawn[0])
#         msg.y     = float(spawn[1])
#         msg.theta = float(spawn[2])
#         self.reset_pub.publish(msg)

#         for _ in range(5):
#             rclpy.spin_once(self.node, timeout_sec=0.05)

#         self.step_count = 0
#         self.collision  = False

#         obs                = self._get_obs()
#         self.prev_distance = float(obs[-2])
#         return obs, {}

#     # ── Close ─────────────────────────────────────────────────────────────
#     def close(self):
#         cmd = Twist()
#         self.cmd_pub.publish(cmd)
#         self.node.destroy_node()
#         rclpy.shutdown()


# myown_env.py
import gymnasium as gym
from gymnasium import spaces
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

    MAX_STEPS      = 8000
    GOAL_TOLERANCE = 0.25
    LINEAR_MAX     = 0.5
    ANGULAR_MAX    = 1.2
    LIDAR_RANGE    = 3.5

    GOAL_LIST = [
        ( 2.0,  2.0), (-2.0,  2.0), ( 2.0, -2.0), (-2.0, -2.0),
        ( 0.0,  2.0), ( 0.0, -2.0), ( 2.0,  0.0), (-2.0,  0.0),
        ( 1.5,  1.5), (-1.5,  1.5), ( 1.5, -1.5), (-1.5, -1.5),
    ]

    SPAWN_LIST = [
        ( 2.0, -2.0,  3*math.pi/4),
        (-2.0, -2.0,  math.pi/4),
        ( 2.0,  2.0, -3*math.pi/4),
        (-2.0,  2.0, -math.pi/4),
        ( 0.0, -2.0,  math.pi/2),
        ( 0.0,  2.0, -math.pi/2),
        ( 2.0,  0.0,  math.pi),
        (-2.0,  0.0,  0.0),
    ]

    def __init__(self):
        super().__init__()

        rclpy.init()
        self.node = Node("turtlebot_env")

        self.node.create_subscription(Bool,      "collision", self.collision_cb, 10)
        self.node.create_subscription(Odometry,  "odom",      self.odom_cb,      10)
        self.node.create_subscription(LaserScan, "scan",      self.lidar_cb,     10)

        self.cmd_pub   = self.node.create_publisher(Twist,  "cmd_vel",        10)
        self.reset_pub = self.node.create_publisher(Pose2D, "/reset_pose",    10)
        self.goal_pub  = self.node.create_publisher(Point,  "/goal_position", 10)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.lidar = np.full(24, self.LIDAR_RANGE, dtype=np.float32)
        self.collision = False

        # Episode
        self.step_count = 0
        self.prev_distance = 0.0
        self.goal = np.array([2.0, 2.0])

        # OBS: 24 lidar + dist + angle
        self.observation_space = spaces.Box(
            low  = np.array([0.0]*24 + [0, -math.pi], dtype=np.float32),
            high = np.array([self.LIDAR_RANGE]*24 + [6.0, math.pi], dtype=np.float32),
        )

        # CONTINUOUS ACTIONS
        self.action_space = spaces.Box(
            low  = np.array([-1.0, -1.0], dtype=np.float32),
            high = np.array([ 1.0,  1.0], dtype=np.float32),
        )

    # ---------------- Callbacks ----------------
    def collision_cb(self, msg):
        self.collision = msg.data

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = float(np.arctan2(siny, cosy))

    def lidar_cb(self, msg):
        raw = np.array(msg.ranges, dtype=np.float32)
        raw = np.where(np.isfinite(raw), raw, self.LIDAR_RANGE)
        self.lidar = np.clip(raw[:24], 0.0, self.LIDAR_RANGE)

    # ---------------- Observation ----------------
    def _get_obs(self):
        dx = self.goal[0] - self.x
        dy = self.goal[1] - self.y

        dist = float(math.hypot(dx, dy))
        angle = math.atan2(dy, dx) - self.yaw
        angle = (angle + math.pi) % (2 * math.pi) - math.pi

        return np.append(self.lidar, [dist, angle]).astype(np.float32)

    # ---------------- Action ----------------
    def _apply_action(self, action):
        linear  = ((action[0] + 1.0) / 2.0) * self.LINEAR_MAX
        angular = action[1] * self.ANGULAR_MAX

        cmd = Twist()
        cmd.linear.x  = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_pub.publish(cmd)

        for _ in range(3):
            rclpy.spin_once(self.node, timeout_sec=0.05)

    # ---------------- Step ----------------
    def step(self, action):
        self.step_count += 1

        self._apply_action(action)
        obs = self._get_obs()

        dist = obs[-2]
        lidar_min = np.min(obs[:24])

        # -------- REWARD (THIS IS THE KEY) --------
        progress = self.prev_distance - dist
        reward = progress * 20.0

        # obstacle avoidance
        if lidar_min < 0.5:
            reward -= (0.5 - lidar_min) * 15.0

        # small step penalty
        reward -= 0.02

        terminated = False

        if self.collision:
            reward = -100.0
            terminated = True

        elif dist < self.GOAL_TOLERANCE:
            reward = 200.0
            terminated = True

        truncated = self.step_count >= self.MAX_STEPS

        self.prev_distance = dist
        self.collision = False

        return obs, float(reward), terminated, truncated, {}

    # ---------------- Reset ----------------
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        while True:
            self.goal = np.array(random.choice(self.GOAL_LIST))
            spawn = random.choice(self.SPAWN_LIST)

            if math.hypot(self.goal[0] - spawn[0], self.goal[1] - spawn[1]) > 1.5:
                break

        # publish goal
        goal_msg = Point()
        goal_msg.x = float(self.goal[0])
        goal_msg.y = float(self.goal[1])
        goal_msg.z = 0.1
        self.goal_pub.publish(goal_msg)

        # reset robot
        msg = Pose2D()
        msg.x = float(spawn[0])
        msg.y = float(spawn[1])
        msg.theta = float(spawn[2])
        self.reset_pub.publish(msg)

        for _ in range(5):
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.step_count = 0
        self.collision = False

        obs = self._get_obs()
        self.prev_distance = obs[-2]

        return obs, {}

    def close(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.node.destroy_node()
        rclpy.shutdown()