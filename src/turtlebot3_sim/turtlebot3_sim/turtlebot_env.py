# import gymnasium as gym
# from gymnasium import spaces
# import numpy as np
# import random
# import math
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, Pose2D, Point
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import LaserScan
# from std_msgs.msg import Bool
# import csv

# from .components import is_position_free


# class TurtleBotEnv(gym.Env):
#     metadata = {"render_modes": ["human"]}

#     # ------------------------------------------------------------------ #
#     #  Constants                                                           #
#     # ------------------------------------------------------------------ #
#     MAX_STEPS        = 10000     # safety ceiling — should rarely trigger
#     GOAL_TOLERANCE   = 0.15      # metres
#     LIDAR_RANGE      = 5.0       # metres (sensor max)
#     LINEAR_MIN       = 0.2       # m/s — robot always moves forward
#     LINEAR_MAX       = 1.0       # m/s
#     ANGULAR_MAX      = 1.0       # rad/s
#     MAP_DIAGONAL     = 8.5       # metres — diagonal of [-2,4]×[-2,4] map
#     SAFE_DIST        = 0.5       # metres — obstacle penalty starts here

#     GOAL_REWARD      = 100.0
#     COLLISION_REWARD = -200.0

#     PROGRESS_SCALE   = 8.0       # reward for closing distance to goal
#     OBSTACLE_SCALE   = 2.0       # penalty per metre inside SAFE_DIST
#     ANGULAR_PENALTY  = 0.01      # discourages spinning in place
#     STEP_PENALTY     = 0.01      # tiny time penalty — encourages efficiency

#     GRACE_STEPS      = 10        # ignore collision for N steps after reset

#     # ------------------------------------------------------------------ #
#     #  Spawn & goal lists                                                  #
#     # ------------------------------------------------------------------ #
#     GOAL_LIST = [
#         ( 0.5, -1.5),
#         ( 0.0,  1.5),
#         (-1.5,  1.5),
#         (-1.0,  0.0),
#         ( 1.5,  1.5),
#         ( 1.0,  0.0),
#     ]

#     SPAWN_LIST = [
#         ( 0.4, -0.1,  math.pi / 2),
#         (-1.5, -1.5,  0.0),
#         ( 0.0, -1.5,  math.pi / 2),
#         ( 1.5,  0.0,  0.0),
#         ( 1.0, -1.0,  math.pi),
#     ]

#     # ------------------------------------------------------------------ #
#     #  Init                                                                #
#     # ------------------------------------------------------------------ #
#     def __init__(self):
#         super().__init__()

#         rclpy.init()
#         self.node = Node("turtlebot_nav_env")

#         # --- ROS subscribers ---
#         self.node.create_subscription(Bool,      "collision", self.collision_callback, 10)
#         self.node.create_subscription(Odometry,  "odom",      self.odom_callback,      10)
#         self.node.create_subscription(LaserScan, "scan",      self.lidar_callback,     10)

#         # --- ROS publishers ---
#         self.cmd_pub   = self.node.create_publisher(Twist,  "cmd_vel",        10)
#         self.reset_pub = self.node.create_publisher(Pose2D, "/reset_pose",    10)
#         self.goal_pub  = self.node.create_publisher(Point,  "/goal_position", 10)

#         # --- Robot state ---
#         self.x   = 0.0
#         self.y   = 0.0
#         self.yaw = 0.0
#         self.lidar_ranges = np.full(24, self.LIDAR_RANGE, dtype=np.float32)
#         self.collision    = False

#         # --- Episode state ---
#         self.step_count    = 0
#         self.prev_distance = None
#         self.grace_count   = 0

#         # --- Action space: [linear, angular] both normalised to [-1, 1] ---
#         self.action_space = spaces.Box(
#             low=np.array([-1.0, -1.0], dtype=np.float32),
#             high=np.array([ 1.0,  1.0], dtype=np.float32),
#             dtype=np.float32,
#         )

#         # --- Observation space ---
#         # obs[0:24]  = 24 lidar rays normalised to [0, 1]
#         # obs[24]    = distance to goal normalised to [0, 1]
#         # obs[25]    = angle to goal in [-pi, pi]
#         self.observation_space = spaces.Box(
#             low=np.array( [0.0] * 24 + [0.0, -math.pi], dtype=np.float32),
#             high=np.array([1.0] * 24 + [1.0,  math.pi], dtype=np.float32),
#             dtype=np.float32,
#         )

#         # --- Initial goal (visualisation only) ---
#         self.goal = np.array(random.choice(self.GOAL_LIST))
#         self._publish_goal(self.goal[0], self.goal[1])

#     # ------------------------------------------------------------------ #
#     #  ROS callbacks                                                       #
#     # ------------------------------------------------------------------ #
#     def odom_callback(self, msg):
#         self.x = msg.pose.pose.position.x
#         self.y = msg.pose.pose.position.y
#         q     = msg.pose.pose.orientation
#         siny  = 2.0 * (q.w * q.z + q.x * q.y)
#         cosy  = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#         self.yaw = np.arctan2(siny, cosy)

#     def lidar_callback(self, msg):
#         self.lidar_ranges = np.array(
#             [r if np.isfinite(r) else self.LIDAR_RANGE for r in msg.ranges[:24]],
#             dtype=np.float32,
#         )

#     def collision_callback(self, msg):
#         self.collision = msg.data

#     # ------------------------------------------------------------------ #
#     #  Observation                                                         #
#     # ------------------------------------------------------------------ #
#     def get_obs(self):
#         # 24 raw lidar rays normalised to [0, 1]
#         lidar = self.lidar_ranges / self.LIDAR_RANGE

#         dx = self.goal[0] - self.x
#         dy = self.goal[1] - self.y

#         # distance normalised by map diagonal -> [0, 1]
#         distance_to_goal = np.linalg.norm([dx, dy]) / self.MAP_DIAGONAL

#         # angle to goal in robot frame -> [-pi, pi]
#         angle_to_goal = np.arctan2(dy, dx) - self.yaw
#         angle_to_goal = (angle_to_goal + np.pi) % (2 * np.pi) - np.pi

#         return np.concatenate(
#             [lidar, [distance_to_goal, angle_to_goal]]
#         ).astype(np.float32)

#     # ------------------------------------------------------------------ #
#     #  Action helpers                                                      #
#     # ------------------------------------------------------------------ #
#     def scale_action(self, action):
#         """Map [-1, 1] to physical velocity ranges.
#         Linear is clamped to [LINEAR_MIN, LINEAR_MAX] so the robot
#         always moves forward — prevents the policy from learning to
#         stand still as an avoidance strategy."""
#         linear  = self.LINEAR_MIN + ((action[0] + 1) / 2.0) * (self.LINEAR_MAX - self.LINEAR_MIN)
#         angular = action[1] * self.ANGULAR_MAX
#         return float(linear), float(angular)

#     def apply_action(self, linear, angular):
#         cmd = Twist()
#         cmd.linear.x  = linear
#         cmd.angular.z = angular
#         self.cmd_pub.publish(cmd)
#         # spin enough for robot to physically move and for all
#         # callbacks (odom, lidar, collision) to update
#         for _ in range(10):
#             rclpy.spin_once(self.node, timeout_sec=0.05)

#     # ------------------------------------------------------------------ #
#     #  Reward                                                              #
#     # ------------------------------------------------------------------ #
#     def calculate_reward(self, obs, angular):
#         """
#         Reward components:
#           1. Terminal      — goal (+100) or collision (-200)
#           2. Progress      — reward for closing distance to goal
#           3. Alignment     — cos(angle) bonus: +1 facing goal, -1 facing away
#           4. Combined      — extra bonus when BOTH facing AND moving toward goal
#           5. Obstacle      — proportional penalty when lidar < SAFE_DIST
#           6. Regularise    — small angular and step penalties
#         """
#         lidar         = obs[:24] * self.LIDAR_RANGE   # real metres
#         angle_to_goal = obs[25]                        # already [-pi, pi]
#         distance      = math.hypot(self.goal[0] - self.x, self.goal[1] - self.y)

#         # update prev_distance BEFORE any early return so next step
#         # never sees a spurious progress spike after a terminal state
#         if self.prev_distance is None:
#             progress = 0.0
#         else:
#             progress = self.prev_distance - distance
#         self.prev_distance = distance

#         # --- 1. Terminal conditions ---
#         if distance < self.GOAL_TOLERANCE:
#             return self.GOAL_REWARD
#         if self.collision and self.grace_count <= 0:
#             return self.COLLISION_REWARD

#         forward_alignment = math.cos(angle_to_goal)

#         # --- 2. Progress toward goal ---
#         reward = self.PROGRESS_SCALE * progress

#         # --- 3. Heading alignment (continuous signal) ---
#         # +1.0 when perfectly facing goal, -1.0 when facing directly away
#         reward += 1.0 * forward_alignment

#         # --- 4. Combined bonus — only fires when BOTH conditions are true ---
#         # teaches: face the goal THEN move, not just face it while standing still
#         if progress > 0 and forward_alignment > 0:
#             reward += 2.0 * progress * forward_alignment

#         # --- 5. Obstacle proximity ---
#         min_lidar = np.min(lidar)
#         if min_lidar < self.SAFE_DIST:
#             reward -= self.OBSTACLE_SCALE * (self.SAFE_DIST - min_lidar)

#         # --- 6. Regularisation ---
#         reward -= self.ANGULAR_PENALTY * abs(angular)
#         reward -= self.STEP_PENALTY

#         return float(reward)

#     # ------------------------------------------------------------------ #
#     #  Step                                                                #
#     # ------------------------------------------------------------------ #
#     def step(self, action):
#         linear, angular = self.scale_action(action)
#         self.apply_action(linear, angular)

#         obs    = self.get_obs()
#         reward = self.calculate_reward(obs, angular)

#         distance = math.hypot(self.goal[0] - self.x, self.goal[1] - self.y)

#         done      = False
#         truncated = False

#         if distance < self.GOAL_TOLERANCE:
#             done = True
#         elif self.collision and self.grace_count <= 0:
#             done = True

#         # clear collision AFTER done check and reward calculation
#         self.collision = False

     
#         self.step_count += 1

#         if not done and self.step_count >= self.MAX_STEPS:
#             truncated = True

#         self.log_data(
#             self.step_count, self.x, self.y,
#             distance, linear, angular, reward,
#         )

#         return obs, reward, done, truncated, {}

#     # ------------------------------------------------------------------ #
#     #  Reset                                                               #
#     # ------------------------------------------------------------------ #
#     def reset(self, seed=None, options=None):
#         if seed is not None:
#             np.random.seed(seed)

#         # ensure spawn and goal are at least 1m apart
#         while True:
#             self.goal = np.array(random.choice(self.GOAL_LIST))
#             spawn     = random.choice(self.SPAWN_LIST)
#             if math.hypot(self.goal[0] - spawn[0], self.goal[1] - spawn[1]) > 1.0:
#                 break

#         # publish goal marker — no spin_once here to avoid processing
#         # stale /reset_pose messages from previous episode
#         self._publish_goal(self.goal[0], self.goal[1])

#         # reset robot pose in simulation
#         reset_msg       = Pose2D()
#         reset_msg.x     = float(spawn[0])
#         reset_msg.y     = float(spawn[1])
#         reset_msg.theta = float(spawn[2])
#         self.reset_pub.publish(reset_msg)

#         # flush ALL stale messages from previous episode before reading state
#         # this clears any queued collision, odom, lidar, and reset_pose messages
#         for _ in range(50):
#             rclpy.spin_once(self.node, timeout_sec=0.05)

#         # reset episode state AFTER flush so stale collision is cleared
#         self.step_count  = 0
#         self.collision   = False
#         self.grace_count = self.GRACE_STEPS

#         obs = self.get_obs()

#         # seed prev_distance with real starting distance so step 1
#         # always produces a valid progress signal instead of zero
#         self.prev_distance = math.hypot(
#             self.goal[0] - self.x,
#             self.goal[1] - self.y,
#         )

#         return obs, {}

#     # ------------------------------------------------------------------ #
#     #  Helpers                                                             #
#     # ------------------------------------------------------------------ #
#     def _publish_goal(self, x, y):
#         """Publish goal marker without spin_once to avoid processing
#         stale messages at the wrong time."""
#         msg   = Point()
#         msg.x = float(x)
#         msg.y = float(y)
#         msg.z = 0.01
#         self.goal_pub.publish(msg)

#     def send_goal(self, x, y):
#         """Public alias kept for compatibility."""
#         self._publish_goal(x, y)

#     def stop_robot(self):
#         cmd = Twist()
#         cmd.linear.x  = 0.0
#         cmd.angular.z = 0.0
#         self.cmd_pub.publish(cmd)
#         rclpy.spin_once(self.node, timeout_sec=0.1)

#     def log_data(self, step, x, y, distance, linear, angular, reward):
#         with open("training_log.csv", "a", newline="") as f:
#             writer = csv.writer(f)
#             writer.writerow([step, x, y, distance, linear, angular, reward])

#     def close(self):
#         self.stop_robot()
#         self.node.destroy_node()
#         rclpy.shutdown()

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
    metadata = {"render_modes": ["human"]}

    # ---------------- Constants ----------------
    MAX_STEPS      = 2000
    GOAL_TOLERANCE = 0.3
    LIDAR_RANGE    = 3.5
    LINEAR_MAX     = 0.6 #0.8
    ANGULAR_MAX    = 1.0

    # ---------------- Goal & spawn positions ----------------
    GOAL_LIST = [
        ( 1.5,  1.5), (-1.5,  1.5), ( 1.5, -1.5), (-1.5, -1.5),
        ( 0.0, 1.0), ( 0.0, -1.5), ( 1.0,  0.0), (-1.0,  0.0),(-1.5,0),
        ( 1.5,  1.5), (-1.5,  1.5), ( 1.5, -1.5), (-1.5, -1.5),(0,0),(0,0.5),(0.5,0)

    ]
    SPAWN_LIST = [
        ( 1.5, -1.5,  3*math.pi/4),
        (-1.5, -1.5,  math.pi/4),
        ( 1.5,  1.5, -3*math.pi/4),
        (-1.5,  1.5, -math.pi/4),
        ( 0.0, -1.5,  math.pi/2),
        ( 0.0,  1.5, -math.pi/2),
        ( 1.5,  0.0,  math.pi),
        (-1.5,  0.0,  0.0),
        ( 1.5, -1.5,  math.pi/2),
        (-1.5,  1.5, -math.pi/2),
        (0.0,0.0,0.0),
        (0.0,0.0,math.pi/2)
    ]

    def __init__(self):
        super().__init__()

        # ---------------- ROS init ----------------
        rclpy.init()
        self.node = Node("turtlebot_env")

        self.node.create_subscription(Bool,      "collision", self.collision_cb, 10)
        self.node.create_subscription(Odometry,  "odom",      self.odom_cb,      10)
        self.node.create_subscription(LaserScan, "scan",      self.lidar_cb,     10)

        self.cmd_pub   = self.node.create_publisher(Twist,  "cmd_vel",        10)
        self.reset_pub = self.node.create_publisher(Pose2D, "/reset_pose",    10)
        self.goal_pub  = self.node.create_publisher(Point,  "/goal_position", 10)

        # ---------------- Robot state ----------------
        self.x             = 0.0
        self.y             = 0.0
        self.yaw           = 0.0
        self.lidar         = np.full(24, self.LIDAR_RANGE, dtype=np.float32)
        self.collision     = False
        self.step_count    = 0
        self.prev_distance = 0.0
        self.goal          = np.array([2.0, 2.0])

        # Raw unnormalized observations — exactly like PPO_2
        # 24 lidar + distance + angle = 26
        self.observation_space = spaces.Box(
            low  = np.array([0.0]*24 + [0.0, -math.pi], dtype=np.float32),
            high = np.array([self.LIDAR_RANGE]*24 + [5.0, math.pi], dtype=np.float32),
        )
        self.action_space = spaces.Box(
            low  = np.array([0.0, -1.0], dtype=np.float32),
            high = np.array([0.8,  1.0], dtype=np.float32),
        )


    def collision_cb(self, msg):
        self.collision = msg.data

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q    = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = float(np.arctan2(siny, cosy))

    def lidar_cb(self, msg):
        raw        = np.array(msg.ranges, dtype=np.float32)
        raw        = np.where(np.isfinite(raw), raw, self.LIDAR_RANGE)
        self.lidar = np.clip(raw[:24], 7.0, self.LIDAR_RANGE)

    def _get_obs(self):
        dx    = self.goal[0] - self.x
        dy    = self.goal[1] - self.y
        dist  = float(math.hypot(dx, dy))
        angle = float(math.atan2(dy, dx) - self.yaw)
        angle = (angle + math.pi) % (2 * math.pi) - math.pi
        return np.concatenate([self.lidar, [dist, angle]]).astype(np.float32)

    def _apply_action(self, action):
        # linear_min = 0.1
        # linear  =  float(((action[0] + 1.0) / 2.0) * self.LINEAR_MAX)  # [0, 1.0]
        # angular = float(action[1] * self.ANGULAR_MAX)                   # [-1.0, 1.0]
        cmd = Twist()
        cmd.linear.x  = float(action[0])
        cmd.angular.z = float(action[1])
        self.cmd_pub.publish(cmd)
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _publish_goal(self, x, y):
        msg   = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = 0.01
        self.goal_pub.publish(msg)

    def _stop_robot(self):
        cmd = Twist()
        cmd.linear.x  = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


    def step(self, action):
        self.step_count += 1

        linear_vel, angular_vel = action
        self._apply_action(action)

        obs   = self._get_obs()
        dist  = float(obs[-2])
        angle = float(obs[-1])

        front_beams = np.concatenate([obs[0:9], obs[15:24]])
        lidar_min   = float(np.min(front_beams))

        # ---------------- PROGRESS ----------------
        progress = (self.prev_distance - dist) if self.prev_distance is not None else 0.0
    


        alignment_reward  =  math.cos(angle) * 1.0

        if progress >= 0:
            distance_reward = progress * 2.0
        else:
            distance_reward = progress * 4.0

        SAFE_DIST = 0.4
        if lidar_min < SAFE_DIST:
            obstacle_penalty = -0.5 * (1.0 - lidar_min / SAFE_DIST)
        else:
            obstacle_penalty = 0.0

        speed_reward = 0.05 * (linear_vel / self.LINEAR_MAX) * max(progress, 0.0)
        collision_penalty = -10.0 if self.collision else 0.0
        success_reward = 20.0 if dist < self.GOAL_TOLERANCE else 0.0
        step_penalty = -0.005
        truncated = False
        termination_penalty = 0.0
        if self.step_count >= self.MAX_STEPS:
            truncated = True
            termination_penalty = -2.0
        reward = (
            distance_reward +
            obstacle_penalty +
            speed_reward +
            collision_penalty +
            success_reward +
            step_penalty +
            termination_penalty + alignment_reward
        )
        terminated = self.collision or dist < self.GOAL_TOLERANCE
        self.prev_distance = dist
        self.collision = False
        return obs, float(reward), terminated, truncated, {}
        # Reset
    # ──────────────────────────────────────────
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        self._stop_robot()

        # Pick goal and spawn far enough apart
        for _ in range(100):
            self.goal = np.array(random.choice(self.GOAL_LIST))
            spawn     = random.choice(self.SPAWN_LIST)
            if math.hypot(self.goal[0] - spawn[0], self.goal[1] - spawn[1]) > 1.5:
                break

        self._publish_goal(self.goal[0], self.goal[1])

        msg       = Pose2D()
        msg.x     = float(spawn[0])
        msg.y     = float(spawn[1])
        msg.theta = float(spawn[2])
        self.reset_pub.publish(msg)

        for _ in range(30):
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.step_count = 0
        self.collision  = False

        obs                = self._get_obs()
        self.prev_distance = float(obs[-2])   # raw meters, no multiply
        return obs, {}

    def close(self):
        self._stop_robot()
        self.node.destroy_node()
        rclpy.shutdown()