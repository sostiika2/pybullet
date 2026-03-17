import gymnasium as gym
from gymnasium import spaces
import numpy as np
import random
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import csv

from .components import is_position_free


class TurtleBotEnv(gym.Env):


    metadata = {"render_modes": ["human"]}


    MAX_STEPS      = 8000
    GOAL_TOLERANCE = 0.20           # metres — generous for learning
    LIDAR_RANGE    = 5.0            # metres
    LINEAR_MAX     = 0.6            # m/s
    ANGULAR_MAX    = 1.0            # rad/s
    MAP_DIAGONAL   = 7.07           # sqrt(5²+5²) for [-2.5,2.5]²


    FRONT_WARN   = 0.6              # gentle penalty starts here
    FRONT_DANGER = 0.3              # strong penalty starts here


    GOAL_REWARD      =  200.0       # always dominates episode penalties
    COLLISION_REWARD =  -50.0       # firm but not overwhelming

    # Navigation — core signals
    PROGRESS_SCALE   =   20.0       # per metre closed toward goal
    ANGLE_PENALTY    =    0.2       # per radian facing away from goal
                                    # max per step = 0.2 * pi = 0.63
                                    # drives alignment AND recovery

    # Regularisation — very small, just shape behaviour
    ANGULAR_PENALTY  =    0.005     # discourages unnecessary spinning
    STEP_PENALTY     =    0.001     # prefer shorter paths

    # Obstacle — front only, proportional to closeness
    OBSTACLE_WARN_SCALE   =  0.5    # gentle — 0 to 0.3 max per step
    OBSTACLE_DANGER_SCALE =  2.0    # strong — 0 to 0.6 max per step

    # Proximity bonus — pulls robot into goal zone regardless of mode
    PROXIMITY_BONUS  =    5.0       # max +2.0 at goal, 0 at 0.4m
    PROXIMITY_RADIUS =    0.4       # metres

    # Stagnation — prevents hiding in obstacle-free corners
    STAGNATION_STEPS =   50         # look-back window
    STAGNATION_DIST  =    0.05      # must close 5cm over window
    STAGNATION_FAR   =    0.8       # only check if far from goal
    STAGNATION_PEN   =    2.0       # penalty when stuck

    # Grace period — ignore collision right after spawn
    GRACE_STEPS      =   10

    FRONT_RAYS = [23, 0, 1]         # ±15° — only cone that matters

    # ------------------------------------------------------------------ #
    #  Goal positions — pulled inward from walls                          #
    # ------------------------------------------------------------------ #
    GOAL_LIST = [
        ( 0.0,  1.5), ( 1.5,  0.0), (-1.5,  0.0), ( 0.0, -1.5),
        ( 1.0,  1.0), (-1.0,  1.0), ( 1.0, -1.0), (-1.0, -1.0),
        ( 0.0,  0.8), ( 0.8,  0.0), (-0.8,  0.0), ( 0.0, -0.8),
        (-1.5,  1.5), (-1.5, -1.5),
    ]

    # Spawn positions with initial heading
    SPAWN_LIST = [
        ( 1.8, -1.8,  math.pi / 4),
        ( 1.8,  1.8,  5 * math.pi / 4),
        (-1.8,  1.8,  7 * math.pi / 4),
        ( 0.0, -1.8,  math.pi / 2),
        (-1.8, -1.8,  math.pi / 4),
        ( 1.8,  0.0,  math.pi),
        (-1.8,  0.0,  0.0),
        ( 0.0,  1.8, -math.pi / 2),
        ( 1.5, -1.2,  math.pi / 2),
        (-1.5, -1.6,  math.pi / 2),
        ( 1.5,  1.6,  math.pi),
        (-1.5,  1.6,  0.0),
    ]

    # ------------------------------------------------------------------ #
    #  Init                                                                #
    # ------------------------------------------------------------------ #
    def __init__(self):
        super().__init__()

        rclpy.init()
        self.node = Node("turtlebot_nav_env")

        # ROS subscribers
        self.node.create_subscription(
            Bool,      "collision",  self.collision_callback, 10)
        self.node.create_subscription(
            Odometry,  "odom",       self.odom_callback,      10)
        self.node.create_subscription(
            LaserScan, "scan",       self.lidar_callback,     10)

        # ROS publishers
        self.cmd_pub   = self.node.create_publisher(
            Twist,  "cmd_vel",        10)
        self.reset_pub = self.node.create_publisher(
            Pose2D, "/reset_pose",    10)
        self.goal_pub  = self.node.create_publisher(
            Point,  "/goal_position", 10)

        # Robot state — updated every ROS callback
        self.x            = 0.0
        self.y            = 0.0
        self.yaw          = 0.0
        self.lidar_ranges = np.full(24, self.LIDAR_RANGE, dtype=np.float32)
        self.collision    = False

        # Episode state
        self.step_count         = 0
        self.prev_distance      = None
        self.prev_angle_to_goal = None
        self.grace_count        = 0
        self.goal               = np.array([0.0, 1.0])
        self.stag_window        = []

        # ----------------------------------------------------------------
        # Observation — 28 dimensions, zero redundancy
        #
        # [0:24] 24 raw lidar rays normalised [0,1]
        #        Full spatial detail. No sector summaries —
        #        those are redundant since raw rays already have
        #        the same information.
        #
        # [24]   distance to goal normalised [0,1]
        #        How far is the goal.
        #
        # [25]   angle to goal [-pi, pi]
        #        Which direction is the goal relative to heading.
        #
        # [26]   closure rate [-1, 1]
        #        How fast distance is changing this step.
        #        PPO is memoryless — cannot derive from one snapshot.
        #        +1 = closing at max speed, -1 = moving away.
        #
        # [27]   heading rate [-1, 1]
        #        How fast angle to goal is changing this step.
        #        PPO is memoryless — cannot derive from one snapshot.
        #        +1 = aligning fast, -1 = drifting away from goal.
        # ----------------------------------------------------------------
        self.observation_space = spaces.Box(
            low = np.array(
                [0.0] * 24 + [0.0, -math.pi, -1.0, -1.0],
                dtype=np.float32,
            ),
            high = np.array(
                [1.0] * 24 + [1.0,  math.pi,  1.0,  1.0],
                dtype=np.float32,
            ),
            dtype=np.float32,
        )

        # Action space: [linear, angular] both normalised to [-1, 1]
        # Scaled to physical ranges in scale_action()
        self.action_space = spaces.Box(
            low  = np.array([-1.0, -1.0], dtype=np.float32),
            high = np.array([ 1.0,  1.0], dtype=np.float32),
            dtype=np.float32,
        )

        self._publish_goal(self.goal[0], self.goal[1])

    # ------------------------------------------------------------------ #
    #  ROS callbacks                                                       #
    # ------------------------------------------------------------------ #
    def odom_callback(self, msg):
        self.x   = msg.pose.pose.position.x
        self.y   = msg.pose.pose.position.y
        q        = msg.pose.pose.orientation
        siny     = 2.0 * (q.w * q.z + q.x * q.y)
        cosy     = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = float(np.arctan2(siny, cosy))

    def lidar_callback(self, msg):
        self.lidar_ranges = np.array(
            [r if np.isfinite(r) else self.LIDAR_RANGE
             for r in msg.ranges[:24]],
            dtype=np.float32,
        )

    def collision_callback(self, msg):
        self.collision = msg.data

    # ------------------------------------------------------------------ #
    #  Observation builder                                                 #
    # ------------------------------------------------------------------ #
    def get_obs(self):
        # Group 1 — lidar normalised to [0, 1]
        lidar_norm = self.lidar_ranges / self.LIDAR_RANGE

        # Group 2 — goal geometry
        dx            = self.goal[0] - self.x
        dy            = self.goal[1] - self.y
        distance      = float(np.linalg.norm([dx, dy]))
        dist_norm     = float(np.clip(distance / self.MAP_DIAGONAL, 0.0, 1.0))
        angle_to_goal = float(np.arctan2(dy, dx) - self.yaw)
        angle_to_goal = float(
            (angle_to_goal + math.pi) % (2.0 * math.pi) - math.pi
        )

        # Group 3 — temporal feedback
        # Closure rate: distance shrinking (+) or growing (-)?
        if self.prev_distance is None:
            closure_rate = 0.0
        else:
            delta        = self.prev_distance - distance
            closure_rate = float(np.clip(
                delta / (self.LINEAR_MAX * 0.5), -1.0, 1.0
            ))

        # Heading rate: angle to goal shrinking (+) or growing (-)?
        abs_angle = abs(angle_to_goal)
        if self.prev_angle_to_goal is None:
            heading_rate = 0.0
        else:
            delta        = self.prev_angle_to_goal - abs_angle
            heading_rate = float(np.clip(delta / math.pi, -1.0, 1.0))
        self.prev_angle_to_goal = abs_angle

        return np.concatenate([
            lidar_norm,
            [dist_norm, angle_to_goal, closure_rate, heading_rate],
        ]).astype(np.float32)

    # ------------------------------------------------------------------ #
    #  Action scaling                                                      #
    # ------------------------------------------------------------------ #
    def scale_action(self, action):
        # linear:  [-1, 1] → [0, 0.6]  no reverse, max 0.6 m/s
        # angular: [-1, 1] → [-1.0, 1.0] rad/s
        linear  = float(((action[0] + 1.0) / 2.0) * self.LINEAR_MAX)
        angular = float(action[1] * self.ANGULAR_MAX)
        return linear, angular

    def apply_action(self, linear, angular):
        cmd           = Twist()
        cmd.linear.x  = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.05)

    # ------------------------------------------------------------------ #
    #  Reward                                                              #
    # ------------------------------------------------------------------ #
    def calculate_reward(self, linear, angular, distance):


        # Recompute angle to goal from raw state (not from obs)
        dx            = self.goal[0] - self.x
        dy            = self.goal[1] - self.y
        angle_to_goal = float(np.arctan2(dy, dx) - self.yaw)
        angle_to_goal = float(
            (angle_to_goal + math.pi) % (2.0 * math.pi) - math.pi
        )

        # Progress: positive = moved closer, negative = moved away
        if self.prev_distance is None:
            progress = 0.0
        else:
            progress = self.prev_distance - distance
        self.prev_distance = distance

        # Update stagnation window
        self.stag_window.append(progress)
        if len(self.stag_window) > self.STAGNATION_STEPS:
            self.stag_window.pop(0)

        # ============================================================
        # TERMINAL — always checked first
        # ============================================================
        if distance < self.GOAL_TOLERANCE:
            return self.GOAL_REWARD

        if self.collision and self.grace_count <= 0:
            return self.COLLISION_REWARD

        # ============================================================
        # CORE NAVIGATION REWARD
        # ============================================================

        # 1. Progress — main learning signal
        #    +0.20 per step when closing 1cm/step
        #    -0.10 per step when drifting away
        reward = self.PROGRESS_SCALE * progress

        # 2. Angle penalty — continuous alignment signal
        #    = 0   when perfectly facing goal
        #    = 0.63 when facing directly away (pi radians)
        #    This term also drives post-obstacle recovery:
        #    robot turns away to avoid → angle grows → penalty grows →
        #    robot turns back as soon as front is clear
        reward -= self.ANGLE_PENALTY * abs(angle_to_goal)

        # 3. Angular regularisation — small, just reduce spinning
        reward -= self.ANGULAR_PENALTY * abs(angular)

        # 4. Step penalty — tiny, prefer shorter paths
        reward -= self.STEP_PENALTY

        # ============================================================
        # OBSTACLE AVOIDANCE — FRONT RAYS ONLY
        # rays [23, 0, 1] = ±15° straight ahead
        # Side/rear walls never trigger this — robot can pass freely
        # ============================================================
        front_min = float(np.min(self.lidar_ranges[self.FRONT_RAYS]))

        if front_min < self.FRONT_DANGER:
            # Something very close straight ahead
            # max penalty = 2.0 * 0.3 = 0.6 per step
            reward -= self.OBSTACLE_DANGER_SCALE * (self.FRONT_DANGER - front_min)

        elif front_min < self.FRONT_WARN:
            # Obstacle approaching from front — start preparing
            # max penalty = 0.5 * 0.6 = 0.3 per step
            reward -= self.OBSTACLE_WARN_SCALE * (self.FRONT_WARN - front_min)

        # ============================================================
        # PROXIMITY BONUS — always active, mode-independent
        # Pulls robot into goal zone even if wall is nearby
        # max = 5.0 * 0.4 = 2.0 at goal position
        # ============================================================
        if distance < self.PROXIMITY_RADIUS:
            reward += self.PROXIMITY_BONUS * (self.PROXIMITY_RADIUS - distance)

        # ============================================================
        # STAGNATION PENALTY
        # Only fires: far from goal + full window + no progress
        # Prevents robot finding a corner with no obstacles and sitting
        # ============================================================
        if (distance > self.STAGNATION_FAR
                and len(self.stag_window) == self.STAGNATION_STEPS):
            total_progress = sum(self.stag_window)
            if total_progress < self.STAGNATION_DIST:
                severity = float(np.clip(
                    1.0 - total_progress / self.STAGNATION_DIST,
                    0.0, 1.0,
                ))
                reward -= self.STAGNATION_PEN * severity

        return float(reward)

    # ------------------------------------------------------------------ #
    #  Step                                                                #
    # ------------------------------------------------------------------ #
    def step(self, action):
        linear, angular = self.scale_action(action)
        self.apply_action(linear, angular)

        obs      = self.get_obs()
        distance = math.hypot(
            self.goal[0] - self.x,
            self.goal[1] - self.y,
        )
        reward = self.calculate_reward(linear, angular, distance)

        done      = False
        truncated = False

        if distance < self.GOAL_TOLERANCE:
            done = True
        elif self.collision and self.grace_count <= 0:
            done = True

        # Clear collision flag after checking
        self.collision = False

        # Count down grace period
        if self.grace_count > 0:
            self.grace_count -= 1

        self.step_count += 1

        if not done and self.step_count >= self.MAX_STEPS:
            truncated = True

        self.log_data(
            self.step_count, self.x, self.y,
            distance, linear, angular, reward,
        )

        return obs, reward, done, truncated, {}

    # ------------------------------------------------------------------ #
    #  Reset                                                               #
    # ------------------------------------------------------------------ #
    def reset(self, seed=None, options=None):
        if seed is not None:
            np.random.seed(seed)

        # Pick spawn and goal at least 1.5m apart
        while True:
            self.goal = np.array(random.choice(self.GOAL_LIST))
            spawn     = random.choice(self.SPAWN_LIST)
            if math.hypot(
                self.goal[0] - spawn[0],
                self.goal[1] - spawn[1],
            ) > 1.5:
                break

        self._publish_goal(self.goal[0], self.goal[1])

        # Teleport robot to spawn position
        msg       = Pose2D()
        msg.x     = float(spawn[0])
        msg.y     = float(spawn[1])
        msg.theta = float(spawn[2])
        self.reset_pub.publish(msg)

        # Flush ALL stale messages from previous episode
        # before reading any new state
        for _ in range(50):
            rclpy.spin_once(self.node, timeout_sec=0.05)

        # Reset episode state AFTER flush
        self.step_count         = 0
        self.collision          = False
        self.grace_count        = self.GRACE_STEPS
        self.prev_distance      = None
        self.prev_angle_to_goal = None
        self.stag_window        = []

        obs = self.get_obs()

        # Seed prev_distance so step 1 produces valid progress signal
        # instead of 0
        self.prev_distance = math.hypot(
            self.goal[0] - self.x,
            self.goal[1] - self.y,
        )

        return obs, {}

    # ------------------------------------------------------------------ #
    #  Helpers                                                             #
    # ------------------------------------------------------------------ #
    def _publish_goal(self, x, y):
        msg   = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = 0.01
        self.goal_pub.publish(msg)

    def send_goal(self, x, y):
        self._publish_goal(x, y)

    def stop_robot(self):
        cmd           = Twist()
        cmd.linear.x  = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        rclpy.spin_once(self.node, timeout_sec=0.1)

    def log_data(self, step, x, y, distance, linear, angular, reward):
        with open("training_log.csv", "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [step, x, y, distance, linear, angular, reward]
            )

    def close(self):
        self.stop_robot()
        self.node.destroy_node()
        rclpy.shutdown()