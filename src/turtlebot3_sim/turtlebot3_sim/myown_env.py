# import gymnasium as gym
# from gymnasium import spaces
# import numpy as np
# import random
# import math
# import rcply
# from rcply.node import Node
# from geometry_msgs.msg import Twist, Pose2D, Point
# from nav_msg.msg import Odometry
# from sensor_msgs.msg import LaserScan
# from std_msgs.msg import Bool
# import csv


# class TurtleBotEnv(gym.Env):
#     metadata = {"render_modes:":["human"]}

#     MAX_STEPS = 4000
#     GOAL_TOLERANGE = 0.15
#     LIDAR_RANGE = 5.0
#     LIDAR_RANGE_MIN = 0.12
#     LINEAR_MAX = 0.8
#     ANGULAR_MAX = 1.0
#     MAP_DIAGONAL = 7.07

#     FRONT_WARN = 0.48
#     FRONT_DANGER = 0.25

#     GOAL_REWARD = 200.0
#     COLLISION_REWARD = -100

#     PROGRESS_SCALE = 10.0
#     ANGLE_PENALTY = 0.1

#     STEP_PENALTY = 0.001

#     GOAL_LIST = [
#         (0.0,1.5),
#         (1.5,0.0),
#         (-1.5,0.0),
#         (0.0,-1.5),
#         (0.0,0.8)
#         (1.8,-1.8),
#         (1.8,1.8),
#         (-1.8,1.8),
#         (-1.8,1.8),
#         (0.0,2.0),
#         (0.0,-1.8),
#         (1.8,0),
#         (0,1.8),
#         (-1.8,0),
#         (0.0,-1.8),
#         (1.0,-2.0),
#         (1.0,2.0),
#         (-1.0,2.0)
#     ]
#     SPAWN_LIST = [
#         ( 1.8, -1.8,  math.pi / 4),
#         ( 1.8,  1.8,  5 * math.pi / 4),
#         (-1.8,  1.8,  7 * math.pi / 4),
#         ( 0.0, -1.8,  math.pi / 2),
#         (-1.8, -1.8,  math.pi / 4),
#         ( 1.8,  0.0,  math.pi),
#         (-1.8,  0.0,  0.0),
#         ( 0.0,  1.8, -math.pi / 2),
#         ( 1.5, -1.2,  math.pi / 2),
#         (-1.5, -1.6,  math.pi / 2),
#         ( 1.5,  1.6,  math.pi),
#         (-1.5,  1.6,  0.0),
#     ]
#     LIDAR_INDICES = [0,1,2,3,4,5,6,30,31,32,33,34,35]
#     FRONT_RAYS = [0,1,35]
#     FRONT_RIGHT_RAYS = [30,31,32]
#     FRONT_RIGHT_MID = [33,34,35]
#     FRONT_LEFT_RAYS = [2,3,4]
#     FRONT_LEFT_MID = [5,6]

#     def __init__(self):
#         super().__init__()

#         rcply.init()
#         self.node = Node("navtion_node")

#         self.node.create_subscription(Bool,"collision",self.collision_callback,10)
#         self.node.create_subscription(Odometry,"odom",self.odom_callback,10)
#         self.node.create_subscription(LaserScan,"scan",self.lidar_callback,10)
#         self.cmd_pub = self.node.create_publisher(Twist, "cmd_vel",10)
#         self.reset_pub = self.node.create_publisher(Pose2D,"/reset_pose",10)
#         self.goal_pub = self.node.create_pubsliher(Point,"/goal_position",10)

#         self.x = 0.0
#         self.y = 0.0
#         self.yaw = 0.0
#         self.lidar_ranges = np.full(36,self.LIDAR_RANGES,dtype=np.float32)
#         self.collision = False

#         sefl.step_count = 0
#         self.prev_distance = None
#         self.prev_angle_to_goal = None

#         self.goal = np.array([0.0,1.0])

#         self.observation_space = spaces.Box(
#             low = np.array(
#                 [0.0] * 13 + [0.0,-math.pi],dtype=np.float32,
#             ),
#             high = np.array(
#                 [1.0] * 13 + [1.0,math.pi],dtype = np.float32

#             ),
#             np.float32
#         )

#         self.action_space = spaces.Box(
#             low = np.array([-1.0,-1.0],dtype=np.float32),
#             high = np.array([1.0,1.0],dtype=np.float32),
#             np.float32
#         )

#         self._publish_goal(self.goal[0],self.goal[1])
    

    



#     def collision_callback(self,msg):
#         self.collision = msg.data

    
#     def odom_callback(self,msg):
#         self.x = msg.pose.pose.position.x
#         self.y = msg.pose.pose.position.y
#         q = msg.pose.pose.orientation
#         siny = 2.0 * (q.w * q.z + q.x * q.y)
#         cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#         self.yaw = float(np.arctan2(siny,cosy))

#     def lidar_callback(self,msg):
#         self.lidar_ranges = msg.ranges

#     def apply_action(self,linear,angular):
#         cmd  = Twist()
#         cmd.linear.x = linear
#         cmd.angular.z = angular
#         self.cmd_pub.publish(cmd)
#         for _ in range(10):
#             rcply.spin_once(self.node, timeout_sec= 0.05)

#     def scale_action(self,action):
#         linear = float(((action[0] + 1.0) / 2.0) * self.LINEAR_MAX)
#         angular = float(action[1] * self.ANGULAR_MAX)
#         return linear,angular
    
    
#     def _publish_goal(self,x,y):
#         msg = Point()
#         msg.x = float(x)
#         msg.y = float(y)
#         msg.z = 0.01
#         self.goal_pub.publish(msg)

#     def get_obs(self):
#         lidar_data = [i for i in LIDAR_INDICES] 
#         lidar_norm = -1 + 2 * (lidar_data - 0.12) / (5 - 0.12)
#         lidar_norm = np.clip(lidar_norm, -1.0, 1.0)
#         dx = self.goal[0] - self.x
#         dy = self.goal[1] - self.y
#         distance = np.linalg.norm([dx, dy])
#         dist_norm = np.tanh(distance)
#         angle_to_goal = float(np.arctan2(dy,dx) - self.yaw)
#         angle_to_goal = float(
#             (angle_to_goal + math.pi) % (2.0 * math.pi) - math.pi
#         )
#         angle_norm = angle_to_goal / math.pi

#         return np.concatenate([lidar_norm],[dist_norm,angle_norm])
        
#     def calculate_reward(self,linear,angular,obs):
#         distance  = obs[12]
#         angle_to_goal = obs[13]
#         if self.prev_distance is None:
#             progress = 0.0
#         else:
#             progress = self.prev_distance - distance
#         self.prev_distance = distance

#         if distance < self.GOAL_TOLERANCE:
#             return self.GOAL_REWARD

#         elif self.collision:
#             return self.COLLISION_REWARD
        
#         reward = self.PROGRESS_SCALE * progress
#         reward -= self.STEP_PENALTY
#         return float(reward)
    
#     def step(self,action):
#         linear,angular = self.scale_action(action)
#         self.apply_action(linear,angular)

#         obs = self.get_obs()
#         reward = self.calculate_reward(linear,angular,obs)
#         done = False
#         truncated = False
#         if distance < self.GOAL_TOLERANCE:
#             done = True
#         if not done and self.step_count >= self.MAX_STEPS:
#             truncated = True

#         self.log_data(self.step_count, self.x, self.y,distance,linear,angular,reward)
#         return obs,reward,,done,truncated,{}

    
#     def reset(self,seed= None, opions= None):
#         if seed is not None:
#             np.random.seed(seed)
#         while True:
#             self.goal = np.array(random.choice(self.GOAL_LIST))
#             spawn     = random.choice(self.SPAWN_LIST)
#             if math.hypot(
#                 self.goal[0] - spawn[0],
#                 self.goal[1] - spawn[1],
#             ) > 1.5:
#                 break


#         self._publish_goal(self.goal[0], self.goal[1])
#         # Teleport robot to spawn position
#         msg       = Pose2D()
#         msg.x     = float(spawn[0])
#         msg.y     = float(spawn[1])
#         msg.theta = float(spawn[2])
#         self.reset_pub.publish(msg)

#         # Flush ALL stale messages from previous episode
#         # before reading any new state
#         for _ in range(50):
#             rclpy.spin_once(self.node, timeout_sec=0.05)

#         # Reset episode state AFTER flush
#         self.step_count         = 0
#         self.collision          = False
#         obs = self.get_obs()

#         # Seed prev_distance so step 1 produces valid progress signal
#         # instead of 0
#         self.prev_distance = math.hypot(
#             self.goal[0] - self.x,
#             self.goal[1] - self.y,
#         )

#         return obs, {}
#     def stop_robot(self):
#         cmd           = Twist()
#         cmd.linear.x  = 0.0
#         cmd.angular.z = 0.0
#         self.cmd_pub.publish(cmd)
#         rclpy.spin_once(self.node, timeout_sec=0.1)

#     def log_data(self, step, x, y, distance, linear, angular, reward):
#         with open("training_log.csv", "a", newline="") as f:
#             writer = csv.writer(f)
#             writer.writerow(
#                 [step, x, y, distance, linear, angular, reward]
#             )

#     def close(self):
#         self.stop_robot()
#         self.node.destroy_node()
#         rclpy.shutdown()


    


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


class TurtleBotEnv(gym.Env):
    metadata = {"render_modes": ["human"]}

    # ── Episode / sensor limits ──────────────────────────────────────────
    MAX_STEPS       = 3000
    GOAL_TOLERANCE  = 0.1
    LIDAR_RANGE     = 5.0
    LIDAR_RANGE_MIN = 0.12
    LINEAR_MAX      = 0.6
    ANGULAR_MAX     = 1.0

    # ── Reward weights ───────────────────────────────────────────────────
    GOAL_REWARD      = 1500.0
    COLLISION_REWARD = -500.0
    PROGRESS_SCALE   = 3   # reward per metre closer to goal
    ANGLE_WEIGHT     =  1  # raised from 0.1 – makes facing goal matter
    OBSTACLE_PENALTY = 0.5    # applied when obstacle inside OBSTACLE_THRESH
    STEP_PENALTY     = 0.05  # raised from 0.001 – discourages stalling
    DISTANCE_PULL    = 0.05   # constant pull toward goal every step
    OBSTACLE_THRESH  = 0.4    # metres – front-cone warning distance

    # ── Goals & spawns ───────────────────────────────────────────────────
    GOAL_LIST = [
        ( 0.0,  1.5), ( 1.5,  0.0), (-1.5,  0.0), ( 0.0, -1.5),
        ( 0.0,  0.8), ( 1.8, -1.8), ( 1.8,  1.8), (-1.8,  1.8),
        (-1.8, -1.8), ( 0.0,  2.0), ( 0.0, -1.8), ( 1.8,  0.0),
        ( 0.0,  1.8), (-1.8,  0.0), ( 1.0, -2.0), ( 1.0,  2.0),
        (-1.0,  2.0),
    ]

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

    # 36 rays @ 10° each – symmetric 130° forward arc (13 rays)
    LIDAR_INDICES   = [0, 1, 2, 3, 4, 5, 6, 30, 31, 32, 33, 34, 35]
    # ±30° front cone indices *within* the 13-ray obs vector
    FRONT_CONE_IDX  = [0, 1, 2, 3, 10, 11, 12]

    def __init__(self):
        super().__init__()

        rclpy.init()
        self.node = Node("navigation_node")

        self.node.create_subscription(Bool,      "collision", self.collision_callback, 10)
        self.node.create_subscription(Odometry,  "odom",      self.odom_callback,      10)
        self.node.create_subscription(LaserScan, "scan",      self.lidar_callback,     10)

        self.cmd_pub   = self.node.create_publisher(Twist,  "cmd_vel",        10)
        self.reset_pub = self.node.create_publisher(Pose2D, "/reset_pose",    10)
        self.goal_pub  = self.node.create_publisher(Point,  "/goal_position", 10)

        # Robot state
        self.x            = 0.0
        self.y            = 0.0
        self.yaw          = 0.0
        self.lidar_ranges = np.full(36, self.LIDAR_RANGE, dtype=np.float32)
        self.collision    = False
        self.step_count   = 0
        self.prev_distance = None
        self.goal         = np.array([0.0, 1.0])

        # Obs: 13 lidar ([-1,1]) + distance (tanh,[0,1]) + angle ([-1,1])
        self.observation_space = spaces.Box(
            low  = np.array([-1.0] * 13 + [0.0, -1.0], dtype=np.float32),
            high = np.array([ 1.0] * 13 + [1.0,  1.0], dtype=np.float32),
            dtype=np.float32,
        )

        # Action: [linear, angular] in [-1, 1]
        self.action_space = spaces.Box(
            low  = np.array([-1.0, -1.0], dtype=np.float32),
            high = np.array([ 1.0,  1.0], dtype=np.float32),
            dtype=np.float32,
        )

        self._publish_goal(self.goal[0], self.goal[1])

    def collision_callback(self, msg):
        self.collision = msg.data

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = float(np.arctan2(siny, cosy))

    def lidar_callback(self, msg):
        self.lidar_ranges = np.array(msg.ranges, dtype=np.float32)

    def scale_action(self, action):
        """Map [-1,1] → actual robot velocities."""
        linear  = float(((action[0] + 1.0) / 2.0) * self.LINEAR_MAX)  # [0, LINEAR_MAX]
        angular = float(action[1] * self.ANGULAR_MAX)
        return linear, angular

    def apply_action(self, linear, angular):
        cmd = Twist()
        cmd.linear.x  = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _publish_goal(self, x, y):
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = 0.01
        self.goal_pub.publish(msg)

    
    def get_obs(self):
        # 13 lidar rays → normalised to [-1, 1]
        lidar_data = self.lidar_ranges[self.LIDAR_INDICES]
        lidar_norm = -1.0 + 2.0 * (lidar_data - self.LIDAR_RANGE_MIN) / \
                     (self.LIDAR_RANGE - self.LIDAR_RANGE_MIN)
        lidar_norm = np.clip(lidar_norm, -1.0, 1.0).astype(np.float32)

        # Goal distance → tanh → [0, 1]
        dx           = self.goal[0] - self.x
        dy           = self.goal[1] - self.y
        raw_distance = float(np.linalg.norm([dx, dy]))
        dist_norm    = float(np.tanh(raw_distance))

        # Angle to goal → normalised to [-1, 1]
        angle_to_goal = math.atan2(dy, dx) - self.yaw
        angle_to_goal = (angle_to_goal + math.pi) % (2.0 * math.pi) - math.pi
        angle_norm    = float(angle_to_goal / math.pi)

        return np.concatenate([lidar_norm, [dist_norm, angle_norm]], dtype=np.float32)

    def calculate_reward(self, obs):
        dx = self.goal[0] - self.x
        dy = self.goal[1] - self.y
        raw_distance = float(np.linalg.norm([dx, dy]))

        # Terminal rewards
        if raw_distance < self.GOAL_TOLERANCE:
            return self.GOAL_REWARD
        if self.collision:
            return self.COLLISION_REWARD

        # Progress
        if self.prev_distance is None:
            progress = 0
        else:
            progress = self.prev_distance - raw_distance

        # Angle
        angle_to_goal = obs[14] * math.pi
        angle_reward = math.cos(angle_to_goal)


        # Obstacle awareness (IMPORTANT)
        min_lidar = min(self.lidar_ranges[self.LIDAR_INDICES])
        obstacle_penalty = 0
        if min_lidar < 0.5:
            obstacle_penalty = (0.5 - min_lidar)

        reward = (
            self.PROGRESS_SCALE * progress
            + self.ANGLE_WEIGHT * angle_reward
            - self.STEP_PENALTY
            - obstacle_penalty
        )

        return float(reward)

    def step(self, action):
        self.step_count += 1
        linear, angular = self.scale_action(action)
        self.apply_action(linear, angular)

        obs          = self.get_obs()
        reward       = self.calculate_reward(obs)

        raw_distance = float(np.linalg.norm([
            self.goal[0] - self.x,
            self.goal[1] - self.y,
        ]))


        done      = (raw_distance < self.GOAL_TOLERANCE) or self.collision
        truncated = (not done) and (self.step_count >= self.MAX_STEPS)

        self.log_data(self.step_count, self.x, self.y,
                      raw_distance, linear, angular, reward)
        return obs, reward, done, truncated, {}

    def reset(self, seed=None, options=None):
        if seed is not None:
            np.random.seed(seed)

        # Pick goal/spawn at least 1.5 m apart
        while True:
            self.goal = np.array(random.choice(self.GOAL_LIST))
            spawn     = random.choice(self.SPAWN_LIST)
            if math.hypot(self.goal[0] - spawn[0],
                          self.goal[1] - spawn[1]) > 1.5:
                break

        self._publish_goal(self.goal[0], self.goal[1])

        msg       = Pose2D()
        msg.x     = float(spawn[0])
        msg.y     = float(spawn[1])
        msg.theta = float(spawn[2])
        self.reset_pub.publish(msg)

        # Flush stale sensor messages
        for _ in range(50):
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.step_count = 0
        self.collision  = False
        obs = self.get_obs()

        # Seed prev_distance so step 1 has a valid progress signal
        self.prev_distance = float(np.linalg.norm([
            self.goal[0] - self.x,
            self.goal[1] - self.y,
        ]))
        return obs, {}

    # ── Utilities ─────────────────────────────────────────────────────────
    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x  = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        rclpy.spin_once(self.node, timeout_sec=0.1)

    def log_data(self, step, x, y, distance, linear, angular, reward):
        with open("training_log.csv", "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([step, x, y, distance, linear, angular, reward])

    def close(self):
        self.stop_robot()
        self.node.destroy_node()
        rclpy.shutdown()