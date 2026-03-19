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


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pybullet as p
import time
from .pybullet_world import create_world
from sensor_msgs.msg import LaserScan
import numpy as np
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math

WHEEL_RADIUS = 0.033   # metres
WHEEL_BASE   = 0.160   # metres (distance between wheels)

# Joint indices in the URDF (verify against your URDF if different)
LEFT_WHEEL_JOINT  = 1
RIGHT_WHEEL_JOINT = 2

# LiDAR mount height above base (metres)
LIDAR_HEIGHT_OFFSET = 0.10

# Minimum valid LiDAR range (avoids self-hits inside robot body)
LIDAR_RANGE_MIN = 0.12


class TurtleBotSim(Node):
    def __init__(self):
        super().__init__('turtlebot3_sim')
        self.get_logger().info("Starting PyBullet TurtleBot3 Simulation...")

        # ── PyBullet world ──────────────────────────────────────────────
        self.physicsClient, self.robotId, self.planeId = create_world()

        # ── ROS2 interfaces ─────────────────────────────────────────────
        self.sub_cmd   = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.pub_scan  = self.create_publisher(LaserScan, 'scan', 10)
        self.pub_odom  = self.create_publisher(Odometry,  'odom', 10)
        self.srv_reset = self.create_service(Trigger, 'reset_robot', self.reset_robot_callback)
        self.pub_collision = self.create_publisher(Bool, 'collision', 10)
        self.collision = False

        # ── Simulation parameters ───────────────────────────────────────
        self.num_lidar_rays = 24
        self.lidar_range    = 5.0

        # Current desired robot velocity [linear_x, angular_z]
        self._linear  = 0.0
        self._angular = 0.0

        # Default start pose
        self.start_pos = [0, -1.6, 0.05]
        self.start_orn = p.getQuaternionFromEuler([0, 0, math.pi/2]) 

        self.timer = self.create_timer(1.0 / 240.0, self.step_sim)

        self.get_logger().info("TurtleBotSim ready.")

    # ────────────────────────────────────────────────────────────────────
    #  Callbacks
    # ────────────────────────────────────────────────────────────────────

    def cmd_vel_callback(self, msg: Twist):
        self._linear  = msg.linear.x
        self._angular = msg.angular.z

    def reset_robot_callback(self, request, response):
        p.resetBasePositionAndOrientation(
            self.robotId, self.start_pos, self.start_orn
        )

        # FIX: Clear residual PyBullet momentum (previously missing)
        p.resetBaseVelocity(self.robotId, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

        # Reset all joint states (wheels, caster, etc.)
        for j in range(p.getNumJoints(self.robotId)):
            p.resetJointState(self.robotId, j, targetValue=0.0, targetVelocity=0.0)

        # Stop wheel motors explicitly
        for joint in (LEFT_WHEEL_JOINT, RIGHT_WHEEL_JOINT):
            p.setJointMotorControl2(
                self.robotId, joint,
                p.VELOCITY_CONTROL,
                targetVelocity=0.0,
                force=10.0,
            )

        # Clear cached command
        self._linear  = 0.0
        self._angular = 0.0

        response.success = True
        response.message = "Robot reset to start position."
        self.get_logger().info("Robot reset.")
        return response

    def step_sim(self):
        # FIX: Proper differential-drive kinematics using robot geometry
        left_vel  = (self._linear - self._angular * WHEEL_BASE / 2.0) / WHEEL_RADIUS
        right_vel = (self._linear + self._angular * WHEEL_BASE / 2.0) / WHEEL_RADIUS

        p.setJointMotorControl2(
            self.robotId, LEFT_WHEEL_JOINT,
            p.VELOCITY_CONTROL, targetVelocity=left_vel, force=50.0
        )
        p.setJointMotorControl2(
            self.robotId, RIGHT_WHEEL_JOINT,
            p.VELOCITY_CONTROL, targetVelocity=right_vel, force=50.0
        )

        # Publish sensors BEFORE stepping so they reflect the current state
        self._publish_lidar_scan()
        self._publish_odometry()

        # Advance the physics engine by one step
        p.stepSimulation()
        # Check collision
        contacts = p.getContactPoints(bodyA=self.robotId)
        self.collision = any(c[2] != self.planeId for c in contacts)

        # Publish collision flag
        msg = Bool()
        msg.data = self.collision
        self.pub_collision.publish(msg)

 

    def _publish_lidar_scan(self):
        scan_msg = LaserScan()
        scan_msg.header.stamp    = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'base_laser'
        scan_msg.angle_min       = 0.0
        scan_msg.angle_max       = 2.0 * np.pi
        scan_msg.angle_increment = 2.0 * np.pi / self.num_lidar_rays
        # FIX: range_min must be > 0 to exclude hits inside the robot body
        scan_msg.range_min       = LIDAR_RANGE_MIN
        scan_msg.range_max       = self.lidar_range
        scan_msg.ranges          = self._perform_lidar_scan()
        self.pub_scan.publish(scan_msg)

    def _perform_lidar_scan(self) -> list:
     
        pos, orn = p.getBasePositionAndOrientation(self.robotId)
        yaw = p.getEulerFromQuaternion(orn)[2]

        origin_z = pos[2] + LIDAR_HEIGHT_OFFSET

        ray_from = []
        ray_to   = []

        for i in range(self.num_lidar_rays):
            angle = yaw + i * (2.0 * np.pi / self.num_lidar_rays)
            ray_from.append([pos[0], pos[1], origin_z])
            ray_to.append([
                pos[0] + self.lidar_range * np.cos(angle),
                pos[1] + self.lidar_range * np.sin(angle),
                origin_z,
            ])

        results = p.rayTestBatch(ray_from, ray_to)

        ranges = []
        for r in results:
            object_uid  = r[0]   # -1 means no hit
            hit_fraction = r[2]  # 0.0–1.0; multiply by range for distance

            if object_uid == -1:
                # No obstacle hit → report max range
                ranges.append(self.lidar_range)
            else:
                distance = hit_fraction * self.lidar_range
                # Clamp to valid sensor range
                ranges.append(float(np.clip(distance, LIDAR_RANGE_MIN, self.lidar_range)))

        return ranges

    def _publish_odometry(self):
        pos, orn     = p.getBasePositionAndOrientation(self.robotId)
        lin_vel, ang_vel = p.getBaseVelocity(self.robotId)

        odom = Odometry()
        odom.header.stamp    = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        odom.pose.pose.position.x    = pos[0]
        odom.pose.pose.position.y    = pos[1]
        odom.pose.pose.position.z    = pos[2]
        odom.pose.pose.orientation.x = orn[0]
        odom.pose.pose.orientation.y = orn[1]
        odom.pose.pose.orientation.z = orn[2]
        odom.pose.pose.orientation.w = orn[3]

        odom.twist.twist.linear.x  = lin_vel[0]
        odom.twist.twist.linear.y  = lin_vel[1]
        odom.twist.twist.linear.z  = lin_vel[2]
        odom.twist.twist.angular.x = ang_vel[0]
        odom.twist.twist.angular.y = ang_vel[1]
        odom.twist.twist.angular.z = ang_vel[2]

        self.pub_odom.publish(odom)



    def destroy_node(self):
        """Disconnect PyBullet physics server on shutdown."""
        self.get_logger().info("Shutting down PyBullet...")
        p.disconnect(self.physicsClient)
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()