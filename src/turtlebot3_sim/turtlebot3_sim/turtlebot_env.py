import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
import time
import csv



class TurtleBotEnv(gym.Env):
    metadata = {"render_modes": ["human"]}

    def __init__(self):
        super().__init__()

        # Initialize ROS
        rclpy.init()
        self.node = Node("turtlebot_nav_env")

        # Publishers
        self.cmd_pub = self.node.create_publisher(Twist, "cmd_vel", 10)

        # Subscribers
        self.node.create_subscription(Odometry, "odom", self.odom_callback, 10)

        # Reset service
        self.reset_client = self.node.create_client(Trigger, "reset_robot")

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Goal (fixed for now)
        self.goal = np.array([1.0, 0.0])

        # Step counter
        self.step_count = 0
        self.max_steps = 900  # Will be updated dynamically

        # Action and observation spaces
        self.linear_limit = (0.0, 0.5)
        self.angular_limit = (-1.0, 1.0)

        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0]),
            high=np.array([1.0, 1.0]),
            dtype=np.float32
        )

        self.observation_space = spaces.Box(
            low=np.array([-np.inf, -np.inf, -np.pi, 0.0]),
            high=np.array([np.inf, np.inf, np.pi, np.inf]),
            dtype=np.float32
        )

        # Goal tolerance
        self.goal_tolerance = 0.05  # 4 cm

        # Previous distance for reward calculation
        self.prev_distance = None

    # ROS odometry callback
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = np.arctan2(siny, cosy)

    # Get current observation
    def get_obs(self):
        dist = np.linalg.norm(self.goal - np.array([self.x, self.y]))
        return np.array([self.x, self.y, self.yaw, dist], dtype=np.float32)

    # Scale normalized action [-1,1] to actual velocity
    def scale_action(self, action):
        linear_norm = (action[0] + 1) / 2.0  # [-1,1] -> [0,1]
        linear_vel = linear_norm * (self.linear_limit[1] - self.linear_limit[0]) + self.linear_limit[0]
        angular_vel = action[1] * self.angular_limit[1]  # scale to [-1,1] max angular
        return linear_vel, angular_vel

    # Apply action multiple times
    def apply_action(self, linear, angular, repeats=2):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        # for _ in range(repeats):
        self.cmd_pub.publish(cmd)
        rclpy.spin_once(self.node, timeout_sec=0.01)

    def log_data(self, step, x, y, distance, linear, angular, reward):
        with open("training_log.csv", "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([step, x, y, distance, linear, angular, reward])

    # Stop robot
    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        rclpy.spin_once(self.node, timeout_sec=0.1)
        print(f"Robot stopped at ({self.x:.2f}, {self.y:.2f})")

    def calculate_reward(self, obs, linear, angular):

        distance = obs[3]

        # Progress reward
        if self.prev_distance is None:
            progress = 0.0
        else:
            progress = self.prev_distance - distance

        self.prev_distance = distance
        reward_progress = 5.0 * progress


        # Heading reward
        angle_to_goal = np.arctan2(self.goal[1] - self.y, self.goal[0] - self.x)
        heading_error = np.arctan2(
            np.sin(angle_to_goal - self.yaw),
            np.cos(angle_to_goal - self.yaw)
        )

        reward_heading = 0.3 * np.cos(heading_error)


        # Encourage forward motion
        reward_forward = 0.1 * linear


        # Penalize excessive turning
        reward_turn = -0.03 * abs(angular)


        reward = reward_progress + reward_heading + reward_forward + reward_turn


        # Goal reward
        if distance < self.goal_tolerance:
            reward = 100.0

        return reward
    

    # Step function
    def step(self, action):
        linear, angular = self.scale_action(action)
        self.apply_action(linear, angular)
        obs = self.get_obs()
        reward = self.calculate_reward(obs, linear, angular)

        done = False
        # Check goal
        if obs[3] < self.goal_tolerance:
            self.stop_robot()
            done = True

        # Check max steps
        self.step_count += 1
        if self.step_count >= self.max_steps:
            done = True
        
        self.log_data(self.step_count, self.x, self.y, obs[3], linear, angular, reward)
        return obs, reward, done, False, {}

    # Reset function
    def reset(self, seed=None, options=None):
        if seed is not None:
            np.random.seed(seed)


        # Call ROS reset service
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            print("Waiting for reset service...")

        req = Trigger.Request()
        future = self.reset_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        time.sleep(1.0)  # Ensure robot is at start
        self.step_count = 0
        self.prev_distance = None

        rclpy.spin_once(self.node, timeout_sec=0.1)
        obs = self.get_obs()
        self.prev_distance = None

        return obs, {}

    # Close environment
    def close(self):
        self.stop_robot()
        self.node.destroy_node()
        rclpy.shutdown()

# import gymnasium as gym
# from gymnasium import spaces
# import numpy as np
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from std_srvs.srv import Trigger
# import time

# class TurtleBotEnv(gym.Env):
#     metadata = {"render_modes": ["human"]}

#     def __init__(self):
#         super().__init__()

#         # Initialize ROS
#         rclpy.init()
#         self.node = Node("turtlebot_nav_env")

#         # Publishers
#         self.cmd_pub = self.node.create_publisher(Twist, "cmd_vel", 10)

#         # Subscribers
#         self.node.create_subscription(Odometry, "odom", self.odom_callback, 10)

#         # Reset service
#         self.reset_client = self.node.create_client(Trigger, "reset_robot")

#         # Robot state
#         self.x = 0.0
#         self.y = 0.0
#         self.yaw = 0.0

#         # Goal (fixed for now)
#         self.goal = np.array([1.0, 0.0])

#         # Step counter
#         self.step_count = 0
#         self.max_steps = 500  # fixed for now

#         # Action and observation spaces
#         self.linear_limit = (0.0, 0.3)
#         self.angular_limit = (-1.0, 1.0)

#         self.action_space = spaces.Box(
#             low=np.array([-1.0, -1.0]),
#             high=np.array([1.0, 1.0]),
#             dtype=np.float32
#         )

#         self.observation_space = spaces.Box(
#             low=np.array([-np.inf, -np.inf, -np.pi, 0.0]),
#             high=np.array([np.inf, np.inf, np.pi, np.inf]),
#             dtype=np.float32
#         )

#         # Goal tolerance
#         self.goal_tolerance = 0.05

#         # Store previous distance for step-wise reward
#         self.prev_distance = None

#     # ROS odometry callback
#     def odom_callback(self, msg):
#         self.x = msg.pose.pose.position.x
#         self.y = msg.pose.pose.position.y

#         q = msg.pose.pose.orientation
#         siny = 2.0 * (q.w * q.z + q.x * q.y)
#         cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#         self.yaw = np.arctan2(siny, cosy)

#     # Get current observation
#     def get_obs(self):
#         dist = np.linalg.norm(self.goal - np.array([self.x, self.y]))
#         return np.array([self.x, self.y, self.yaw, dist], dtype=np.float32)

#     # Scale normalized action [-1,1] to actual velocity
#     def scale_action(self, action):
#         linear_norm = (action[0] + 1) / 2.0  # [-1,1] -> [0,1]
#         linear_vel = linear_norm * (self.linear_limit[1] - self.linear_limit[0]) + self.linear_limit[0]
#         angular_vel = action[1] * self.angular_limit[1]
#         return linear_vel, angular_vel

#     # Apply action multiple times
#     def apply_action(self, linear, angular, repeats=5):
#         cmd = Twist()
#         cmd.linear.x = float(linear)
#         cmd.angular.z = float(angular)
#         for _ in range(repeats):
#             self.cmd_pub.publish(cmd)
#             rclpy.spin_once(self.node, timeout_sec=0.05)

#     # Stop robot
#     def stop_robot(self):
#         cmd = Twist()
#         cmd.linear.x = 0.0
#         cmd.angular.z = 0.0
#         self.cmd_pub.publish(cmd)
#         rclpy.spin_once(self.node, timeout_sec=0.05)
#         print(f"Robot stopped at ({self.x:.2f}, {self.y:.2f})")

#     # Improved reward function
#     def calculate_reward(self, obs, linear, angular):
#         distance = obs[3]

#         # Step-wise progress reward
#         if self.prev_distance is None:
#             progress_reward = 0.0
#         else:
#             progress_reward = self.prev_distance - distance  # positive if getting closer

#         self.prev_distance = distance

#         # Heading to goal
#         angle_to_goal = np.arctan2(self.goal[1] - self.y, self.goal[0] - self.x)
#         heading_error = np.arctan2(np.sin(angle_to_goal - self.yaw), np.cos(angle_to_goal - self.yaw))
#         heading_reward = np.cos(heading_error)  # +1 if facing goal, -1 if opposite

#         # Penalize high angular velocity (encourage smooth motion)
#         angular_penalty = -0.05 * abs(angular)

#         reward = progress_reward + heading_reward + angular_penalty

#         # Big reward for reaching goal
#         if distance < self.goal_tolerance:
#             reward = 100.0

#         return reward

#     # Step function
#     def step(self, action):
#         linear, angular = self.scale_action(action)
#         self.apply_action(linear, angular, repeats=5)

#         obs = self.get_obs()
#         reward = self.calculate_reward(obs, linear, angular)

#         done = False
#         if obs[3] < self.goal_tolerance:
#             done = True  # Episode done; stop will be handled outside
#         self.step_count += 1
#         if self.step_count >= self.max_steps:
#             done = True

#         return obs, reward, done, False, {}

#     # Reset function
#     def reset(self, seed=None, options=None):
#         if seed is not None:
#             np.random.seed(seed)

#         while not self.reset_client.wait_for_service(timeout_sec=1.0):
#             print("Waiting for reset service...")

#         req = Trigger.Request()
#         future = self.reset_client.call_async(req)
#         rclpy.spin_until_future_complete(self.node, future)

#         time.sleep(1.0)
#         self.step_count = 0
#         obs = self.get_obs()
#         self.prev_distance = obs[3]  # reset previous distance

#         return obs, {}

#     # Close environment
#     def close(self):
#         self.stop_robot()
#         self.node.destroy_node()
#         rclpy.shutdown()