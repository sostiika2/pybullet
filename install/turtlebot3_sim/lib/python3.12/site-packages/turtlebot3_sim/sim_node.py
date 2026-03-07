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
from geometry_msgs.msg import Quaternion

class TurtleBotSim(Node):
    def __init__(self):
        super().__init__('turtlebot3_sim')
        self.get_logger().info("Starting PyBullet TurtleBot3 Simulation...")

        # Initialize PyBullet world
        self.physicsClient, self.robotId = create_world()
        self.last_cmd_time = time.time()

        # Velocity subscriber
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        #Lidar publisher
        self.pub_scan = self.create_publisher(LaserScan, 'scan', 10)

        # Odometry publisher
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)

        # --- Service ---
        self.srv_reset = self.create_service(Trigger, 'reset_robot', self.reset_robot_callback)

        # --- Simulation parameters ---
        self.num_lidar_rays = 36
        self.lidar_range = 5.0
        
        self.robot_velocity = [0,0]  # linear, angular

        # Timer for simulation step
        self.timer = self.create_timer(1/240, self.step_sim)
        #   # Default start position and orientation
        self.startPos = [-4, 0, 0.05]
        self.startOrientation = p.getQuaternionFromEuler([0, 0, 0])

    def cmd_vel_callback(self, msg):
        self.robot_velocity[0] = msg.linear.x
        self.robot_velocity[1] = msg.angular.z
        self.last_cmd_time = time.time()

    def step_sim(self):
        linear, angular = self.robot_velocity
        wheel_velocity = [linear - angular, linear + angular]

        # Apply to TurtleBot3 wheels
        p.setJointMotorControl2(self.robotId, 1, p.VELOCITY_CONTROL, targetVelocity=wheel_velocity[0])
        p.setJointMotorControl2(self.robotId, 2, p.VELOCITY_CONTROL, targetVelocity=wheel_velocity[1])
        #scan
        self.publish_lidar_scan()
        self.publish_odometry()
        p.stepSimulation()
        time.sleep(1/240)
        # if time.time() - self.last_cmd_time > 3:  # 0.2 sec timeout
        #     self.robot_velocity = [0,0]
    
    # --- Helper: perform Lidar and publish ---
    def publish_lidar_scan(self):
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'base_laser'
        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 2*np.pi
        scan_msg.angle_increment = 2*np.pi / self.num_lidar_rays
        scan_msg.range_min = 0.0
        scan_msg.range_max = self.lidar_range
        scan_msg.ranges = self.perform_lidar_scan()
        self.pub_scan.publish(scan_msg)
    
     # --- Lidar rays ---
    def perform_lidar_scan(self):
        pos, orn = p.getBasePositionAndOrientation(self.robotId)
        yaw = p.getEulerFromQuaternion(orn)[2]

        ray_from = []
        ray_to = []

        for i in range(self.num_lidar_rays):
            angle = yaw + (i * 2*np.pi / self.num_lidar_rays)
            start = [pos[0], pos[1], pos[2] + 0.1]
            end = [pos[0] + self.lidar_range*np.cos(angle),
                   pos[1] + self.lidar_range*np.sin(angle),
                   pos[2] + 0.1]
            ray_from.append(start)
            ray_to.append(end)

        results = p.rayTestBatch(ray_from, ray_to)
        ranges = []
        for r in results:
            hit_pos = r[3]
            if hit_pos is None:
                ranges.append(self.lidar_range)
            else:
                dist = np.linalg.norm(np.array(hit_pos) - np.array(pos))
                ranges.append(dist)
        return ranges

    def reset_robot_callback(self, request, response):
        # Reset robot pose
        p.resetBasePositionAndOrientation(self.robotId, self.startPos, self.startOrientation)
    
         # Reset wheels
        for j in range(p.getNumJoints(self.robotId)):
            p.resetJointState(self.robotId, j, 0)

        # Reset internal velocity
        self.robot_velocity = [0, 0]
        self.last_cmd_time = time.time()

        response.success = True
        response.message = "Robot has been reset."
        self.get_logger().info("Robot reset to start position")
        return response

    #for odometry
    def publish_odometry(self):

        pos, orn = p.getBasePositionAndOrientation(self.robotId)
        lin_vel, ang_vel = p.getBaseVelocity(self.robotId)

        odom = Odometry()

        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Position
        odom.pose.pose.position.x = pos[0]
        odom.pose.pose.position.y = pos[1]
        odom.pose.pose.position.z = pos[2]

        # Orientation
        odom.pose.pose.orientation.x = orn[0]
        odom.pose.pose.orientation.y = orn[1]
        odom.pose.pose.orientation.z = orn[2]
        odom.pose.pose.orientation.w = orn[3]

        # Linear velocity
        odom.twist.twist.linear.x = lin_vel[0]
        odom.twist.twist.linear.y = lin_vel[1]
        odom.twist.twist.linear.z = lin_vel[2]

        # Angular velocity
        odom.twist.twist.angular.x = ang_vel[0]
        odom.twist.twist.angular.y = ang_vel[1]
        odom.twist.twist.angular.z = ang_vel[2]

        self.pub_odom.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotSim()
    rclpy.spin(node)
    rclpy.shutdown()