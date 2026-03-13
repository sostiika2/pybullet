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