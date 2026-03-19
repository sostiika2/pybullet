import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

def lidar_callback(msg):
    # Print the raw LiDAR ranges exactly as received
    print("LiDAR scan:", msg.ranges)

def main(args=None):
    rclpy.init(args=args)
    node = Node("lidar_listener")

    # Subscribe to /scan topic
    node.create_subscription(LaserScan, "scan", lidar_callback, 10)

    print("Listening to /scan topic...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
