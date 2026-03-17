# import pybullet as p
# import pybullet_data
# import numpy as np
# import time

# p.connect(p.GUI)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.setGravity(0,0,-9.8)

# p.loadURDF("plane.urdf")

# wall_height = 1
# thickness = 0.15


# def create_wall_segment(x1,y1,x2,y2):

#     dx = x2 - x1
#     dy = y2 - y1

#     length = np.sqrt(dx**2 + dy**2)

#     cx = (x1 + x2) / 2
#     cy = (y1 + y2) / 2

#     yaw = np.arctan2(dy,dx)

#     collision = p.createCollisionShape(
#         p.GEOM_BOX,
#         halfExtents=[length/2, thickness/2, wall_height/2]
#     )

#     visual = p.createVisualShape(
#         p.GEOM_BOX,
#         halfExtents=[length/2, thickness/2, wall_height/2],
#         rgbaColor=[0.85,0.6,0.3,1]
#     )

#     orientation = p.getQuaternionFromEuler([0,0,yaw])

#     p.createMultiBody(
#         baseMass=0,
#         baseCollisionShapeIndex=collision,
#         baseVisualShapeIndex=visual,
#         basePosition=[cx,cy,wall_height/2],
#         baseOrientation=orientation
#     )


# # -----------------------
# # WALL SEGMENTS
# # (Approx from image)
# # -----------------------

# walls = [

# # outer shape
# (-6,3, -1,3),
# (-1,3, 4,3),
# (4,3, 6,2),
# (6,2, 6,-2),
# (6,-2, 3,-3),
# (3,-3, -3,-3),
# (-3,-3, -6,-2),
# (-6,-2, -6,3),

# # internal walls
# (-1,3, -1,1),
# (-1,1, 2,1),
# (2,1, 2,3),

# (-3,-1, 1,-1),
# (1,-1, 1,1),

# (-4,-1, -4,-3)

# ]


# for w in walls:
#     create_wall_segment(*w)


# # -----------------------
# # furniture
# # -----------------------

# def box(x,y,l,w,h,color):

#     col = p.createCollisionShape(
#         p.GEOM_BOX,
#         halfExtents=[l/2,w/2,h/2]
#     )

#     vis = p.createVisualShape(
#         p.GEOM_BOX,
#         halfExtents=[l/2,w/2,h/2],
#         rgbaColor=color
#     )

#     p.createMultiBody(
#         baseMass=0,
#         baseCollisionShapeIndex=col,
#         baseVisualShapeIndex=vis,
#         basePosition=[x,y,h/2]
#     )


# # tables
# box(-2.5,2.2,1.8,1.2,0.6,[0.7,0.7,0.7,1])
# box(-1,-1.5,0.8,0.5,0.5,[0.7,0.4,0.2,1])

# # bins
# box(-4,-2,0.3,0.3,0.5,[0,0.6,0,1])
# box(2,0.5,0.3,0.3,0.5,[0,0.6,0,1])


# # robot
# robot = p.loadURDF("r2d2.urdf",[-1,0,0.2])






# x, y = 1.0, -2.0  # candidate goal
# if is_position_free(x, y):
#     print("Position is free, safe to use as goal")
# else:
#     print("Position blocked by wall/obstacle")
# while True:
#     p.stepSimulation()
#     time.sleep(1/240)



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

rclpy.init()
node = Node('goal_pub')

pub = node.create_publisher(Point, '/goal_position', 10)

msg = Point()
msg.x = 2.0
msg.y = 0.0
msg.z = 0.01

# Give a tiny delay to ensure the message is sent

pub.publish(msg)
rclpy.spin_once(node, timeout_sec=0.1)

node.get_logger().info(f'Published goal: {msg}')



# Cleanup

rclpy.shutdown()