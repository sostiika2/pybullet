import pybullet as p
import pybullet_data
import time
import numpy as np
import math

# Connect to PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

# Load plane
planeId = p.loadURDF("plane.urdf")

# Environment parameters
env_size = 5
wall_height = 1.0
wall_thickness = 0.2
wall_color = [0.8, 0.8, 0.8, 1]

# Helper to create walls
def create_wall(pos, size, color):
    col_box = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
    vis_box = p.createVisualShape(p.GEOM_BOX, halfExtents=size, rgbaColor=color)
    return p.createMultiBody(baseMass=0,
                             baseCollisionShapeIndex=col_box,
                             baseVisualShapeIndex=vis_box,
                             basePosition=pos)

# Outer walls
# create_wall([env_size, 0, wall_height/2], [wall_thickness, env_size, wall_height/2], wall_color)
# create_wall([-env_size, 0, wall_height/2], [wall_thickness, env_size, wall_height/2], wall_color)
# create_wall([0, env_size, wall_height/2], [env_size, wall_thickness, wall_height/2], wall_color)
# create_wall([0, -env_size, wall_height/2], [env_size, wall_thickness, wall_height/2], wall_color)

# House
house_size = [1, 1, 1]
house_pos = [0, 0, house_size[2]]
house_color = [0.7, 0.4, 0.2, 1]
# create_wall(house_pos, house_size, house_color)

# Add realistic obstacles
def add_obstacle(pos, urdf_name, scale=1.0):
    return p.loadURDF(urdf_name, basePosition=pos, globalScaling=scale)

# Tables
# table1 = add_obstacle([2, 2, 0], "table/table.urdf", scale=0.5)
# table2 = add_obstacle([-2, -2, 0], "table/table.urdf", scale=0.5)

# Small boxes / objects
# box1 = add_obstacle([1.5, 0, 0], "cube_small.urdf", scale=0.3)
# box2 = add_obstacle([-1.5, 1.5, 0], "cube_small.urdf", scale=0.3)

# Duck as decoration
# duck = add_obstacle([0, -2, 0], "duck_vhacd.urdf", scale=0.3)

# ----------------------------
# Robot start position
# ----------------------------
startPos = [1, 1, 0.05]
startOrientation = p.getQuaternionFromEuler([0, 0, math.pi/2])  # yaw = 90°
# Load TurtleBot3 Burger
robotId = p.loadURDF("/home/sostika/catkin_ws/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.urdf",
                     startPos, startOrientation)

# Reset robot base and joints
p.resetBasePositionAndOrientation(robotId, startPos, startOrientation)
for j in range(p.getNumJoints(robotId)):
    p.resetJointState(robotId, j, 0)

# Wheel indices
left_wheel = 1
right_wheel = 2
# Simulation loop
while True:
    p.stepSimulation()
    time.sleep(1./240.)
    # for _ in range(1000):
    # p.setJointMotorControl2(robotId, left_wheel, p.VELOCITY_CONTROL, targetVelocity=0.5, force=50)
    # p.setJointMotorControl2(robotId, right_wheel, p.VELOCITY_CONTROL, targetVelocity=0.5, force=50)
    # pos, orn = p.getBasePositionAndOrientation(robotId)
    # roll, pitch, yaw = p.getEulerFromQuaternion(orn)
    # print("Yaw:", yaw)
    left_state = p.getJointState(robotId, left_wheel)
    right_state = p.getJointState(robotId, right_wheel)
    left_actual_vel = left_state[1]
    right_actual_vel = right_state[1]
    print(f"Left wheel velocity: {left_actual_vel:.3f}, Right wheel velocity: {right_actual_vel:.3f}")
        
    
        
      

    # Stop wheels
    # p.setJointMotorControl2(robotId, left_wheel, p.VELOCITY_CONTROL, targetVelocity=0, force=5)
    # p.setJointMotorControl2(robotId, right_wheel, p.VELOCITY_CONTROL, targetVelocity=0, force=5)

    # Reset robot position
    # p.resetBasePositionAndOrientation(robotId, startPos, startOrientation)
    # move robot for some steps
    # for i in range(1000):

    #     p.setJointMotorControl2(boxId,
    #                             left_wheel,
    #                             p.VELOCITY_CONTROL,
    #                             targetVelocity=5,
    #                             force=5)

    #     p.setJointMotorControl2(boxId,
    #                             right_wheel,
    #                             p.VELOCITY_CONTROL,
    #                             targetVelocity=5,
    #                             force=5)

    #     p.stepSimulation()
    #     time.sleep(1./240.)


