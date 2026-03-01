# import pybullet as p
# import pybullet_data
# import time
# import numpy as np
# # Connect to PyBullet
# physicsClient = p.connect(p.GUI)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.setGravity(0, 0, -10)

# # Load plane
# planeId = p.loadURDF("plane.urdf")
# # Function to create a wall
# def create_wall(pos, size, color):
#     collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
#     visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=size, rgbaColor=color)
#     body = p.createMultiBody(baseMass=0,
#                              baseCollisionShapeIndex=collision_shape,
#                              baseVisualShapeIndex=visual_shape,
#                              basePosition=pos)
#     return body

# # Environment parameters
# env_size = 5
# wall_height = 1.0
# wall_thickness = 0.2
# wall_color = [0.8, 0.8, 0.8, 1]

# # Outer walls
# create_wall([env_size, 0, wall_height/2], [wall_thickness, env_size, wall_height/2], wall_color)
# create_wall([-env_size, 0, wall_height/2], [wall_thickness, env_size, wall_height/2], wall_color)
# create_wall([0, env_size, wall_height/2], [env_size, wall_thickness, wall_height/2], wall_color)
# create_wall([0, -env_size, wall_height/2], [env_size, wall_thickness, wall_height/2], wall_color)

# # House
# house_size = [0.5, 0.5, 0.5]
# house_pos = [0, 0, house_size[2]]
# house_color = [0.7, 0.4, 0.2, 1]
# create_wall(house_pos, house_size, house_color)

# # Add realistic obstacles
# def add_obstacle(pos, urdf_name, scale=1.0):
#     return p.loadURDF(urdf_name, basePosition=pos, globalScaling=scale)

# # Tables
# table1 = add_obstacle([2, 2, 0], "table/table.urdf", scale=0.5)
# table2 = add_obstacle([-2, -2, 0], "table/table.urdf", scale=0.5)

# # Small boxes / objects
# box1 = add_obstacle([1.5, 0, 0], "cube_small.urdf", scale=0.3)
# box2 = add_obstacle([-1.5, 1.5, 0], "cube_small.urdf", scale=0.3)

# # Duck as decoration
# duck = add_obstacle([0, -2, 0], "duck_vhacd.urdf", scale=0.3)

# # Robot start position
# startPos = [-4, 0, 0.05]
# startOrientation = p.getQuaternionFromEuler([0, 0, 0])

# # Load TurtleBot3 Burger
# robotId = p.loadURDF("/home/sostika/catkin_ws/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.urdf",
#                      startPos, startOrientation)

# # Reset robot base and joints
# p.resetBasePositionAndOrientation(robotId, startPos, startOrientation)
# for j in range(p.getNumJoints(robotId)):
#     p.resetJointState(robotId, j, 0)

# # Simulation loop
# while True:
#     p.stepSimulation()
#     time.sleep(1./240.)



import pybullet as p
import pybullet_data

def create_wall(pos, size, color):
    collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
    visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=size, rgbaColor=color)
    return p.createMultiBody(baseMass=0,
                             baseCollisionShapeIndex=collision_shape,
                             baseVisualShapeIndex=visual_shape,
                             basePosition=pos)

def add_obstacle(pos, urdf_name, scale=1.0):
    return p.loadURDF(urdf_name, basePosition=pos, globalScaling=scale)

def create_world():
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)

    # Plane
    p.loadURDF("plane.urdf")

    # Example walls (you can add more)
    create_wall([5,0,0.5], [0.2,5,0.5], [0.8,0.8,0.8,1])
    create_wall([-5,0,0.5], [0.2,5,0.5], [0.8,0.8,0.8,1])
    create_wall([0,5,0.5], [5,0.2,0.5], [0.8,0.8,0.8,1])
    create_wall([0,-5,0.5], [5,0.2,0.5], [0.8,0.8,0.8,1])

    # Example obstacles
    add_obstacle([2,2,0], "table/table.urdf", scale=0.5)
    add_obstacle([-2,-2,0], "table/table.urdf", scale=0.5)

    # Robot
    startPos = [-4, 0, 0.05]
    startOrientation = p.getQuaternionFromEuler([0,0,0])
    robotId = p.loadURDF("/home/sostika/catkin_ws/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.urdf",
                         startPos, startOrientation)

    return physicsClient, robotId