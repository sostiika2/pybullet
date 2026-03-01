import pybullet as p
import pybullet_data
import time
import numpy as np

# ----------------------
# Connect to PyBullet
# ----------------------
physicsClient = p.connect(p.GUI)  # Use GUI to visualize
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# ----------------------
# Load Floor
# ----------------------
plane = p.loadURDF("plane.urdf")

# ----------------------
# Create simple house walls
# ----------------------
wall_height = 1.0
wall_thickness = 0.1
wall_color = [0.8, 0.8, 0.8, 1]

def create_wall(pos, size):
    col_box = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
    vis_box = p.createVisualShape(p.GEOM_BOX, halfExtents=size, rgbaColor=wall_color)
    return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col_box,
                             baseVisualShapeIndex=vis_box, basePosition=pos)

# Outer walls
create_wall([2, 0, wall_height/2], [wall_thickness, 2, wall_height/2])
create_wall([-2, 0, wall_height/2], [wall_thickness, 2, wall_height/2])
create_wall([0, 2, wall_height/2], [2, wall_thickness, wall_height/2])
create_wall([0, -2, wall_height/2], [2, wall_thickness, wall_height/2])

# Inner wall (like a room)
create_wall([0, 0, wall_height/2], [wall_thickness, 1, wall_height/2])

# ----------------------
# Add obstacles (boxes)
# ----------------------
obstacles = []
for i in range(5):
    x = np.random.uniform(-1.5, 1.5)
    y = np.random.uniform(-1.5, 1.5)
    size = [0.2, 0.2, 0.2]
    col_box = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
    vis_box = p.createVisualShape(p.GEOM_BOX, halfExtents=size, rgbaColor=[1,0,0,1])
    obs = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col_box,
                            baseVisualShapeIndex=vis_box, basePosition=[x, y, 0.2])
    obstacles.append(obs)

# ----------------------
# Load a robot (TurtleBot3 URDF)
# ----------------------
robot_path = "r2d2.urdf"  # placeholder robot, can be replaced with TurtleBot3 URDF
robot = p.loadURDF(robot_path, [0, 0, 0.1])

# ----------------------
# Simple manual control
# ----------------------
linear_velocity = 0.5  # meters per second
angular_velocity = 1.0  # radians per second
dt = 1./240.  # simulation step

print("Controls: w/s = forward/backward, a/d = turn, q = quit")

keys = {}
while True:
    p.stepSimulation()
    time.sleep(dt)
    
    # get key events
    keys_events = p.getKeyboardEvents()
    for k in keys_events:
        keys[k] = keys_events[k]
    
    # Get robot position and orientation
    pos, orn = p.getBasePositionAndOrientation(robot)
    euler = p.getEulerFromQuaternion(orn)
    yaw = euler[2]
    
    dx = dy = dyaw = 0
    
    if ord('w') in keys and keys[ord('w')] & p.KEY_IS_DOWN:
        dx = linear_velocity * dt * np.cos(yaw)
        dy = linear_velocity * dt * np.sin(yaw)
    if ord('s') in keys and keys[ord('s')] & p.KEY_IS_DOWN:
        dx = -linear_velocity * dt * np.cos(yaw)
        dy = -linear_velocity * dt * np.sin(yaw)
    if ord('a') in keys and keys[ord('a')] & p.KEY_IS_DOWN:
        dyaw = angular_velocity * dt
    if ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN:
        dyaw = -angular_velocity * dt
    if ord('q') in keys and keys[ord('q')] & p.KEY_IS_DOWN:
        break
    
    # Update robot position
    new_pos = [pos[0]+dx, pos[1]+dy, pos[2]]
    new_yaw = yaw + dyaw
    new_orn = p.getQuaternionFromEuler([0,0,new_yaw])
    p.resetBasePositionAndOrientation(robot, new_pos, new_orn)

p.disconnect()