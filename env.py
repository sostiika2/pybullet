import pybullet as p
import pybullet_data
import numpy as np
import time

# ----------------------------
# Connect to PyBullet
# ----------------------------
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -10)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load plane
planeId = p.loadURDF("plane.urdf")

# ----------------------------
# Environment Parameters
# ----------------------------
env_size = 10          # Half-length of environment (x and y)
wall_height = 1.0
wall_thickness = 0.2
wall_color = [0.8, 0.8, 0.8, 1]
obstacle_count = 20    # Number of obstacles
obstacle_size = 0.3    # Half-size of obstacle cube

# ----------------------------
# Helper functions
# ----------------------------
def create_wall(pos, size, color):
    col_box = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
    vis_box = p.createVisualShape(p.GEOM_BOX, halfExtents=size, rgbaColor=color)
    return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col_box,
                             baseVisualShapeIndex=vis_box, basePosition=pos)

def create_outer_walls():
    # Left and Right walls
    create_wall([env_size, 0, wall_height/2], [wall_thickness, env_size, wall_height/2], wall_color)
    create_wall([-env_size, 0, wall_height/2], [wall_thickness, env_size, wall_height/2], wall_color)
    # Top and Bottom walls
    create_wall([0, env_size, wall_height/2], [env_size, wall_thickness, wall_height/2], wall_color)
    create_wall([0, -env_size, wall_height/2], [env_size, wall_thickness, wall_height/2], wall_color)

def create_obstacles():
    obstacles = []
    for i in range(obstacle_count):
        x = np.random.uniform(-env_size+1, env_size-1)
        y = np.random.uniform(-env_size+1, env_size-1)
        col_box = p.createCollisionShape(p.GEOM_BOX, halfExtents=[obstacle_size]*3)
        vis_box = p.createVisualShape(p.GEOM_BOX, halfExtents=[obstacle_size]*3, rgbaColor=[1, 0, 0, 1])
        obs = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col_box,
                                baseVisualShapeIndex=vis_box, basePosition=[x, y, obstacle_size])
        obstacles.append(obs)
    return obstacles

def create_environment():
    create_outer_walls()
    # Removed inner walls
    obstacles = create_obstacles()
    return obstacles

# ----------------------------
# Load environment
# ----------------------------
obstacles = create_environment()

# ----------------------------
# Run simulation
# ----------------------------
while True:
    p.stepSimulation()
    time.sleep(1./240.)

    