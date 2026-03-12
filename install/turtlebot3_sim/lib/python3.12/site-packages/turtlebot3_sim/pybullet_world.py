import pybullet as p
import pybullet_data

# ---------- WALL ----------
def create_wall(pos, size, color):
    collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
    visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=size, rgbaColor=color)

    return p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=pos
    )


# ---------- URDF OBSTACLE ----------
def add_obstacle(pos, urdf_name, scale=1.0):
    return p.loadURDF(urdf_name, basePosition=pos, globalScaling=scale)


# ---------- SMALL BOX ----------
def create_fixed_box(pos, size=0.1):
    collision_shape = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[size, size, size]
    )

    visual_shape = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[size, size, size],
        rgbaColor=[0.6, 0.3, 0.1, 1]
    )

    return p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=[pos[0], pos[1], size]
    )


# ---------- PILLAR ----------
def create_pillar(pos, radius=0.15, height=1.0, color=[0.4,0.4,0.4,1]):
    collision_shape = p.createCollisionShape(
        p.GEOM_CYLINDER,
        radius=radius,
        height=height
    )

    visual_shape = p.createVisualShape(
        p.GEOM_CYLINDER,
        radius=radius,
        length=height,
        rgbaColor=color
    )

    return p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=[pos[0], pos[1], height/2]
    )


# ---------- WORLD ----------
def create_world():

    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)

    # Plane
    planeId = p.loadURDF("plane.urdf")

    # ---------- Boundary walls ----------
    create_wall([5,0,0.5], [0.2,5,0.5], [0.8,0.8,0.8,1])
    create_wall([-5,0,0.5], [0.2,5,0.5], [0.8,0.8,0.8,1])
    create_wall([0,5,0.5], [5,0.2,0.5], [0.8,0.8,0.8,1])
    create_wall([0,-5,0.5], [5,0.2,0.5], [0.8,0.8,0.8,1])


    # Table near the goal (forces curved approach)
    add_obstacle([3.0, 3.0, 0], "table/table.urdf", scale=0.3)

    # Pillars that block straight path
    create_pillar([0.6, 1.2])
    create_pillar([1.8, 1.5])

    # Small clutter boxes
    create_fixed_box([1.5, 0.8])

    create_fixed_box([-0.5, 1.5])


    # ---------- Robot ----------
    startPos = [1,2,0.001]
    startOrientation = p.getQuaternionFromEuler([0,0,0])

    robotId = p.loadURDF(
        "/home/sostika/catkin_ws/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.urdf",
        startPos,
        startOrientation
    )

    return physicsClient, robotId, planeId