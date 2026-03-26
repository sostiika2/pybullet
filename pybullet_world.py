"""
mypy.py  –  PyBullet world builder for TurtleBot3 navigation.

Every PyBullet call passes physicsClientId so multiple parallel
training instances never accidentally share state.
"""

import math
import pybullet as p
import pybullet_data


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  Primitive builders  (all require an explicit client handle)            ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def create_wall(
    client: int,
    pos: list,
    size: list,
    color: list = None,
) -> int:
    if color is None:
        color = [0.85, 0.85, 0.85, 1.0]

    collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=size,
        physicsClientId=client,
    )
    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=size,
        rgbaColor=color,
        physicsClientId=client,
    )
    return p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=pos,
        physicsClientId=client,
    )


def create_box(
    client: int,
    pos: list,
    size: float = 0.1,
) -> int:
    half = [size, size, size * 2]

    collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=half,
        physicsClientId=client,
    )
    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=half,
        rgbaColor=[0.6, 0.4, 0.2, 1.0],
        physicsClientId=client,
    )
    return p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=[pos[0], pos[1], size],
        physicsClientId=client,
    )


def create_pillar(
    client: int,
    pos: list,
    radius: float = 0.2,
    height: float = 1.0,
) -> int:
    collision = p.createCollisionShape(
        p.GEOM_CYLINDER,
        radius=radius,
        height=height,
        physicsClientId=client,
    )
    visual = p.createVisualShape(
        p.GEOM_CYLINDER,
        radius=radius,
        length=height,
        rgbaColor=[0.4, 0.4, 0.4, 1.0],
        physicsClientId=client,
    )
    return p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=[pos[0], pos[1], height / 2],
        physicsClientId=client,
    )


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  World factory                                                           ║
# ╚══════════════════════════════════════════════════════════════════════════╝

URDF_PATH = (
    "/home/sostika/catkin_ws/turtlebot3/"
    "turtlebot3_description/urdf/turtlebot3_burger.urdf"
)


def create_world(render: bool = False):
    """
    Spawn a complete arena and return (physicsClient, robotId, planeId).

    Parameters
    ----------
    render : bool
        True  → open a GUI window  (use for evaluation / manual testing)
        False → headless DIRECT mode  (use for training)
    """
    client = p.connect(p.GUI if render else p.DIRECT)
    p.setTimeStep(0.01, physicsClientId=client)
    p.setPhysicsEngineParameter(numSolverIterations=50, physicsClientId=client)

    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
    p.setGravity(0, 0, -9.8, physicsClientId=client)

    # ── Floor ──────────────────────────────────────────────────────────────
    plane_id = p.loadURDF("plane.urdf", physicsClientId=client)

    # ── Perimeter walls ────────────────────────────────────────────────────
    create_wall(client, [ 2.5,  0.0, 0.1], [0.06, 3.0, 0.5])   # right
    create_wall(client, [-2.5,  0.0, 0.1], [0.06, 3.0, 0.5])   # left
    create_wall(client, [ 0.0,  2.5, 0.1], [3.0, 0.06, 0.5])   # top
    create_wall(client, [ 0.0, -2.5, 0.1], [3.0, 0.06, 0.5])   # bottom

    # ── Interior obstacles ─────────────────────────────────────────────────
    # create_pillar(client, [ 1.0,  1.0], radius=0.14, height=0.8)
    # create_pillar(client, [-1.0,  1.0], radius=0.14, height=0.8)
    # create_box(client, [ 1.2, -1.0])
    # create_box(client, [-1.2, -1.0])
    # create_box(client, [ 1.0,  1.0])
    create_pillar(client, [ 0.0,  0.0], radius=0.14, height=0.8)  # center
    create_pillar(client, [ 1.8, -1.0], radius=0.14, height=0.8)
    create_pillar(client, [-1.8, -1.0], radius=0.14, height=0.8)
    create_box(client,    [ 0.0,  1.5])
    create_box(client,    [ 0.0, -1.5])
    create_box(client,    [-1.5,  0.5])
    create_box(client,    [ 1.5,  0.5])

    

     
    # ── Robot ──────────────────────────────────────────────────────────────
    robot_id = p.loadURDF(
        URDF_PATH,
        [0.0, 0.2, 0.01],
        p.getQuaternionFromEuler([0.0, 0.0, math.pi / 2]),
        physicsClientId=client,
    )

    # ── Camera (GUI only) ──────────────────────────────────────────────────
    if render:
        p.resetDebugVisualizerCamera(
            cameraDistance=6,
            cameraYaw=0,
            cameraPitch=-45,
            cameraTargetPosition=[0, 0, 0],
            physicsClientId=client,
        )

    return client, robot_id, plane_id


# ── Quick smoke-test ───────────────────────────────────────────────────────
if __name__ == "__main__":
    import time

    client, robot_id, plane_id = create_world(render=True)
    print(f"Connected: client={client}, robot={robot_id}, plane={plane_id}")

    while True:          # 10 seconds @ 240 Hz
        p.stepSimulation(physicsClientId=client)
        time.sleep(1.0 / 240.0)

    p.disconnect(physicsClientId=client)