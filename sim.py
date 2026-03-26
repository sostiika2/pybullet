"""
sim.py  –  Low-level robot utilities for TurtleBot3 in PyBullet.

All functions are stateless and require an explicit physicsClientId
so they work correctly across any number of parallel environments.
"""

import math
import numpy as np
import pybullet as p


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  Hardware constants                                                      ║
# ╚══════════════════════════════════════════════════════════════════════════╝

WHEEL_RADIUS        = 0.033    # m
WHEEL_BASE          = 0.160    # m   (left ↔ right wheel centre distance)
LEFT_WHEEL_JOINT    = 1
RIGHT_WHEEL_JOINT   = 2

# ── LiDAR defaults ─────────────────────────────────────────────────────────
DEFAULT_NUM_RAYS    = 36
DEFAULT_LIDAR_RANGE = 3.5      # m
LIDAR_RANGE_MIN     = 0.12     # m   (inside-robot blind zone)
LIDAR_HEIGHT_OFFSET = 0.10     # m   above base-link origin


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  1. Visual markers                                                       ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def create_goal_marker(
    client: int,
    x: float,
    y: float,
    z: float = 0.01,
    radius: float = 0.10,
    colour: tuple = (1.0, 0.0, 0.0, 1.0),
) -> int:
    """
    Create a flat cylinder goal marker with NO collision shape.
    Returns the body id.
    """
    visual = p.createVisualShape(
        p.GEOM_CYLINDER,
        radius=radius,
        length=0.02,
        rgbaColor=list(colour),
        physicsClientId=client,
    )
    return p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=-1,      # no collision – robot can drive over it
        baseVisualShapeIndex=visual,
        basePosition=[x, y, z],
        physicsClientId=client,
    )


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  2. Reset helpers                                                        ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def reset_robot(
    client: int,
    robot_id: int,
    x: float = 0.0,
    y: float = -2.0,
    theta: float = math.pi / 2,
    stop_wheels: bool = True,
) -> None:
    """Teleport the robot to (x, y, theta) and zero all velocities."""
    pos = [x, y, 0.01]
    orn = p.getQuaternionFromEuler([0.0, 0.0, theta])

    p.resetBasePositionAndOrientation(robot_id, pos, orn, physicsClientId=client)
    p.resetBaseVelocity(robot_id, [0, 0, 0], [0, 0, 0], physicsClientId=client)

    num_joints = p.getNumJoints(robot_id, physicsClientId=client)
    for j in range(num_joints):
        p.resetJointState(
            robot_id, j,
            targetValue=0.0, targetVelocity=0.0,
            physicsClientId=client,
        )

    if stop_wheels:
        for joint in (LEFT_WHEEL_JOINT, RIGHT_WHEEL_JOINT):
            p.setJointMotorControl2(
                robot_id, joint,
                p.VELOCITY_CONTROL,
                targetVelocity=0.0,
                force=10.0,
                physicsClientId=client,
            )


def reset_goal_marker(
    client: int,
    marker_id: int,
    x: float,
    y: float,
    z: float = 0.01,
) -> None:
    """Teleport an existing goal marker to a new (x, y) position."""
    if marker_id is None or marker_id < 0:
        return
    p.resetBasePositionAndOrientation(
        marker_id,
        [x, y, z],
        [0, 0, 0, 1],
        physicsClientId=client,
    )


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  3. Actuation                                                            ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def apply_velocity(
    client: int,
    robot_id: int,
    linear: float,
    angular: float,
    max_force: float = 10.0,
) -> None:
    """
    Convert (linear m/s, angular rad/s) → individual wheel speeds
    and send them to the velocity controllers.
    """
    v_left  = (linear - angular * WHEEL_BASE / 2.0) / WHEEL_RADIUS
    v_right = (linear + angular * WHEEL_BASE / 2.0) / WHEEL_RADIUS

    for joint, omega in (
        (LEFT_WHEEL_JOINT,  v_left),
        (RIGHT_WHEEL_JOINT, v_right),
    ):
        p.setJointMotorControl2(
            robot_id, joint,
            p.VELOCITY_CONTROL,
            targetVelocity=omega,
            force=max_force,
            physicsClientId=client,
        )


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  4. Sensing                                                              ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def get_lidar_scan(
    client: int,
    robot_id: int,
    num_rays: int = DEFAULT_NUM_RAYS,
    lidar_range: float = DEFAULT_LIDAR_RANGE,
) -> np.ndarray:
    """
    Fire `num_rays` rays in a full circle and return distances (m).
    Rays that miss any object return `lidar_range`.
    Rays that hit within LIDAR_RANGE_MIN are clamped upward.
    """
    pos, orn = p.getBasePositionAndOrientation(robot_id, physicsClientId=client)
    yaw      = p.getEulerFromQuaternion(orn)[2]
    origin_z = pos[2] + LIDAR_HEIGHT_OFFSET

    ray_from, ray_to = [], []
    for i in range(num_rays):
        angle = yaw + i * (2.0 * math.pi / num_rays)
        ray_from.append([pos[0], pos[1], origin_z])
        ray_to.append([
            pos[0] + lidar_range * math.cos(angle),
            pos[1] + lidar_range * math.sin(angle),
            origin_z,
        ])

    results = p.rayTestBatch(ray_from, ray_to, physicsClientId=client)

    ranges = []
    for r in results:
        object_uid   = r[0]     # -1 → no hit
        hit_fraction = r[2]     # 0.0 – 1.0

        if object_uid == -1:
            ranges.append(lidar_range)
        else:
            dist = hit_fraction * lidar_range
            ranges.append(float(np.clip(dist, LIDAR_RANGE_MIN, lidar_range)))

    return np.array(ranges, dtype=np.float32)


def get_robot_state(client: int, robot_id: int) -> dict:
    """
    Return a dict with position, orientation (euler + quat) and velocities.

    Keys: x, y, z, roll, pitch, yaw, quat, lin_vel, ang_vel
    """
    pos, orn         = p.getBasePositionAndOrientation(robot_id, physicsClientId=client)
    lin_vel, ang_vel = p.getBaseVelocity(robot_id, physicsClientId=client)
    roll, pitch, yaw = p.getEulerFromQuaternion(orn)

    return {
        "x":       pos[0],
        "y":       pos[1],
        "z":       pos[2],
        "roll":    roll,
        "pitch":   pitch,
        "yaw":     yaw,
        "quat":    orn,          # (x, y, z, w)
        "lin_vel": lin_vel,      # (vx, vy, vz) world frame
        "ang_vel": ang_vel,      # (wx, wy, wz) world frame
    }


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  5. Collision helpers                                                    ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def check_collision(
    client: int,
    robot_id: int,
    plane_id: int,
    ignore_ids: set = None,
    force_threshold: float = 0.05,
) -> bool:
    """
    Return True if the robot is in contact with any body other than
    the ground plane (and anything in ignore_ids).
    """
    ignored = {plane_id}
    if ignore_ids:
        ignored.update(ignore_ids)

    contacts = p.getContactPoints(bodyA=robot_id, physicsClientId=client)
    if not contacts:
        return False

    for c in contacts:
        other_body   = c[2]    # body B uid
        normal_force = c[9]    # contact normal force magnitude
        if other_body not in ignored and normal_force > force_threshold:
            return True

    return False


def get_contact_bodies(
    client: int,
    robot_id: int,
    plane_id: int,
) -> list:
    """Return a list of unique body ids currently touching the robot (excluding plane)."""
    contacts = p.getContactPoints(bodyA=robot_id, physicsClientId=client)
    if not contacts:
        return []
    return list({c[2] for c in contacts if c[2] != plane_id})


def can_spawn_here(
    client: int,
    x: float,
    y: float,
    blocked_ids: set,
    radius: float = 0.20,
    z: float = 0.25,
) -> bool:
    """
    Cast 8 short rays outward from (x, y, z) to check whether
    the position is clear of walls and obstacles.
    Returns True if all rays are clear.
    """
    origin = [x, y, z]
    ray_to = [
        [
            x + radius * math.cos(2 * math.pi * i / 8),
            y + radius * math.sin(2 * math.pi * i / 8),
            z,
        ]
        for i in range(8)
    ]

    results = p.rayTestBatch([origin] * 8, ray_to, physicsClientId=client)

    for hit_id, _, fraction, _, _ in results:
        if hit_id != -1 and fraction < 1.0:
            return False    # something is too close

    return True