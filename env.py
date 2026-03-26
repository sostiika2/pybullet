"""
turtlebot_env.py  –  Gymnasium environment for TurtleBot3 point-navigation.

Observation  (41-D float32)
───────────────────────────
  [0]      dist_norm    – distance to goal, normalised [0, 1]
  [1]      cos_angle    – cos of heading error  [-1, 1]
  [2]      sin_angle    – sin of heading error  [-1, 1]
  [3]      linear_norm  – forward speed, normalised  [0, 1]
  [4]      angular_norm – yaw rate, normalised  [-1, 1]
  [5:41]   lidar_norm   – 36-ray LiDAR, normalised [0, 1]

Action  (2-D float32, clipped to [-1, 1])
──────────────────────────────────────────
  [0]  -> [0,  MAX_LINEAR_VEL]          (always forward, no reversing)
  [1]  -> [-MAX_ANGULAR_VEL, MAX_ANGULAR_VEL]
"""

import math
import random

import numpy as np
import pybullet as p
import gymnasium as gym
from gymnasium import spaces

from pynullet_world import create_world
from sim import (
    create_goal_marker,
    reset_robot,
    reset_goal_marker,
    apply_velocity,
    get_lidar_scan,
    get_robot_state,
    check_collision,
    can_spawn_here,
)

# Change these 3 lines at the top
SIM_HZ    = 100    # was 240
POLICY_HZ = 100    # was 30
SIM_STEPS = 1      # was 8
class TurtleBotEnv(gym.Env):

    metadata = {"render_modes": ["human"]}

    # # Candidate spawn poses  [x, y, theta]
    # SPAWN_LIST = [
    #     ( 0.0, -1.0,  math.pi / 2),
    #     ( 0.0,  1.0,  math.pi / 2),
    #     ( 1.0,  0.0,  0.0),
    #     (-1.0,  0.0,  math.pi / 2),
    #     ( 2.0,  1.5, -math.pi / 4),
    # ]

    # # Candidate goal positions  [x, y]
    # GOAL_LIST = [
    #     ( 0.0, -1.2),
    #     ( 1.3, -0.5),
    #     (-1.2, -1.1),
    #     ( 1.0,  0.0),
    #     (-1.0,  0.0),
    #     ( 1.0,  1.3),
    #     (-1.0,  0.8),
    #     ( 1.4,  1.4),
    #     (-1.4,  1.0),
    # ]

    def __init__(self, render: bool = False):
        super().__init__()

        # Task parameters
        self.LIDAR_RANGE      = 3.5
        self.MAX_LINEAR_VEL   = 0.22      # m/s
        self.MAX_ANGULAR_VEL  = 2.0       # rad/s
        self.MAX_GOAL_DIST    = math.hypot(5.0, 5.0)   # rough arena diagonal

        self.GOAL_TOLERANCE   = 0.1     # m  – "arrived" radius
        self.MIN_SPAWN_DIST   = 1.5       # m  – minimum spawn <-> goal distance
        self.MAX_STEPS        = 6000

        # PyBullet world
        self.physicsClient, self.robotId, self.planeId = create_world(render=render)

        # Goal marker: created once here, teleported on every reset
        self.goal_marker_id = create_goal_marker(
            self.physicsClient, 0.0, 0.0
        )

        # Runtime state
        self.goal          = np.zeros(2, dtype=np.float32)
        self.x             = 0.0
        self.y             = 0.0
        self.yaw           = 0.0
        self.vel_linear    = 0.0
        self.vel_angular   = 0.0
        self.lidar         = np.full(36, self.LIDAR_RANGE, dtype=np.float32)
        self.prev_distance = None
        self.step_count    = 0
        self.total_episodes = 0

        # Spaces
        n_lidar = 36
        low  = np.array([0.0, -1.0, -1.0,  0.0, -1.0] + [0.0] * n_lidar, dtype=np.float32)
        high = np.array([1.0,  1.0,  1.0,  1.0,  1.0] + [1.0] * n_lidar, dtype=np.float32)

        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0], dtype=np.float32),
            high=np.array([ 1.0,  1.0], dtype=np.float32),
        )

    # -----------------------------------------------------------------------
    # Internal helpers
    # -----------------------------------------------------------------------

    def _sync_state(self) -> None:
        """Pull the latest robot pose, velocities and LiDAR into instance vars."""
        state = get_robot_state(self.physicsClient, self.robotId)

        self.x   = state["x"]
        self.y   = state["y"]
        self.yaw = state["yaw"]

        lin_vel = state["lin_vel"]
        ang_vel = state["ang_vel"]

        # Project world-frame velocity onto robot heading -> signed forward speed
        self.vel_linear  = (lin_vel[0] * math.cos(self.yaw)
                            + lin_vel[1] * math.sin(self.yaw))
        self.vel_angular = ang_vel[2]   # yaw rate

        self.lidar = get_lidar_scan(self.physicsClient, self.robotId)

    def _get_obs(self) -> np.ndarray:
        dx = self.goal[0] - self.x
        dy = self.goal[1] - self.y

        dist      = math.hypot(dx, dy)
        dist_norm = float(np.clip(dist / self.MAX_GOAL_DIST, 0.0, 1.0))

        heading_error = math.atan2(dy, dx) - self.yaw
        # Wrap to [-pi, pi]
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        linear_norm  = float(np.clip(self.vel_linear  / self.MAX_LINEAR_VEL,  0.0,  1.0))
        angular_norm = float(np.clip(self.vel_angular / self.MAX_ANGULAR_VEL, -1.0, 1.0))
        lidar_norm   = np.clip(self.lidar / self.LIDAR_RANGE, 0.0, 1.0).astype(np.float32)

        return np.concatenate([
            [dist_norm],
            [math.cos(heading_error)],
            [math.sin(heading_error)],
            [linear_norm],
            [angular_norm],
            lidar_norm,
        ]).astype(np.float32)

    def _apply_action(self, action: np.ndarray) -> None:
        """
        Convert normalised action -> real velocities, then advance physics.
          action[0] in [-1, 1]  ->  [0, MAX_LINEAR_VEL]   (no reversing)
          action[1] in [-1, 1]  ->  [-MAX_ANGULAR_VEL, MAX_ANGULAR_VEL]
        """
        linear  = ((float(action[0]) + 1.0) / 2.0) * self.MAX_LINEAR_VEL
        angular = float(action[1]) * self.MAX_ANGULAR_VEL

        apply_velocity(self.physicsClient, self.robotId, linear, angular)

        for _ in range(SIM_STEPS):
            p.stepSimulation(physicsClientId=self.physicsClient)

    
    def _get_obstacle_bias(self):
        """
        Gradually increase obstacle encounter rate
        as training progresses.
        """
        if self.total_episodes < 2000:
            return 0.1    # stage 1: mostly open space
        elif self.total_episodes < 6000:
            return 0.3    # stage 2: occasionally near obstacles
        elif self.total_episodes < 12000:
            return 0.5    # stage 3: half near obstacles
        else:
            return 0.7    # stage 4: mostly near obstacles


    # def _pick_spawn_and_goal(self):
    #     """
    #     Sample a (spawn, goal) pair where:
    #     - Euclidean distance >= MIN_SPAWN_DIST
    #     - Both positions are clear of obstacles
    #     Falls back to safe defaults if 200 attempts fail.
    #     """
    #     for _ in range(200):
    #         goal = np.array([
    #             random.uniform(-1.9, 1.9),
    #             random.uniform(-1.9, 1.9)
    #         ], dtype=np.float32)

    #         spawn = [
    #             random.uniform(-1.8, 1.8),
    #             random.uniform(-1.8, 1.8),
    #             random.uniform(-math.pi, math.pi)
    #         ]

    #         # Reject if spawn and goal are too close
    #         dist = math.hypot(goal[0] - spawn[0], goal[1] - spawn[1])
    #         if dist < self.MIN_SPAWN_DIST:
    #             continue

    #         # Reject if goal is inside an obstacle
    #         if not can_spawn_here(self.physicsClient, goal[0], goal[1], blocked_ids=set()):
    #             continue

    #         # Reject if spawn is inside an obstacle
    #         if not can_spawn_here(self.physicsClient, spawn[0], spawn[1], blocked_ids=set()):
    #             continue

    #         return spawn, goal

    #     # Safety fallback — guaranteed clear positions
    #     return [0.0, -2.0, math.pi / 2], np.array([0.0, 1.5], dtype=np.float32)


    def _pick_spawn_and_goal(self):
        obstacle_bias = self._get_obstacle_bias()

        OBSTACLE_POSITIONS = [
           ( 0.0,  0.0),( 1.8, -1.0),(-1.8, -1.0),(-1.8, -1.0),( 0.0, -1.5),( 1.5,  0.5)
        ]

        for _ in range(200):
            # Spawn position
            if random.random() < obstacle_bias:
                obs = random.choice(OBSTACLE_POSITIONS)
                angle = random.uniform(0, 2 * math.pi)
                d = random.uniform(0.35, 0.9)
                sx = float(np.clip(obs[0] + math.cos(angle)*d, -1.8, 1.8))
                sy = float(np.clip(obs[1] + math.sin(angle)*d, -1.8, 1.8))
            else:
                sx = random.uniform(-1.8, 1.8)
                sy = random.uniform(-1.8, 1.8)

            spawn = [sx, sy, random.uniform(-math.pi, math.pi)]

            # Goal position
            if random.random() < obstacle_bias:
                obs = random.choice(OBSTACLE_POSITIONS)
                angle = random.uniform(0, 2 * math.pi)
                d = random.uniform(0.35, 0.9)
                gx = float(np.clip(obs[0] + math.cos(angle)*d, -1.8, 1.8))
                gy = float(np.clip(obs[1] + math.sin(angle)*d, -1.8, 1.8))
            else:
                gx = random.uniform(-1.8, 1.8)
                gy = random.uniform(-1.8, 1.8)

            goal = np.array([gx, gy], dtype=np.float32)

            dist = math.hypot(goal[0]-spawn[0], goal[1]-spawn[1])
            if dist < self.MIN_SPAWN_DIST:
                continue
            if not can_spawn_here(self.physicsClient, goal[0], goal[1], blocked_ids=set()):
                continue
            if not can_spawn_here(self.physicsClient, spawn[0], spawn[1], blocked_ids=set()):
                continue

            return spawn, goal

        return [0.0, -2.0, math.pi/2], np.array([0.0, 1.5], dtype=np.float32)

    # -----------------------------------------------------------------------
    # Gymnasium API
    # -----------------------------------------------------------------------

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.total_episodes += 1

        # Stop motors before teleporting
        apply_velocity(self.physicsClient, self.robotId, 0.0, 0.0)

        # # Sample valid spawn + goal pair
        # spawn, self.goal = self._pick_spawn_and_goal()
        # spawn,self.goal = [1.0,2.0,0.0],[2.0,-2.0]

        # Teleport robot and move goal marker
        reset_robot(
            self.physicsClient, self.robotId,
            x=spawn[0], y=spawn[1], theta=spawn[2],
        )
        reset_goal_marker(
            self.physicsClient, self.goal_marker_id,
            x=self.goal[0], y=self.goal[1],
        )

        # Let physics settle
        for _ in range(10):
            p.stepSimulation(physicsClientId=self.physicsClient)

        self.step_count = 0
        self._sync_state()

        obs = self._get_obs()
        self.prev_distance = float(obs[0])   # normalised distance at episode start

        return obs, {}

    def step(self, action):
        self.step_count += 1

        self._apply_action(action)
        self._sync_state()

        obs = self._get_obs()

        # Derived quantities
        dist_norm = float(obs[0])
        dist_m    = dist_norm 
        lidar_min = float(np.min(self.lidar))          # metres

        # Collision detection
        collision = check_collision(
            self.physicsClient, self.robotId, self.planeId
        )

        # Reward
        # 1. Progress: positive when moving toward goal
        progress = self.prev_distance - dist_norm      # +ve -> closer
        reward   = 1.00 * progress

        # 2. Proximity penalty: smooth penalty when < 0.3 m from obstacle
        SAFE_DIST = 0.3
        if lidar_min < SAFE_DIST:
            reward += -0.2 * (1.0 - lidar_min / SAFE_DIST) ** 2

        # 3. Small time penalty to encourage efficiency
        reward += -0.005

        # 4. Terminal events override accumulated reward
        reached_goal = dist_m < self.GOAL_TOLERANCE

        if collision:
            reward = -20.0
        elif reached_goal:
            reward = 20.0

        terminated = collision or reached_goal
        truncated  = self.step_count >= self.MAX_STEPS

        self.prev_distance = dist_norm

        return obs, float(reward), terminated, truncated, {}

    def close(self):
        """Clean shutdown: stop motors and disconnect PyBullet."""
        apply_velocity(self.physicsClient, self.robotId, 0.0, 0.0)
        p.disconnect(physicsClientId=self.physicsClient)