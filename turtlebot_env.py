import pybullet as p
import pybullet_data
import numpy as np

from your_world_file import create_world  # this is your create_world() function

class TurtleBotEnv:
    def __init__(self, gui=True):
        if gui:
            self.physicsClient, self.robotId = create_world()
        else:
            self.physicsClient = p.connect(p.DIRECT)
            self.robotId = None
        # RL parameters
        self.startPos = [-4, 0, 0.001]
        self.startOrientation = p.getQuaternionFromEuler([0,0,0])
        self.goal = [4, 0]  # example goal

    def reset(self):
        # Reset robot position
        p.resetBasePositionAndOrientation(self.robotId, self.startPos, self.startOrientation)
        # Reset joints
        for j in range(p.getNumJoints(self.robotId)):
            p.resetJointState(self.robotId, j, 0)
        return self.get_state()

    def step(self, action):
        # Apply action: action = [linear_velocity, angular_velocity]
        linear, angular = action
        wheel_velocity = [linear - angular, linear + angular]
        p.setJointMotorControl2(self.robotId, 1, p.VELOCITY_CONTROL, targetVelocity=wheel_velocity[0])
        p.setJointMotorControl2(self.robotId, 2, p.VELOCITY_CONTROL, targetVelocity=wheel_velocity[1])

        # Step simulation for RL timestep (e.g., 0.1s)
        steps = int(0.1 / (1/240))  # 0.1 s / 1/240 s per physics step
        for _ in range(steps):
            p.stepSimulation()

        # Return updated state, reward, done
        state = self.get_state()
        reward = self.compute_reward()
        done = self.check_done()
        return state, reward, done, {}

    def get_state(self):
        pos, orn = p.getBasePositionAndOrientation(self.robotId)
        yaw = p.getEulerFromQuaternion(orn)[2]
        return np.array([pos[0], pos[1], yaw, self.goal[0], self.goal[1]], dtype=np.float32)

    def compute_reward(self):
        pos, _ = p.getBasePositionAndOrientation(self.robotId)
        dist = np.linalg.norm(np.array(pos[:2]) - np.array(self.goal[:2]))
        return -dist  # negative distance to goal

    def check_done(self):
        pos, _ = p.getBasePositionAndOrientation(self.robotId)
        dist = np.linalg.norm(np.array(pos[:2]) - np.array(self.goal[:2]))
        return dist < 0.1  # goal reached