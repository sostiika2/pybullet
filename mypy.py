import pybullet as p
import pybullet_data

import math

def create_goal_marker(pos):

    visual = p.createVisualShape(
        p.GEOM_CYLINDER,
        radius=0.1,
        length=0.02,
        rgbaColor=[1,0,0,1]   # bright red
    )

    return p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=-1,   # IMPORTANT: no collision
        baseVisualShapeIndex=visual,
        basePosition=[pos[0],pos[1],0.01]
    )


def create_world():

    physicsClient = p.connect(p.GUI)  # Use p.GUI for visualization, p.DIRECT for headless
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)

    # ---------- FLOOR ----------
    planeId = p.loadURDF("plane.urdf")



    startPos = [0.0, -2.0, 0.01]

    startOrientation = p.getQuaternionFromEuler([0, 0, math.pi/2]) 

    robotId = p.loadURDF(
        "/home/sostika/catkin_ws/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.urdf",
        startPos,
        startOrientation
    )


    return physicsClient, robotId, planeId

if __name__ == "__main__":
    physicsClient, robotId, planeId = create_world()

    # Keep simulation running
    while True:
        p.stepSimulation()



# # ---------- MAIN LOOP ----------
if __name__ == "__main__":
    physicsClient, robotId, planeId = create_world()
    p.resetDebugVisualizerCamera(cameraDistance=8, cameraYaw=50, cameraPitch=-35, cameraTargetPosition=[0,0,0])

    while True:
        p.stepSimulation()
        time.sleep(1./240)