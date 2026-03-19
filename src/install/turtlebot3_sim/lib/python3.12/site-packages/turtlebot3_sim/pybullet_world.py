# import pybullet as p
# import pybullet_data

# import math
# # ---------- WALL ----------
# def create_wall(pos, size, color=[0.85,0.85,0.85,1]):
#     collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
#     visual = p.createVisualShape(p.GEOM_BOX, halfExtents=size, rgbaColor=color)

#     return p.createMultiBody(
#         baseMass=0,
#         baseCollisionShapeIndex=collision,
#         baseVisualShapeIndex=visual,
#         basePosition=pos
#     )

# def create_trash_bin(pos, radius=0.1, height=0.3, color=[0,0,0,1]):
#     collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
#     visual = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=color)
#     bin_id = p.createMultiBody(
#         baseMass=0,
#         baseCollisionShapeIndex=collision,
#         baseVisualShapeIndex=visual,
#         basePosition=[pos[0], pos[1], height/2]
#     )
#     return bin_id

# # ---------- BOX OBSTACLE ----------
# def create_box(pos, size=0.25):
#     collision = p.createCollisionShape(
#         p.GEOM_BOX,
#         halfExtents=[size,size,size]
#     )

#     visual = p.createVisualShape(
#         p.GEOM_BOX,
#         halfExtents=[size,size,size],
#         rgbaColor=[0.6,0.4,0.2,1]
#     )

#     return p.createMultiBody(
#         baseMass=0,
#         baseCollisionShapeIndex=collision,
#         baseVisualShapeIndex=visual,
#         basePosition=[pos[0],pos[1],size]
#     )


# # ---------- PILLAR ----------
# def create_pillar(pos, radius=0.2, height=1.0):
#     collision = p.createCollisionShape(
#         p.GEOM_CYLINDER,
#         radius=radius,
#         height=height
#     )

#     visual = p.createVisualShape(
#         p.GEOM_CYLINDER,
#         radius=radius,
#         length=height,
#         rgbaColor=[0.4,0.4,0.4,1]
#     )

#     return p.createMultiBody(
#         baseMass=0,
#         baseCollisionShapeIndex=collision,
#         baseVisualShapeIndex=visual,
#         basePosition=[pos[0],pos[1],height/2]
#     )


# # ---------- VISUAL GOAL (NO COLLISION) ----------
# def create_goal_marker(pos):

#     visual = p.createVisualShape(
#         p.GEOM_CYLINDER,
#         radius=0.1,
#         length=0.02,
#         rgbaColor=[1,0,0,1]   # bright red
#     )

#     return p.createMultiBody(
#         baseMass=0,
#         baseCollisionShapeIndex=-1,   # IMPORTANT: no collision
#         baseVisualShapeIndex=visual,
#         basePosition=[pos[0],pos[1],0.01]
#     )


# def create_world():

#     physicsClient = p.connect(p.GUI)  # Use p.GUI for visualization, p.DIRECT for headless
#     p.setAdditionalSearchPath(pybullet_data.getDataPath())
#     p.setGravity(0, 0, -10)

#     # ---------- FLOOR ----------
#     planeId = p.loadURDF("plane.urdf")

#     # ---------- OUTER WALLS ----------
#     # create_wall([5, 0, 0.5], [0.2, 5, 0.5])
#     # create_wall([-5, 0, 0.5], [0.2, 5, 0.5])
#     # create_wall([0, 5, 0.5], [5, 0.2, 0.5])
#     # create_wall([0, -5, 0.5], [5, 0.2, 0.5])

#     #   # outer walls only — simple box arena
#     # create_wall([ 2.5,  0.0, 0.5], [0.1, 2.5, 0.5])  # right
#     # create_wall([-2.5,  0.0, 0.5], [0.1, 2.5, 0.5])  # left
#     # create_wall([ 0.0,  2.5, 0.5], [2.5, 0.1, 0.5])  # top
#     # create_wall([ 0.0, -2.5, 0.5], [2.5, 0.1, 0.5])  # bottom

#     create_wall([ 2.5,  0.0, 0.5], [0.1, 3.0, 0.5])  # right
#     create_wall([-2.5,  0.0, 0.5], [0.1, 3.0, 0.5])  # left
#     create_wall([ 0.0,  2.5, 0.5], [3.0, 0.1, 0.5])  # top
#     create_wall([ 0.0, -2.5, 0.5], [3.0, 0.1, 0.5])  # bottom

#     # small obstacles — not too big, not too small
#     # # pillars are best — round so robot can go around any side
#     # create_pillar([ 1.0,  1.0], radius=0.15, height=1.0)
#     # create_pillar([-1.0,  1.0], radius=0.15, height=1.0)
#     # create_pillar([ 1.0, -1.0], radius=0.15, height=1.0)
#     # create_pillar([-1.0, -1.0], radius=0.15, height=1.0)
#     # create_pillar([ 0.0,  0.0], radius=0.15, height=1.0)  # center


#     create_pillar([ 1.0,  1.0], radius=0.14, height=0.8)
#     create_pillar([-1.0,  1.0], radius=0.14, height=0.8)
#     create_pillar([ 1.0, -1.0], radius=0.14, height=0.8)
#     create_pillar([-1.0, -1.0], radius=0.14, height=0.8)


#     startPos = [0.0, -2.0, 0.01]

#     startOrientation = p.getQuaternionFromEuler([0, 0, math.pi/2]) 

#     robotId = p.loadURDF(
#         "/home/sostika/catkin_ws/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.urdf",
#         startPos,
#         startOrientation
#     )


#     return physicsClient, robotId, planeId

# if __name__ == "__main__":
#     physicsClient, robotId, planeId = create_world()

#     # Keep simulation running
#     while True:
#         p.stepSimulation()



# # # ---------- MAIN LOOP ----------
# if __name__ == "__main__":
#     physicsClient, robotId, planeId = create_world()
#     p.resetDebugVisualizerCamera(cameraDistance=8, cameraYaw=50, cameraPitch=-35, cameraTargetPosition=[0,0,0])

#     while True:
#         p.stepSimulation()
#         time.sleep(1./240)


import pybullet as p
import pybullet_data

import math
# ---------- WALL ----------
def create_wall(pos, size, color=[0.85,0.85,0.85,1]):
    collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
    visual = p.createVisualShape(p.GEOM_BOX, halfExtents=size, rgbaColor=color)

    return p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=pos
    )


# ---------- BOX OBSTACLE ----------
def create_box(pos, size=0.25):
    collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[size,size,size]
    )

    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[size,size,size],
        rgbaColor=[0.6,0.4,0.2,1]
    )

    return p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=[pos[0],pos[1],size]
    )


# ---------- PILLAR ----------
def create_pillar(pos, radius=0.2, height=1.0):
    collision = p.createCollisionShape(
        p.GEOM_CYLINDER,
        radius=radius,
        height=height
    )

    visual = p.createVisualShape(
        p.GEOM_CYLINDER,
        radius=radius,
        length=height,
        rgbaColor=[0.4,0.4,0.4,1]
    )

    return p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=[pos[0],pos[1],height/2]
    )


# ---------- VISUAL GOAL (NO COLLISION) ----------
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

    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)

    # ---------- FLOOR ----------
    planeId = p.loadURDF("plane.urdf")

    # ---------- OUTER WALLS ----------
    create_wall([5, 0, 0.5], [0.2, 5, 0.5])
    create_wall([-5, 0, 0.5], [0.2, 5, 0.5])
    create_wall([0, 5, 0.5], [5, 0.2, 0.5])
    create_wall([0, -5, 0.5], [5, 0.2, 0.5])

    # ---------- INNER CORRIDOR WALLS ----------
    # Create a rectangular loop corridor
    create_wall([0, 2, 0.5], [4.0, 0.1, 0.5])   # top horizontal
    create_wall([0, -2, 0.5], [4.0, 0.1, 0.5])  # bottom horizontal
    create_wall([-2, 0, 0.5], [0.1, 2.0, 0.5])  # left vertical
    create_wall([2, 0, 0.5], [0.1, 2.0, 0.5])   # right vertical

    # Optional: small inner obstacles in corridor
    create_box([1, 1], size=0.25)
    create_box([-1, -1], size=0.25)
    create_pillar([0, 0], radius=0.2, height=1.0)

    # ---------- GOAL ----------
    goal_position = [0, 1]  # place goal near corridor end
    create_goal_marker(goal_position)

    # ---------- ROBOT ----------
    startPos = [0, -1.6, 0.01]  # start at opposite side of corridor
    startOrientation = p.getQuaternionFromEuler([0, 0, math.pi/2]) 

    robotId = p.loadURDF(
        "/home/sostika/catkin_ws/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.urdf",
        startPos,
        startOrientation
    )

    return physicsClient, robotId, planeId