def is_position_free(x, y, z=0.4, robot_id=None):
    """
    Checks if the given position is free of obstacles or walls.
    """
    # Create a tiny temporary collision shape for the goal
    temp_goal = p.createCollisionShape(p.GEOM_SPHERE, radius=0.1)
    temp_id = p.createMultiBody(baseCollisionShapeIndex=temp_goal, basePosition=[x, y, z])

    collision = False
    for obj_id in range(p.getNumBodies()):
        if obj_id == temp_id or obj_id == robot_id:
            continue
        # Check if this temporary goal is too close to any object
        pts = p.getClosestPoints(bodyA=temp_id, bodyB=obj_id, distance=0.15)
        if len(pts) > 0:
            collision = True
            break

    # Remove the temporary goal
    p.removeBody(temp_id)

    return not collision