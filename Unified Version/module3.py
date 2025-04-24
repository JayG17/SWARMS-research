import imports as swarms
from typing import List

# MODULE 3: Sphere Functions


def getGateSphereData(gateDataObjs: List[swarms.GateObject]) -> List[swarms.GateSphere]:
    """
    Description:
    - Creates bounding spheres around each gate to define proximity zones.
    - Each sphere is represented by min/max boundaries in x, y, and z.

    Inputs:
    - gateDataObjs (List[GateObject]): List of gate data objects containing x, y, z positions.

    Outputs:
    - List[GateSphere]: List of GateSphere objects with boundary coordinates for each gate.
    """

    RADIUS = 2
    spheres = []
    for gate in gateDataObjs:
        x_min = gate.x_pos-RADIUS
        x_max = gate.x_pos+RADIUS
        y_min = gate.y_pos-RADIUS
        y_max = gate.y_pos+RADIUS
        z_min = gate.z_pos-RADIUS
        z_max = gate.z_pos+RADIUS
        currentSphere = swarms.GateSphere(gate.gateNumber, x_min, x_max, y_min, y_max, z_min, z_max)
        spheres.append(currentSphere)
    return spheres

def reachedSphere(sphere: swarms.GateSphere) -> bool:
    """
    Description:
    - Checks if the drone is currently within the boundaries of a given gate sphere.

    Inputs:
    - sphere (GateSphere): A GateSphere object containing min/max boundary coordinates.

    Outputs:
    - bool: True if the drone is within the sphere, otherwise False.
    """

    # get drone position
    dronePose = swarms.client.getMultirotorState(vehicle_name="drone_1").kinematics_estimated.position
    x_pos = dronePose.x_val
    y_pos = dronePose.y_val
    z_pos = dronePose.z_val
    # x position check
    if (x_pos > sphere.x_min and x_pos < sphere.x_max):
        # y position check
        if (y_pos > sphere.y_min and y_pos < sphere.y_max):
            # z position check
            if (z_pos > sphere.z_min and z_pos < sphere.z_max):
                return True
    return False