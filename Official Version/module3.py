import imports as swarms

# MODULE 3: Sphere Functions

def getGateSphereData(gateDataObjs):
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

def reachedSphere(sphere):
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