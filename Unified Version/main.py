import imports as swarms

# MAIN MODULE


"""
Main program execution for SWARMS drone navigation.
"""
if __name__ == '__main__':
    # Connect to AirSim client and load environment
    #   send 1 for practice run (w/o simStartRace)
    #   send 2 for official test (w/ simStartRace)
    swarms.setup_qualifier(2)

    # Move drone to the initial starting pose
    swarms.moveToStart()

    # Retrieve gate positional data from the scene
    gateDataObjects = swarms.getGateData()

    # Optional: Calculate spheres around each gate for sphere radius check
    # gateSphereObjects = swarms.getGateSphereData(gateDataObjects)

    # Generate interpolated spline waypoints between gates
    splineObjects = swarms.getSplineData(gateDataObjects)

    # Begin movement using PID-based control along spline
    swarms.startMovement(gateDataObjects, splineObjects)