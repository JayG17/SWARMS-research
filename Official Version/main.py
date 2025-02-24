import imports as swarms



if __name__ == '__main__':
    # client connection setup commands
    #   send 1 for practice
    #   send 2 for official test
    swarms.setup_qualifier(2)

    # initalize start position for drone
    swarms.moveToStart()

    # get gate position data
    gateDataObjects = swarms.getGateData()

    # get gate sphere data
    # gateSphereObjects = swarms.getGateSphereData(gateDataObjects)

    # calculate spline data
    splineObjects = swarms.getSplineData(gateDataObjects)

    # move to a gate location
    swarms.startMovement(gateDataObjects, splineObjects)