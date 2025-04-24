import imports as swarms

# MODULE 2: Reading Gate Position Data


def getGateData() -> None:
    """
    Description:
    - Retrieves and parses all gate objects from the simulation environment.
    - Filters objects by name, extracts positions, adjusts alignment, and sorts by gate number.

    Inputs:
    - None

    Outputs:
    - List of GateObject instances, sorted in ascending order by gate number.
    """

    # extract scene objects
    scene_object_names = swarms.client.simListSceneObjects(name_regex='.*')

    # extract scene object names
    gateNameList = [name for name in scene_object_names if 'Gate' in name]

    # create gate objects of gate scene object data only
    gateObjectList = []
    for gateName in gateNameList:
        gatePose = (swarms.client.simGetObjectPose(gateName)) # returns true pose 
        x_pos = gatePose.position.x_val
        y_pos = gatePose.position.y_val
        z_pos = gatePose.position.z_val
        gateNum = int(gateName[4:6])
        currentGateObject = swarms.GateObject(gateNum, (x_pos-5), y_pos, z_pos) # adjust by 5.5 towards correct position
        gateObjectList.append(currentGateObject)

    # sort the gate objects from start gate to end gate
    sortedGateObjectList = sorted(gateObjectList, key=lambda i: i.gateNumber)

    # update issue with last gate
    sortedGateObjectList[19].x_pos += 0.8
    sortedGateObjectList[19].y_pos -= 0.1
    sortedGateObjectList[19].z_pos += 1
    # start_position = swarms.airsimneurips.Vector3r(sortedGateObjectList[19].x_pos,
    #                                                sortedGateObjectList[19].y_pos,
    #                                                sortedGateObjectList[19].z_pos)
    # start_rotation = swarms.airsimneurips.Quaternionr(0, 0, -0.17364919185638428, 0.984807550907135)
    # new_pose = swarms.airsimneurips.Pose(start_position, start_rotation)
    # swarms.client.simSetVehiclePose(new_pose, ignore_collison=True)
    # swarms.time.sleep(500)
    return sortedGateObjectList