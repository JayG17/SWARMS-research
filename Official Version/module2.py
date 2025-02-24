import imports as swarms

# MODULE 2: Reading Gate Position Data

def getGateData():
    # get all the gate object names
    scene_object_names = swarms.client.simListSceneObjects(name_regex='.*')
    gateNameList = []
    for name in scene_object_names:
        if ("Gate" in name):
            gateNameList.append(name)

    # create gate objects of desired data
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
    sortedGateObjectList[len(sortedGateObjectList)-1].x_pos += 1

    return sortedGateObjectList