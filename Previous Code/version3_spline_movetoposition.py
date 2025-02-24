import airsimneurips
from dataclasses import dataclass
from typing import List
import time
import numpy
from scipy.interpolate import CubicSpline

@dataclass
class GateObject:
    gateNumber: int
    x_pos: float
    y_pos: float
    z_pos: float

@dataclass
class GateSphere:
    gateNumber: int
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float

@dataclass
class SplineObject:
    gateNumber: str
    waypoints: List[airsimneurips.Vector3r]



client = airsimneurips.MultirotorClient()
def setup():
    client.confirmConnection()
    client.simLoadLevel('Soccer_Field_Easy')
    client.enableApiControl(vehicle_name="drone_1")
    client.arm(vehicle_name="drone_1")
    client.simStartRace()

def moveToStart():
    start_position = airsimneurips.Vector3r(-4.25, -2.0, 1.8)
    start_rotation = airsimneurips.Quaternionr(0, 0, 0, 4.71)
    new_pose = airsimneurips.Pose(start_position, start_rotation)
    client.simSetVehiclePose(new_pose, ignore_collison=True)


def getGateData():
    # get all the gate object names
    scene_object_names = client.simListSceneObjects(name_regex='.*')
    gateNameList = []
    for name in scene_object_names:
        if ("Gate" in name):
            gateNameList.append(name)

    # create gate objects of desired data
    gateObjectList = []
    for gateName in gateNameList:
        gatePose = (client.simGetObjectPose(gateName)) # returns true pose 
        x_pos = gatePose.position.x_val
        y_pos = gatePose.position.y_val
        z_pos = gatePose.position.z_val
        gateNum = int( gateName[4:6] )
        currentGateObject = GateObject(gateNum, (x_pos-5.5), y_pos, z_pos) # adjust by 5.5 towards correct position
        gateObjectList.append(currentGateObject)

    # sort the gate objects from start gate to end gate
    sortedGateObjectList = sorted(gateObjectList, key=lambda i: i.gateNumber)

    return sortedGateObjectList


def getGateSphereData(gateDataObjs):
    RADIUS = 2.7
    spheres = []
    for gate in gateDataObjs:
        x_min = gate.x_pos-RADIUS
        x_max = gate.x_pos+RADIUS
        y_min = gate.y_pos-RADIUS
        y_max = gate.y_pos+RADIUS
        z_min = gate.z_pos-RADIUS
        z_max = gate.z_pos+RADIUS
        currentSphere = GateSphere(gate.gateNumber, x_min, x_max, y_min, y_max, z_min, z_max)
        spheres.append(currentSphere)
    return spheres


def reachedSphere(sphere):
    # get drone position
    dronePose = client.getMultirotorState(vehicle_name="drone_1").kinematics_estimated.position
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

def getSplineData(gateDataObjs):
    WAYPOINT_AMNT = 4
    splines = []
    for i in range(len(gateDataObjs)-1):
        # set start and end gate positions
        startGate = gateDataObjs[i]
        endGate = gateDataObjs[i+1]
        startGateX = round(startGate.x_pos, 3)
        startGateY = round(startGate.y_pos, 3)
        startGateZ = round(startGate.z_pos, 3)
        endGateX = round(endGate.x_pos, 3)
        endGateY = round(endGate.y_pos, 3)
        endGateZ = round(endGate.z_pos, 3)

        # set interpolation range + set cubic splines for coordinates
        t = [0, 1]
        spline_x = CubicSpline(t, [startGateX, endGateX])
        spline_y = CubicSpline(t, [startGateY, endGateY])
        spline_z = CubicSpline(t, [startGateZ, endGateZ])

        # create waypoints between gates coordinates
        t_new = numpy.linspace(0, 1, WAYPOINT_AMNT)
        waypts = []
        for t_val in t_new:
            x_val = float(spline_x(t_val))
            y_val = float(spline_y(t_val))
            z_val = float(spline_z(t_val))
            waypts.append(airsimneurips.Vector3r(x_val, y_val, z_val))

        # save as spline object dataclass
        path = str(startGate.gateNumber) + " --> " + str(endGate.gateNumber)
        splines.append(SplineObject(gateNumber=path, waypoints=waypts))
    return splines


def startMovement(gateSphereObjs, splineObjs):
    for i in range(len(splineObjs)):
        # Initialize movement on spline
        print('starting spline: ' + str(splineObjs[i].gateNumber))
        waypoints=splineObjs[i].waypoints
        for wp in waypoints:
            client.moveToPositionAsync(wp.x_val, wp.y_val, wp.z_val, 8.25)
        #############
        #client.moveOnSplineAsync(waypoints=splineObjs[i].waypoints, vel_max=25)
        #############
        # Check if reached sphere
        if (i+1) < len(gateSphereObjs):
            currentSphere = gateSphereObjs[i+1]
            print('checking if in gate sphere: ' + str(currentSphere.gateNumber))
            while (reachedSphere(currentSphere) == False):
                pass

if __name__ == '__main__':
    # client connection setup commands
    setup()

    # initalize start position for drone
    moveToStart()

    # get gate position data
    gateDataObjects = getGateData()

    # get gate sphere data
    gateSphereObjects = getGateSphereData(gateDataObjects)

    # calculate spline data
    splineObjects =  getSplineData(gateDataObjects)

    # move to a gate location
    startMovement(gateSphereObjects, splineObjects)

    print("move commands complete")