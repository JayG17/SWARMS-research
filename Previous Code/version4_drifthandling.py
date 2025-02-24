import airsimneurips
from dataclasses import dataclass
from typing import List
import time
import numpy as np
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
    velocityCalculation: float


client = airsimneurips.MultirotorClient()
def setup_soccer():
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
        gateNum = int(gateName[4:6])
        currentGateObject = GateObject(gateNum, (x_pos-5), y_pos, z_pos) # adjust by 5.5 towards correct position
        gateObjectList.append(currentGateObject)

    # sort the gate objects from start gate to end gate
    sortedGateObjectList = sorted(gateObjectList, key=lambda i: i.gateNumber)

    return sortedGateObjectList


def getGateSphereData(gateDataObjs):
    RADIUS = 4
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
    WAYPOINT_AMNT = 10
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
        t_new = np.linspace(0, 1, WAYPOINT_AMNT)
        waypts = []
        for t_val in t_new:
            x_val = float(spline_x(t_val))
            y_val = float(spline_y(t_val))
            z_val = float(spline_z(t_val))
            waypts.append(airsimneurips.Vector3r(x_val, y_val, z_val))

        # save as spline object dataclass
        path = str(startGate.gateNumber) + " --> " + str(endGate.gateNumber)
        splines.append(SplineObject(gateNumber=path, waypoints=waypts, velocityCalculation=11))
    return splines

def calculateSplineVelocities(splineObjects):
    for i in range(len(splineObjects)-1):
        # get current spline curvature
        currentSpline = splineObjects[i]
        currentCurvature = calculateCurvature(currentSpline.waypoints)
        # get next spline curvature
        nextSpline = splineObjects[i+1]
        nextCurvature = calculateCurvature(nextSpline.waypoints)
        # determine velocity
        print("Current Curvature:"+str(currentCurvature))
        if (currentCurvature > 2000 or nextCurvature > 2000):
            print("set vel=8.75")
            velocity = 8.75
        elif (currentCurvature > 500 or nextCurvature > 500):
            print("set vel=9.25")
            velocity = 9.25
        elif (currentCurvature > 100 or nextCurvature > 100):
            print("set vel=10.25")
            velocity = 10.25
        else:
            print("set vel=11")
            velocity = 11
        splineObjects[i].velocityCalculation = velocity

    return splineObjects

def calculateCurvature(waypoints):
    total_k = 0.0 # k curvature calculation
    for i in range(1, len(waypoints) - 1):
        # step 1: get set of three points (use np.array for easy in-array subtraction)
        point1 = np.array([waypoints[i-1].x_val, waypoints[i-1].y_val, waypoints[i-1].z_val])
        point2 = np.array([waypoints[i].x_val, waypoints[i].y_val, waypoints[i].z_val])
        point3 = np.array([waypoints[i+1].x_val, waypoints[i+1].y_val, waypoints[i+1].z_val])
        
        # step 2: calculate possible tangents (tangent vectors)
        tangent1 = point2 - point1
        tangent2 = point3 - point2

        # step 3: calculate unit tangent vectors
        unit_tangent1 = tangent1/np.linalg.norm(tangent1)
        unit_tangent2 = tangent2/np.linalg.norm(tangent2)

        # step 3: calculate dT (change/difference in unit tangent vectors)
        dT = unit_tangent2 - unit_tangent1

        # step 4: calculate ds (arc length between points)
        ds = (np.linalg.norm(tangent1) + np.linalg.norm(tangent2)) / 2
        
        # step 5: calculate k
        if ds == 0:
            k = 0
        else:
            k = np.linalg.norm(dT) / ds
        
        total_k += k


    total_k = round(total_k* (10**17), 2)

    return total_k


def startMovement(gateSphereObjs, splineObjs):
    print("----- begin movement ------")
    for i in range(len(splineObjs)):
        # Initialize movement on spline
        print('starting spline: ' + str(splineObjs[i].gateNumber))
        waypoints=splineObjs[i].waypoints
        velCalculation=splineObjs[i].velocityCalculation
        for wp in waypoints:
            client.moveToPositionAsync(wp.x_val, wp.y_val, wp.z_val, velCalculation)

        # Check if reached sphere
        if (i+1) < len(gateSphereObjs):
            currentSphere = gateSphereObjs[i+1]
            print('checking if in gate sphere: ' + str(currentSphere.gateNumber))
            while (reachedSphere(currentSphere) == False):
                pass


def teleport(x,y,z):
    start_position = airsimneurips.Vector3r(x,y,z)
    start_rotation = airsimneurips.Quaternionr(0, 0, 0, 4.71)
    new_pose = airsimneurips.Pose(start_position, start_rotation)
    client.simSetVehiclePose(new_pose, ignore_collison=True)

if __name__ == '__main__':
    # client connection setup commands
    setup_soccer()

    # initalize start position for drone
    moveToStart()

    # get gate position data
    gateDataObjects = getGateData()

    # get gate sphere data
    gateSphereObjects = getGateSphereData(gateDataObjects)

    # calculate spline data
    splineObjects = getSplineData(gateDataObjects)

    #drift handling
    # calculate spline curve roughness
    splineObjects = calculateSplineVelocities(splineObjects)

    # move to a gate location
    startMovement(gateSphereObjects, splineObjects)

    print("move commands complete")