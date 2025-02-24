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
    curveCalculation: float
    velocityCalculation: float


client = airsimneurips.MultirotorClient()
def setup_qualifier():
    client.confirmConnection()
    client.simLoadLevel('Qualifier_Tier_1')
    client.enableApiControl(vehicle_name="drone_1")
    client.arm(vehicle_name="drone_1")
    client.simStartRace(1)

def moveToStart():
    start_position = airsimneurips.Vector3r(1.3731292486190796, 81.43741607666016, -43.75)
    start_rotation = airsimneurips.Quaternionr(0, 0, 0, 4.71)
    new_pose = airsimneurips.Pose(start_position, start_rotation)
    client.simSetVehiclePose(new_pose, ignore_collison=True)

def teleport(x,y,z):
    start_position = airsimneurips.Vector3r(x,y,z)
    start_rotation = airsimneurips.Quaternionr(0, 0, 0, 4.71)
    new_pose = airsimneurips.Pose(start_position, start_rotation)
    client.simSetVehiclePose(new_pose, ignore_collison=True)

def getGateData():
    gateObjectList = []
    gateObjectList.append(GateObject(0, 10.388-5.25, 80.774, -43.58))#set
    gateObjectList.append(GateObject(1, 18.11-5.25, 76.261, -43.58 ))#set
    gateObjectList.append(GateObject(2, 25.434-5, 66.287, -43.58))#set
    gateObjectList.append(GateObject(3, 30.066-5, 56.55, -43.58 ))#set
    gateObjectList.append(GateObject(4, 32.301-5, 45.631, -43.88))#set
    gateObjectList.append(GateObject(5, 26.503-5, 38.2, -43.38))#set curved gate 1
    gateObjectList.append(GateObject(6, 3.264-5, 37.569, -43.58))#set
    gateObjectList.append(GateObject(7, -16.863-5, 45.418, -46.58))#set
    gateObjectList.append(GateObject(8, -15.494-5, 63.187, -52.08))#set
    gateObjectList.append(GateObject(9, -6.321-6, 78.212, -55.78))#set curved  gate2
    gateObjectList.append(GateObject(10, 5.144-6, 82.385, -55.78))#set
    gateObjectList.append(GateObject(11, 14.559-4, 84.432, -55.18))#set
    gateObjectList.append(GateObject(12, 23.859-5, 82.832, -32.08))#set
    gateObjectList.append(GateObject(13, 38.259-5, 78.132, -31.38))#set
    gateObjectList.append(GateObject(14, 51.059-5, 52.132, -25.88))#set
    gateObjectList.append(GateObject(15, 44.959-5, 38.932, -25.88))#set
    gateObjectList.append(GateObject(16, 25.959-5, 26.332, -19.88))#set
    gateObjectList.append(GateObject(17, 11.659-5, 26.332, -12.78))#set
    gateObjectList.append(GateObject(18, -10.141-5, 22.632, -6.38))#set
    gateObjectList.append(GateObject(19, -24.641-6, 9.132, 2.12))#set
    # for i in range(0, 20, 1):
    #     teleport(-6.321-5, 78.212, -55.78)
    #     time.sleep(0.25)
    # quit()
    return gateObjectList


def getGateSphereData(gateDataObjs):
    RADIUS = 5
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
    WAYPOINT_AMNT = 30
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
        splines.append(SplineObject(gateNumber=path, waypoints=waypts, curveCalculation=0, velocityCalculation=0))
    return splines


def startMovement(gateData, gateSphereObjs, splineObjs):
    ### METHOD 1 ### using sphere
    # for i in range(len(splineObjs)):
    #     # Initialize movement on spline
    #     print('starting spline: ' + str(splineObjs[i].gateNumber))
    #     waypts=splineObjs[i].waypoints
    #     for wp in waypts:
    #         client.moveToPositionAsync(wp.x_val, wp.y_val, wp.z_val, 4).join()

    #     # Check if reached sphere
    #     if (i+1) < len(gateSphereObjs):
    #         currentSphere = gateSphereObjs[i+1]
    #         print('checking if in gate sphere: ' + str(currentSphere.gateNumber))
    #         while (reachedSphere(currentSphere) == False):
    #             # time.sleep(0.1)
    #             pass

    ### METHOD 2 ### using stitched version
    total_arry: List[airsimneurips.Vector3r] = []
    for eachSpline in splineObjects:
        waypts=eachSpline.waypoints
        total_arry += waypts
    for wp in total_arry:
        client.moveToPositionAsync(wp.x_val, wp.y_val, wp.z_val, 5).join()


if __name__ == '__main__':
    # client connection setup commands
    setup_qualifier()

    # initalize start position for drone
    moveToStart()

    # get gate position data
    gateDataObjects = getGateData()

    # get gate sphere data
    gateSphereObjects = getGateSphereData(gateDataObjects)

    # calculate spline data
    splineObjects = getSplineData(gateDataObjects)

    # move to a gate location
    startMovement(gateDataObjects, gateSphereObjects, splineObjects)

    print("move commands complete")