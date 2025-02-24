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
    #client.simStartRace()

def moveToStart():
    # start_position = airsimneurips.Vector3r(-4.25, -2.0, 1.8)# above box
    #start_position = airsimneurips.Vector3r(-4.25, -2.0, 1.7) # more above box
    #start_position = airsimneurips.Vector3r(-4.55, 1.0, 2.0) # at gate
    start_position = airsimneurips.Vector3r(-4.55, 0.5, 2.0) # just right before gate
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
        currentGateObject = GateObject(gateNum, (x_pos-5.5), y_pos, z_pos) # adjust by 5.5 towards correct position
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
        # print("Current Curvature:"+str(currentCurvature))
        if (currentCurvature > 2000 or nextCurvature > 2000):
            # print("set vel=8.75")
            velocity = 8.75
        elif (currentCurvature > 500 or nextCurvature > 500):
            # print("set vel=9.25")
            velocity = 9.25
        elif (currentCurvature > 100 or nextCurvature > 100):
            # print("set vel=10.25")
            velocity = 10.25
        else:
            # print("set vel=11")
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

# this is the PID controller function that uses the PID control formulas.
# it uses proportional (Kp), integral (Ki), and derivative (Kd) terms
# to reduce error between a desired setpoint and current point.
# the function returns the new output, error, and integral values.
# Note that the integral is the sum of past errors over time.
# https://softinery.com/blog/implementation-of-pid-controller-in-python/
def PID_drift_controller(kp, ki, kd, _setPoint, _currentPoint, integral, prevError, delta_time=0.1):
    target_position = _setPoint
    current_position = _currentPoint

    # step 1. calculate error
    error = target_position - current_position

    # step 2. calculate proporational term (p_term)
    p_term = kp * error

    # step 3. calculate integral term (i_term)
    integral = integral + error*delta_time # updates/accumulates the error over time
    i_term = ki*integral # determine the integral value needed for updating the PID output value

    # step 4. calculate derivative term (d_term)
    if delta_time > 0: # prevent divide by 0 at start
        delta_error = error - prevError
        d_term = kd*delta_error/delta_time # determine the derivative value needed for updating the PID output value
    else:
        d_term = 0

    # step 5. calculate manipulated variable or correction value (MV)
    MV_correction_value = p_term+i_term+d_term

    # step 6. return MV and values to be updated
    return MV_correction_value, error, integral

def getDronePos():
    pos = client.getMultirotorState(vehicle_name="drone_1").kinematics_estimated.position
    return pos.x_val, pos.y_val, pos.z_val


def startMovement(gateSphereObjs, splineObjs):
    print("----- begin movement ------")
    startTime = time.time()
    for i in range(len(splineObjs)):
        # Initialize movement on spline
        print('starting spline: ' + str(splineObjs[i].gateNumber))
        waypoints=splineObjs[i].waypoints
        # curvature calculation method of velCalculation
        velCalculation=splineObjs[i].velocityCalculation

        ######### PID CONTROLLER PORTION #########
        # kp, ki, kd = 1.0, 0.005, 0.02
        # kp, ki, kd = 1.5, 0.01, 0.05 # worked at 5 m/s
        # kp, ki, kd = 2.0, 0.01, 0.1 # worked at 8 m/s
        #kp, ki, kd = 2.0, 0.01, 0.1 # worked at 8.4 m/s
        kp, ki, kd = 2.0, 0.01, 0.1

        prev_error_x, prev_error_y, prev_error_z = 0, 0, 0
        integral_x, integral_y, integral_z = 0, 0, 0
        for wp in waypoints:
            # step 1. get drone target and current position xyz
            target_pos_x, target_pos_y, target_pos_z = wp.x_val, wp.y_val, wp.z_val
            current_pos_x, current_pos_y, current_pos_z = getDronePos()

            # step 3. use PID controller to get corrections
            correction_x, prev_error_x, integral_x = PID_drift_controller(kp, ki, kd, target_pos_x, current_pos_x, integral_x, prev_error_x)
            correction_y, prev_error_y, integral_y = PID_drift_controller(kp, ki, kd, target_pos_y, current_pos_y, integral_y, prev_error_y)
            correction_z, prev_error_z, integral_z = PID_drift_controller(kp, ki, kd, target_pos_z, current_pos_z, integral_z, prev_error_z)

            # step 4. clip the corrections to +-limit to avoid large changes and frequent overshots (big issue here)
            MAX_CORRECTION_LIMIT = 5.0
            correction_x = max(min(correction_x, MAX_CORRECTION_LIMIT), -MAX_CORRECTION_LIMIT)
            correction_y = max(min(correction_y, MAX_CORRECTION_LIMIT), -MAX_CORRECTION_LIMIT)
            correction_z = max(min(correction_z, MAX_CORRECTION_LIMIT), -MAX_CORRECTION_LIMIT)
            
            # step 5. update with corrections
            new_corrected_x = current_pos_x + correction_x
            new_corrected_y = current_pos_y + correction_y
            new_corrected_z = current_pos_z + correction_z

            # step 6. send to waypoint
            print(f"SENDING TO... : X={new_corrected_x}, Y={new_corrected_y}, Z={new_corrected_z}")
            velCalculation=8
            client.moveToPositionAsync(new_corrected_x, new_corrected_y, new_corrected_z, velCalculation)

            # step 7. check if ready for next waypt
            # Check if waypoint is reached
            THRESHOLD = 2
            count=0
            while True:
                count+=1
                current_pos_x, current_pos_y, current_pos_z = getDronePos()
                distance_to_wp = np.sqrt(
                    (target_pos_x - current_pos_x)**2 +
                    (target_pos_y - current_pos_y)**2 +
                    (target_pos_z - current_pos_z)**2
                )
                if (count%2000==0):
                    print(f"Target Waypoint: X={target_pos_x}, Y={target_pos_y}, Z={target_pos_z}")
                    print(f"Current Position: X={current_pos_x}, Y={current_pos_y}, Z={current_pos_z}")
                    print(f"dist = {str(distance_to_wp)}")
                if distance_to_wp <= THRESHOLD:
                    # print(f"Waypoint reached: X={target_pos_x}, Y={target_pos_y}, Z={target_pos_z}")
                    # print(f"------------------------------------")
                    break
                # time.sleep(0.05)

        ##########################################

        # # # Check if reached sphere
        # if (i+1) < len(gateSphereObjs):
        #     currentSphere = gateSphereObjs[i+1]
        #     print('checking if in gate sphere: ' + str(currentSphere.gateNumber))
        #     while (reachedSphere(currentSphere) == False):
        #         pass
        endTime = time.time()
        totalTime = endTime-startTime
        print("****** TOTAL RUNTIME = "+str(totalTime)+" ******")

def startMovement2(gateSphereObjs, splineObjs):
    print("----- begin movement ------")
    for i in range(len(splineObjs)):
        # Initialize movement on spline
        print(f"Starting spline: {splineObjs[i].gateNumber}")
        waypoints = splineObjs[i].waypoints
        velCalculation = splineObjs[i].velocityCalculation

        ######### PID CONTROLLER PORTION #########
        kp, ki, kd = 1.0, 0.005, 0.02
        prev_error_x, prev_error_y, prev_error_z = 0, 0, 0
        integral_x, integral_y, integral_z = 0, 0, 0
        prev_time = time.time()

        for wp in waypoints:
            count = 0
            client.moveToPositionAsync(wp.x_val, wp.y_val, wp.z_val, 6)
            while True:
                count+=1
                # Get target and current positions
                target_pos_x, target_pos_y, target_pos_z = wp.x_val, wp.y_val, wp.z_val
                current_pos_x, current_pos_y, current_pos_z = getDronePos()

                # Print current and target positions
                if (count%1500==0):
                    print(f"Target Waypoint: X={target_pos_x}, Y={target_pos_y}, Z={target_pos_z}")
                    print(f"Current Position: X={current_pos_x}, Y={current_pos_y}, Z={current_pos_z}")

                # Check if waypoint is reached
                REACHED_THRESHOLD = 1.8
                distance_to_wp = np.sqrt(
                    (target_pos_x - current_pos_x)**2 +
                    (target_pos_y - current_pos_y)**2 +
                    (target_pos_z - current_pos_z)**2
                )
                if (count%1500==0):
                    print(f"dist = {distance_to_wp}")
                    print(f"-------------------------")
                if distance_to_wp <= REACHED_THRESHOLD:
                    print(f"Waypoint reached: X={target_pos_x}, Y={target_pos_y}, Z={target_pos_z}")
                    break  # Proceed to the next waypoint

                # Time delta for PID calculation
                current_time = time.time()
                delta_time = max(current_time - prev_time, 1e-5)
                prev_time = current_time

                # PID controller for corrections
                correction_x, prev_error_x, integral_x = PID_drift_controller(kp, ki, kd, target_pos_x, current_pos_x, integral_x, prev_error_x, delta_time)
                correction_y, prev_error_y, integral_y = PID_drift_controller(kp, ki, kd, target_pos_y, current_pos_y, integral_y, prev_error_y, delta_time)
                correction_z, prev_error_z, integral_z = PID_drift_controller(kp, ki, kd, target_pos_z, current_pos_z, integral_z, prev_error_z, delta_time)

                # Clip corrections to prevent large jumps
                # MAX_CORRECTION_LIMIT = 2.0
                # correction_x = max(min(correction_x, MAX_CORRECTION_LIMIT), -MAX_CORRECTION_LIMIT)
                # correction_y = max(min(correction_y, MAX_CORRECTION_LIMIT), -MAX_CORRECTION_LIMIT)
                # correction_z = max(min(correction_z, MAX_CORRECTION_LIMIT), -MAX_CORRECTION_LIMIT)

                # Update with corrections
                new_corrected_x = current_pos_x + correction_x
                new_corrected_y = current_pos_y + correction_y
                new_corrected_z = current_pos_z + correction_z

                # Send movement command
                if (count%1500==0):
                    print(f"Moving to: X={new_corrected_x}, Y={new_corrected_y}, Z={new_corrected_z}, Velocity={velCalculation}")
                velCalculation = 6
                client.moveToPositionAsync(new_corrected_x, new_corrected_y, new_corrected_z, velCalculation)



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

    # calculate spline curve roughness for drift handling
    splineObjects = calculateSplineVelocities(splineObjects)

    # move to a gate location
    #startMovement(gateSphereObjects, splineObjects)
    startMovement(gateSphereObjects, splineObjects)

    print("move commands complete")