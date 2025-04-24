import imports as swarms
from typing import List

# MODULE 6: Movement Functions


def moveToPosition(waypoint_num: int, x: float, y: float, z: float, vel: float) -> None:
    """
    Moves the drone to a specified 3D position at a set velocity.

    Inputs:
    - waypoint_num (int): Index or identifier of the current waypoint
    - x, y, z (float): Target 3D coordinates
    - vel (float): Desired velocity

    Outputs:
    - None
    """
    print("Going to waypoint: "+ str(waypoint_num))
    swarms.client.moveToPositionAsync(x, y, z, vel).join()

def moveByVelocity(vx: float, vy: float, vz: float) -> None:
    """
    Sends a velocity command to the drone.

    Inputs:
    - vx, vy, vz (float): Velocity in NED X, Y, and Z axes

    Outputs:
    - None
    """
    # print("Going to waypoint: "+ str(waypoint_num))
    # print("Sending signal: x="+ str(vx)+", y="+ str(vy)+", z="+str(vz))
    swarms.client.moveByVelocityAsync(vx, vy, vz, 0.001)

def check_distance(target_x: float, target_y: float, target_z: float, thold: float) -> bool:
    """
    Checks whether the drone is within a threshold distance of a target position.

    Inputs:
    - target_x, target_y, target_z (float): Coordinates of the target position

    Outputs:
    - bool: True if within threshold, False otherwise
    """
    THRESHOLD = thold
    current_pos_x, current_pos_y, current_pos_z = swarms.getDronePos()
    x_min = target_x - THRESHOLD
    x_max = target_x + THRESHOLD
    y_min = target_y - THRESHOLD
    y_max = target_y + THRESHOLD
    z_min = target_z - THRESHOLD
    z_max = target_z + THRESHOLD
    # x position check
    if (current_pos_x > x_min and current_pos_x < x_max):
        # y position check
        if (current_pos_y > y_min and current_pos_y < y_max):
            # z position check
            if (current_pos_z > z_min and current_pos_z < z_max):
                return True
    return False

def startMovement(gateDataObjs: List[swarms.GateObject], splineWaypts: List[swarms.airsimneurips.Vector3r]) -> None:
    """
    Initiates full motion sequence through spline waypoints using PID controller.

    Inputs:
    - gateDataObjs (List[GateObject]): List of gates with positional data
    - splineWaypts (List[Vector3r]): Interpolated waypoints for the drone to follow

    Outputs:
    - None (controls drone motion and visualizes performance)
    """

    # used for performance metrics
    vel_total_x, vel_total_y, vel_total_z = 0, 0, 0
    # swarms.plotWaypoints(gateDataObjs, splineWaypts)

    # send to first waypoint (get drone moving in proper direction first)
    moveToPosition(0, splineWaypts[0].x_val, splineWaypts[0].y_val, splineWaypts[0].z_val, 8)
    splineWaypts.pop(0)
    startTime = swarms.time.time()

    # intialize flight data collector data-collection class
    # fdc = swarms.FlightDataCollector(client=swarms.client, capture_interval=0.05, vehicle_name="drone_1")
 
    # DEFINE PARAMS AND INITALIZE CONTROLLER
    # kp, ki, kd = 2.0, 0.01, 0.1 # regular performance
    # kp, ki, kd = 2.0, 0.03, 0.2 # hecka bounce
    # kp, ki, kd = 1.7, 0.025, 0.27 # high accuracy lowk but gotta test more
    # kp, ki, kd = 2.1, 0.009, 0.09 # slightly faster movement, avoid agrressive descent slamming
    kp, ki, kd = 2.0, 0.01, 0.08 # lower Z aggressiveness
    # kp, ki, kd = 2.15, 0.008, 0.18 # faster test, works well
    # kp, ki, kd = 3.5, 0.01, 0.2 # even faster test
    controller = swarms.controller_PID(kp, ki, kd)

    pid_counter = 1
    wp_counter = 1
    # splineWaypts = splineWaypts[80::]
    totalwaypts = len(splineWaypts)
    # iterate through all waypoints
    for wp in splineWaypts:
        # retrieve coords
        target_x, target_y, target_z = wp.x_val, wp.y_val, wp.z_val

        # set a bool for check if done (reached/close enough to waypoint)
        status = False
        while (status == False):

            # apply pid and read controller signal
            velocity_x, velocity_y, velocity_z = controller.compute_correction(target_x, target_y, target_z)
            vel_total_x += velocity_x
            vel_total_y += velocity_y
            vel_total_z += velocity_z
            moveByVelocity(velocity_x, velocity_y, velocity_z)

            # check if near waypoint
            # follow closer based on if closer to end
            if(wp_counter>totalwaypts-6):
                status = check_distance(target_x, target_y, target_z, 0.5)
            elif(wp_counter>totalwaypts-12):
                status = check_distance(target_x, target_y, target_z, 0.7)
            else:
                status = check_distance(target_x, target_y, target_z, 1.4)
            # if (pid_counter>=12000): # check for singular wp, parameter tuning
            #     status=True
            #fdc.capture(swarms.client)
            pid_counter+=1
    wp_counter+=1
    # WINDUP/ANTI-WINDUP??? (reset integral values when switching to a new waypoint)
    # https://www.reddit.com/r/ControlTheory/comments/8ys38i/integrator_windup_and_antiwindup/?rdt=59574
    # controller.integral_x = 0
    # controller.integral_y = 0
    # controller.integral_z = 0
    print("Move commands complete.")
    

    #swarms.capture_plot_reference(fdc)
    #fdc.plot_flight_path()

    # plot results
    endTime = swarms.time.time()

    # call functions to plot generated waypoints on 3D grid
    swarms.plotWaypoints(gateDataObjs, splineWaypts)
    swarms.plot_magnitude_data(controller.display_time, controller.display_magnitude)
    swarms.plot_pid_data(controller.display_time, controller.display_errors)

    # run performance metrics
    print("----- PERFORMANCE METRICS -----")
    swarms.metric_1(endTime-startTime)
    swarms.metric_2(controller.display_magnitude)
    swarms.metric_3(controller.display_errors)

    # added an infinite loop to prevent fatal error due to end of instruction
    print("End program.")
    while (True):
        continue