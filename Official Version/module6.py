import imports as swarms

# MODULE 6: Movement Functions

def moveToPosition(waypoint_num, x, y, z, vel):
    print("Going to waypoint: "+ str(waypoint_num))
    swarms.client.moveToPositionAsync(x, y, z, vel).join()

def moveByVelocity(vx, vy, vz):
    # print("Going to waypoint: "+ str(waypoint_num))
    # print("Sending signal: x="+ str(vx)+", y="+ str(vy)+", z="+str(vz))
    swarms.client.moveByVelocityAsync(vx, vy, vz, 0.001)

def check_distance(target_x, target_y, target_z):
    THRESHOLD = 1.5
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
            
def startMovement(gateDataObjs, splineWaypts):
    # used for performance metrics
    vel_total_x, vel_total_y, vel_total_z = 0, 0, 0

    # call functions to plot generated waypoints on 3D grid
    # swarms.plotWaypoints(gateDataObjs, splineWaypts)

    # send to first waypoint (get drone moving in proper direction first)
    moveToPosition(0, splineWaypts[0].x_val, splineWaypts[0].y_val, splineWaypts[0].z_val, 8)
    splineWaypts.pop(0)
    startTime = swarms.time.time()

    # DEFINE PARAMS AND INITALIZE CONTROLLER
    kp, ki, kd = 2.0, 0.01, 0.1 # regular performance
    # kp, ki, kd = 2.0, 0.03, 0.2 # hecka bounce
    # kp, ki, kd = 1.7, 0.025, 0.27 # high accuracy lowk but gotta test more
    controller = swarms.controller_PID(kp, ki, kd)

    pid_counter = 1
    wp_counter = 1
    # iterate through all waypoints
    for wp in splineWaypts:
        # retrieve coords
        target_x, target_y, target_z = wp.x_val, wp.y_val, wp.z_val

        # set a bool for check if done (reached/close enough to waypoint)
        status = False
        while (status == False):
            # tune parameters every 2000 runs?
            if (pid_counter%2000 == 0):
                controller.tune_pid()

            # apply pid and read controller signal
            velocity_x, velocity_y, velocity_z = controller.compute_correction(target_x, target_y, target_z)
            vel_total_x += velocity_x
            vel_total_y += velocity_y
            vel_total_z += velocity_z
            moveByVelocity(velocity_x, velocity_y, velocity_z)

            # check if near waypoint
            status = check_distance(target_x, target_y, target_z)
            # if (pid_counter>=12000): # check for singular wp, parameter tuning
            #     status=True
            pid_counter+=1
    wp_counter+=1
    # WINDUP/ANTI-WINDUP??? (reset integral values when switching to a new waypoint)
    # https://www.reddit.com/r/ControlTheory/comments/8ys38i/integrator_windup_and_antiwindup/?rdt=59574
    # controller.integral_x = 0
    # controller.integral_y = 0
    # controller.integral_z = 0
    print("Move commands complete.")

    # plot results
    endTime = swarms.time.time()
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