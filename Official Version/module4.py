import imports as swarms

# MODULE 4: Spline Interpolation

WAYPOINT_AMNT = 120
def getSplineData(gateDataObjs):

    # Create splines
    # set interpolation range + set cubic splines for coordinates
    t = swarms.np.linspace(0, 1, len(gateDataObjs))

    # loop to deal with error "ValueError: `y` must contain only finite values."
    attempts = 0
    success = False
    while (success == False):
        try:
            spline_x = swarms.CubicSpline(t, [gate.x_pos for gate in gateDataObjs])
            spline_y = swarms.CubicSpline(t, [gate.y_pos for gate in gateDataObjs])
            spline_z = swarms.CubicSpline(t, [gate.z_pos for gate in gateDataObjs])
            success = True
        except Exception as e:
            print("NOTICE: Finite value error occurred. Exception handled, retrying...")
            attempts+=1
            if (attempts >= 5):
                print("NOTICE: Force stopping program due to ValueError")
                quit()
            continue

    # create waypoints between gates coordinates
    waypoints = []
    t_new = swarms.np.linspace(0, 1, WAYPOINT_AMNT)
    for t_val in t_new:
        x_val = float(spline_x(t_val))
        y_val = float(spline_y(t_val))
        z_val = float(spline_z(t_val))
        waypoints.append(swarms.airsimneurips.Vector3r(x_val, y_val, z_val)) 

    return waypoints