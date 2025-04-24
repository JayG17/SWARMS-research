import imports as swarms
from typing import List

# MODULE 4: Spline Interpolation


def getSplineData(gateDataObjs: List[swarms.GateObject]) -> List[swarms.airsimneurips.Vector3r]:
    """
    Description:
    - Generates a smooth cubic spline interpolation between gate positions to create a refined path.
    - Handles non-finite values error during spline fitting.
    - Outputs a list of interpolated 3D waypoints used for drone navigation.

    Inputs:
    - gateDataObjs (List[GateObject]): A list of GateObject instances containing gate positions.

    Outputs:
    - List[airsimneurips.Vector3r]: Interpolated waypoint positions as Vector3r objects.
    """

    # set a constant waypoint amount for race range
    # WAYPOINT_AMNT = 120
    WAYPOINT_AMNT = 125

    # identify current and new interpolation range for the gate range
    t_current = swarms.np.linspace(0, 1, len(gateDataObjs))
    t_new = swarms.np.linspace(0, 1, WAYPOINT_AMNT)

    # create cubic spline, given current interpolation range and gate positions
    # loop with exceptions to deal with the common ValueError issue that
    # appears, "ValueError: `y` must contain only finite values."
    attempts = 0
    success = False
    while (success == False):
        try:
            spline_x = swarms.CubicSpline(t_current, [gate.x_pos for gate in gateDataObjs])
            spline_y = swarms.CubicSpline(t_current, [gate.y_pos for gate in gateDataObjs])
            spline_z = swarms.CubicSpline(t_current, [gate.z_pos for gate in gateDataObjs])
            success = True
        except Exception as e:
            print("NOTICE: Finite value error occurred. Exception handled, retrying...")
            attempts+=1
            if (attempts >= 5):
                print("NOTICE: Force stopping program due to ValueError")
                quit()
            continue

    # create waypoints of Vector3r between gates coordinates
    waypoints = []
    for t in t_new:
        x_val = float(spline_x(t))
        y_val = float(spline_y(t))
        z_val = float(spline_z(t))
        waypoints.append(swarms.airsimneurips.Vector3r(x_val, y_val, z_val)) 

    return waypoints