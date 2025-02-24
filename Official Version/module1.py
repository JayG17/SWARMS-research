import imports as swarms

# MODULE 1: Basic Setup Functions

def setup_qualifier(typeTest):
    swarms.client.confirmConnection()
    swarms.client.confirmConnection()
    swarms.client.simLoadLevel('Qualifier_Tier_1')
    swarms.client.enableApiControl(vehicle_name="drone_1")
    swarms.client.arm(vehicle_name="drone_1")
    if (typeTest == 1):
        pass
    elif (typeTest == 2):
        swarms.client.simStartRace(1)


def moveToStart():
    # Default start position
    # Drone Position: X=1.3731292486190796, Y=81.43741607666016, Z=-43.090362548828125
    # Drone Rotation Angle (Quaternion): qx=0.0, qy=-0.0, qz=-0.17364919185638428, qw=0.984807550907135

    # Adjusted start position (slightly above start box)
    start_position = swarms.airsimneurips.Vector3r(1.3731292486190796, 81.43741607666016, -43.8)
    start_rotation = swarms.airsimneurips.Quaternionr(0, 0, -0.17364919185638428, 0.984807550907135)
    new_pose = swarms.airsimneurips.Pose(start_position, start_rotation)
    swarms.client.simSetVehiclePose(new_pose, ignore_collison=True)


