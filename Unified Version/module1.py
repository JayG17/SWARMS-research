import imports as swarms

# MODULE 1: Basic Setup Functions


def setup_qualifier(typeTest: int) -> None:
    """
    Description:
    - Connects to the simulation client.
    - Loads the Qualifier_Tier_1 level.
    - Enables API control and arms the drone.
    - Starts the race using 'simStartRace()'.

    Inputs:
    - typeTest (int): 1 for setup only, 2 to also start the race.

    Outputs:
    - None
    """
    swarms.client.confirmConnection()
    swarms.client.confirmConnection()
    swarms.client.simLoadLevel('Qualifier_Tier_1')
    swarms.client.enableApiControl(vehicle_name="drone_1")
    swarms.client.arm(vehicle_name="drone_1")
    if (typeTest == 1):
        pass
    elif (typeTest == 2):
        swarms.client.simStartRace(1)

def moveToStart() -> None:
    """
    Description:
    - Repositions and reorients the drone to a fixed starting location.
    - Sets a pose slightly above the starting enclosure for consistent initialization.

    Inputs:
    - None

    Outputs:
    - None
    """
    start_position = swarms.airsimneurips.Vector3r(1.3731292486190796, 81.43741607666016, -43.8)
    start_rotation = swarms.airsimneurips.Quaternionr(0, 0, -0.17364919185638428, 0.984807550907135)
    new_pose = swarms.airsimneurips.Pose(start_position, start_rotation)
    swarms.client.simSetVehiclePose(new_pose, ignore_collison=True)


