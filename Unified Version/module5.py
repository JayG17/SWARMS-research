
import imports as swarms
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import List, Optional, Dict, Tuple, Any
import numpy as np
import math

# MODULE 5: FlightDataCollector, Drift Handling, Performance Metrics, Plotting


# class FlightDataCollector:
#     """
#     A utility class to capture, store, and visualize drone flight data during
#     simulation. This class is designed to support both current development and
#     future research by enabling detailed logging and visualization of flight data.
#     It plays a key role in performance evaluation and debugging, and is intended
#     to be reusable by future researchers for future implementations and extensions.

#     Purpose:
#     - Supports performance tracking by recording position, velocity, orientation, and control input over time.
#     - Helps debug and analyze control logic such as PID behavior.
#     - Offers visualization of 3D flight paths, including dynamic vectors for velocity, orientation, and control effort.

#     Attributes:
#     - client: AirSim client object for communication.
#     - capture_interval: Minimum time delay between consecutive data captures.
#     - vehicle_name: Name of the drone in simulation.
#     - last_capture_time: Timestamp of the last recorded frame.
#     - flight_data: List of dictionaries storing flight-related data.
#     """
#     def __init__(self, client: swarms.airsimneurips.MultirotorClient, capture_interval: float=0.1, vehicle_name: str="drone_1") -> None:
#         self.capture_interval = capture_interval
#         self.client = client
#         self.vehicle_name = vehicle_name
#         self.last_capture_time = swarms.time.time()
#         self.flight_data = []

#     def capture(self, control: Optional[Tuple[float, float, float]] = None) -> None:
#         """
#         Records current position, velocity, orientation, and control vector at a given time if capture interval is met.

#         Inputs:
#         - control: Optional 3D velocity control vector to log (vx, vy, vz).
#         """
#         current_time = swarms.time.time()
#         if current_time - self.last_capture_time >= self.capture_interval:
#             state = self.client.getMultirotorState(vehicle_name=self.vehicle_name)
#             pos = (state.kinematics_estimated.position.x_val,
#                    state.kinematics_estimated.position.y_val,
#                    state.kinematics_estimated.position.z_val)
#             q = state.kinematics_estimated.orientation
#             ori = swarms.get_forward_vector(q)
#             vel = (state.kinematics_estimated.linear_velocity.x_val,
#                    state.kinematics_estimated.linear_velocity.y_val,
#                    state.kinematics_estimated.linear_velocity.z_val)

#             self.flight_data.append({
#                 'time': current_time,
#                 'pos': pos,
#                 'ori': ori,
#                 'vel': vel,
#                 'control': control
#             })

#             self.last_capture_time = current_time

#     def get_flight_data(self) -> List[Dict[str, Any]]:
#         """
#         Returns the recorded flight data.

#         Outputs:
#         - List of telemetry dictionaries.
#         """
#         return self.flight_data

#     def plot_flight_path(
#         self,
#         orientation_color: str = 'green',
#         velocity_color: str = 'blue',
#         control_color: str = 'red'
#     ) -> None:
#         """
#         Plots the drone's 3D flight path with optional gate visualization and vector annotations.

#         Inputs:
#         - gate_positions: Optional dictionary of gate names to Vector3r positions.
#         - orientation_color: Color for orientation vectors.
#         - velocity_color: Color for velocity vectors.
#         - control_color: Color for control vectors.
#         """
#         gate_positions = getGatePositions()
#         fig = plt.figure()
#         ax = fig.add_subplot(111, projection='3d')
        
#         xs = [data['pos'][0] for data in self.flight_data]
#         ys = [data['pos'][1] for data in self.flight_data]
#         zs = [data['pos'][2] for data in self.flight_data]
#         ax.scatter(xs, ys, zs, c='blue', marker='o', label='Flight Path')
#         first_ori = True
#         first_vel = True
#         first_control = True
#         for data in self.flight_data:
#             x, y, z = data['pos']
#             vel = data['vel']
#             ori = data['ori']
#             control = data['control']
#             speed = np.linalg.norm(vel)

#             if first_vel:
#                 ax.quiver(x, y, z, vel[0], vel[1], vel[2], length=1, color=velocity_color, normalize=True, label='Velocity')
#                 first_vel = False
#             else:
#                 ax.quiver(x, y, z, vel[0], vel[1], vel[2], length=1, color=velocity_color, normalize=True)

#             if first_ori:
#                 ax.quiver(x, y, z, ori[0], ori[1], ori[2], length=1, color=orientation_color, normalize=True, label='Orientation')
#                 first_ori = False
#             else:
#                 ax.quiver(x, y, z, ori[0], ori[1], ori[2], length=1, color=orientation_color, normalize=True)
            
#             if control is not None:
#                 if first_control:
#                     ax.quiver(x, y, z, control[0], control[1], control[2], length=1, color=control_color, normalize=True, label='Control')
#                     first_control = False
#                 else:
#                     ax.quiver(x, y, z, control[0], control[1], control[2], length=1, color=control_color, normalize=True)

#             if speed > 0:
#                 ax.text(x, y, z, f"{speed:.2f} m/s", color=velocity_color, fontsize=8)

#         # Draw gate spheres
#         u = np.linspace(0, 2 * np.pi, 20)
#         v = np.linspace(0, np.pi, 20)
#         r = 1.5
#         if gate_positions:
#             for gate, pos in gate_positions.items():
#                 cx, cy, cz = pos.x_val, pos.y_val, pos.z_val
#                 xsphere = cx + r * np.outer(np.cos(u), np.sin(v))
#                 ysphere = cy + r * np.outer(np.sin(u), np.sin(v))
#                 zsphere = cz + r * np.outer(np.ones_like(u), np.cos(v))
#                 ax.plot_wireframe(xsphere, ysphere, zsphere, color='orange', alpha=0.3)
        
#         ax.set_xlabel('X')
#         ax.set_ylabel('Y')
#         ax.set_zlabel('Z')
#         ax.set_title("Drone Flight Path with Orientation and Velocity")
#         ax.legend()
#         xlims = ax.get_xlim3d()
#         ylims = ax.get_ylim3d()
#         zlims = ax.get_zlim3d()

#         xmean = np.mean(xlims)
#         ymean = np.mean(ylims)
#         zmean = np.mean(zlims)
#         max_range = np.max([xlims[1] - xlims[0],
#                             ylims[1] - ylims[0],
#                             zlims[1] - zlims[0]])

#         ax.set_xlim3d([xmean - max_range / 2, xmean + max_range / 2])
#         ax.set_ylim3d([ymean - max_range / 2, ymean + max_range / 2])
#         ax.set_zlim3d([zmean - max_range / 2, zmean + max_range / 2])
#         ax.invert_zaxis()
#         ax.invert_yaxis()

#         plt.show()

# def getGatePositions() -> None:
#     """
#     Retrieves and returns sorted gate positions from the simulation.

#     Returns:
#     - Dict mapping gate names to their Vector3r positions.
#     """
#     objects = swarms.client.simListSceneObjects()
#     print(f"{objects}\n\n")
#     gates = [obj for obj in objects if 'Gate' in obj]
#     print(f"{gates}\n\n")
#     gate_positions = {gate: swarms.client.simGetObjectPose(gate).position for gate in gates}

#     def extract_gate_number(gate_name):
#         remainder = gate_name.replace("Gate", "")
#         number_str = remainder.split("_")[0]
#         try:
#             return int(number_str)
#         except ValueError:
#             return float('inf')
    
#     sorted_gate_positions = {gate: gate_positions[gate] for gate in sorted(gate_positions, key=extract_gate_number)}
#     print(sorted_gate_positions)
#     return sorted_gate_positions

# def get_forward_vector(q: swarms.airsimneurips.Quaternionr) -> Tuple[float, float, float]:
#     """
#     Computes the forward direction vector based on the drone's orientation quaternion.

#     Inputs:
#     - q (Quaternionr): Orientation quaternion of the drone.

#     Outputs:
#     - Tuple[float, float, float]: Forward unit vector (fx, fy, fz) in 3D space.
#     """
#     pitch = math.asin(max(-1.0, min(1.0, 2*(q.w_val*q.y_val - q.z_val*q.x_val))))
#     yaw = math.atan2(2*(q.w_val*q.z_val + q.x_val*q.y_val), 1 - 2*(q.y_val**2 + q.z_val**2))
#     fx = math.cos(pitch) * math.cos(yaw)
#     fy = math.cos(pitch) * math.sin(yaw)
#     fz = math.sin(pitch)
#     return (fx, fy, fz)

# def capture_plot_reference(flight_data_collector: FlightDataCollector) -> None:
#     """
#     Captures two reference drone positions for visualization purposes.
#     Useful for marking the end and start of a flight path on the final plot.

#     Inputs:
#     - flight_data_collector (FlightDataCollector): The object responsible for storing and handling flight data.

#     Outputs:
#     - None
#     """
#     end_position = swarms.airsimneurips.Vector3r(25, 10, -20)
#     end_rotation = swarms.airsimneurips.Quaternionr(0, 0, 0, 4.71)
#     new_pose = swarms.airsimneurips.Pose(end_position, end_rotation)
#     swarms.client.simSetVehiclePose(new_pose, ignore_collison=True)
#     swarms.time.sleep(0.2)
#     flight_data_collector.capture()
#     end_position = swarms.airsimneurips.Vector3r(-3, -2.0, -20)
#     end_rotation = swarms.airsimneurips.Quaternionr(0, 0, 0, 4.71)
#     new_pose = swarms.airsimneurips.Pose(end_position, end_rotation)
#     swarms.client.simSetVehiclePose(new_pose, ignore_collison=True)
#     swarms.time.sleep(0.2)
#     flight_data_collector.capture()

### PLOTTING FUNCTIONS ###

def plotWaypoints(gates: List[swarms.GateObject], waypts: List[swarms.airsimneurips.Vector3r]):
    """
    Plots the spline path and gate positions in 3D space.

    Inputs:
    - gates (list): List of GateObject instances.
    - waypts (list): List of Vector3r waypoint objects.

    Outputs:
    - None (Displays 3D plot)
    """

    #### plot spline #####
    xarr, yarr, zarr = zip(*[(wp.x_val, wp.y_val, -1*wp.z_val) for wp in waypts])
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(xarr, yarr, zarr, label="Spline Path")
    #### plot gate positions #####
    for gate in gates:
        x, y, z = gate.x_pos, gate.y_pos, -1*gate.z_pos
        if (gate.gateNumber == 0):
            ax.scatter(x, y, z, color='green', label="Start Gate (gate 0)")
        elif (gate.gateNumber == 19):
            ax.scatter(x, y, z, color='red', label="End Gate (gate 19)")
        elif (gate.gateNumber == 1):
            ax.scatter(x, y, z, color='black', label="Regular Gate (gate 1-18)")
        else:
            ax.scatter(x, y, z, color='black')
    #### setup #####
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_zlabel("Z-axis")
    ax.set_title("3D Plot of Gates and Spline Path")
    ax.legend()
    plt.show()

def plot_magnitude_data(time_data: List[float], magnitude_data: List[float]) -> None:
    """
    Plots the signal magnitude over time.

    Inputs:
    - time_data (list): Timestamps
    - magnitude_data (list): Magnitudes at each time

    Outputs:
    - None (Displays plot)
    """
    timeframe_axis = [(t-time_data[0]) for t in time_data]
    swarms.plt.plot(timeframe_axis, magnitude_data, label="signal magnitude (m/s)", color='b')
    swarms.plt.xlabel("Time (sec)")
    swarms.plt.ylabel("Magnitude Vector Signal (m/s)")
    swarms.plt.title("PID Data")
    swarms.plt.legend()
    swarms.plt.grid()
    swarms.plt.show()

def plot_pid_data(time_data: List[float], err_data: List[float]) -> None:
    """
    Plots PID error over time.

    Inputs:
    - time_data (list): Timestamps
    - err_data (list): PID error at each timestamp

    Outputs:
    - None (Displays plot)
    """
    timeframe_axis = [(t-time_data[0]) for t in time_data]
    swarms.plt.plot(timeframe_axis, err_data, label="Error (m)", color='b')
    swarms.plt.xlabel("Time (sec)")
    swarms.plt.ylabel("Error Over Time (m)")
    swarms.plt.title("PID Error Response Data")
    swarms.plt.legend()
    swarms.plt.grid()
    swarms.plt.show()

### PERFORMANCE METRICS ###

def metric_1(delta_time: float) -> None:
    """
    Performance Metric #1
    Time-to-completion metric.

    Inputs:
    - delta_time (float): Total time in seconds.

    Outputs:
    - Printed result
    """
    elapsed = round(delta_time, 2)
    print("METRIC 1: "+str(elapsed)+" sec")

def metric_2(magnitudes: List[float]) -> None:
    """
    Performance Metric #2
    Average velocity metric.

    Inputs:
    - magnitudes (list): List of velocity magnitudes.

    Outputs:
    - Printed result
    """
    total_magntiudes = len(magnitudes)
    sum_magnitudes = sum(magnitudes)
    avg_magnitude = round((sum_magnitudes/total_magntiudes), 2)
    print("METRIC 2: "+str(round(avg_magnitude, 3))+" m/s")

def metric_3(error_arr: List[float]) -> None:
    """
    Performance Metric #3
    Navigation accuracy metric (mean error).

    Inputs:
    - error_arr (list): List of error values.

    Outputs:
    - Printed result
    """
    total_err = len(error_arr)
    sum_err = sum(error_arr)
    avg_error = round((sum_err/total_err), 2)
    print("METRIC 3: "+str(round(avg_error, 3))+" error deviation score")