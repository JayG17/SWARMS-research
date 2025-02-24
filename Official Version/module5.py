import imports as swarms
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# MODULE 5: Drift Handling Functions, Performance Metrics, Plotting Functions, Additional Functions

### PLOTTING
def plotWaypoints(gates, waypts):
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

def plot_magnitude_data(time_data, magnitude_data):
    timeframe_axis = [(t-time_data[0]) for t in time_data]
    swarms.plt.plot(timeframe_axis, magnitude_data, label="signal magnitude (m/s)", color='b')
    swarms.plt.xlabel("Time (sec)")
    swarms.plt.ylabel("Magnitude Vector Signal (m/s)")
    swarms.plt.title("PID Data")
    swarms.plt.legend()
    swarms.plt.grid()
    swarms.plt.show()

def plot_pid_data(time_data, err_data):
    timeframe_axis = [(t-time_data[0]) for t in time_data]
    swarms.plt.plot(timeframe_axis, err_data, label="Error (m)", color='b')
    swarms.plt.xlabel("Time (sec)")
    swarms.plt.ylabel("Error Over Time (m)")
    swarms.plt.title("PID Error Response Data")
    swarms.plt.legend()
    swarms.plt.grid()
    swarms.plt.show()

### PERFORMANCE METRICS

# Time-to-Completion
def metric_1(delta_time):
    elapsed = round(delta_time, 2)
    print("METRIC 1: "+str(elapsed)+" sec")

# Average Velocity
def metric_2(magnitudes):
    total_magntiudes = len(magnitudes)
    sum_magnitudes = sum(magnitudes)
    avg_magnitude = round((sum_magnitudes/total_magntiudes), 2)
    print("METRIC 2: "+str(round(avg_magnitude, 3))+" m/s")

# Navigation Ability
def metric_3(error_arr):
    total_err = len(error_arr)
    sum_err = sum(error_arr)
    avg_error = round((sum_err/total_err), 2)
    print("METRIC 3: "+str(round(avg_error, 3))+" error deviation score")