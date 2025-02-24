import imports as swarms
import numpy as np
from scipy.optimize import minimize

# MODULE 7: PID Control

def getDronePos():
    pos = swarms.client.getMultirotorState(vehicle_name="drone_1").kinematics_estimated.position
    return pos.x_val, pos.y_val, pos.z_val

class controller_PID:
    def __init__(self, kp, ki, kd):
        # setting controller parameters
        self.kp = kp
        self.ki = ki
        self.kd = kd
        # initializing prev error and integral values
        self.prevError_x, self.prevError_y, self.prevError_z = 0, 0, 0
        self.integral_x, self.integral_y, self.integral_z = 0, 0, 0
        self.prev_time = swarms.time.time()
        # plotting variables
        self.display_time = []
        self.display_magnitude = []
        self.display_errors = []

    def compute_correction(self, target_x, target_y, target_z):
        # Get current drone position
        current_x, current_y, current_z = getDronePos()

        # Compute delta time (dt)
        current_time = swarms.time.time()
        delta_time = current_time - self.prev_time
        self.prev_time = current_time

        # Compute PID corrections
        vel_x, self.prevError_x, self.integral_x = PID_drift_controller(
            self.kp, self.ki, self.kd, target_x, current_x, self.integral_x, self.prevError_x, delta_time
        )
        vel_y, self.prevError_y, self.integral_y = PID_drift_controller(
            self.kp, self.ki, self.kd, target_y, current_y, self.integral_y, self.prevError_y, delta_time
        )
        vel_z, self.prevError_z, self.integral_z = PID_drift_controller(
            self.kp, self.ki, self.kd, target_z, current_z, self.integral_z, self.prevError_z, delta_time
        )

        # potentially add clipping to ensure no excessive changes anywhere
        # Clip corrections to avoid excessive movement
        #MAX_CORRECTION_LIMIT = 5
        #MAX_CORRECTION_LIMIT = max(2.0, min(5.0, abs(target_x - current_x) * 0.5))  # Adjust this based on testing
        # CORRECTION_THRESHOLD = 5.0
        # vel_x = max(-CORRECTION_THRESHOLD, min(CORRECTION_THRESHOLD, vel_x))
        # vel_y = max(-CORRECTION_THRESHOLD, min(CORRECTION_THRESHOLD, vel_y))
        # vel_z = max(-CORRECTION_THRESHOLD, min(CORRECTION_THRESHOLD, vel_z))
        # INTEGRAL_THRESHOLD = 10.0
        # self.integral_x = max(-INTEGRAL_THRESHOLD, min(INTEGRAL_THRESHOLD, self.integral_x))
        # self.integral_y = max(-INTEGRAL_THRESHOLD, min(INTEGRAL_THRESHOLD, self.integral_y))
        # self.integral_z = max(-INTEGRAL_THRESHOLD, min(INTEGRAL_THRESHOLD, self.integral_z))



        # update metric (magnitude)
        magnitude = np.sqrt(vel_x**2+vel_y**2+vel_z**2)
        self.display_magnitude.append(magnitude)
        
        # update metric (x-axis/time)
        self.display_time.append(current_time)

        # update metric (y-axis/error)
        err_x = abs(target_x-current_x)
        err_y = abs(target_y-current_y)
        err_z = abs(target_z-current_z)
        err_total = err_x+err_y+err_z
        self.display_errors.append(err_total)

        return vel_x, vel_y, vel_z

    # Dynamic cost-based PID tuning algorithm
    # Adapted from https://github.com/simorxb/PID-tuning-opt/tree/main
    def tune_pid(self):
        print("----- Tuning PID -----") 

        # swarms.apply_ziegler()

        print("NEW VALUES: "+str(self.kp)+", "+str(self.ki)+", "+str(self.kd))


def PID_drift_controller(kp, ki, kd, _setPoint, _currentPoint, integral, prevError, delta_time):
    target_position = _setPoint
    current_position = _currentPoint

    # step 1. calculate error
    error = target_position - current_position

    # step 2. calculate proporational term (p_term)
    p_term = kp * error

    # step 3. calculate integral term (i_term)
    integral += ki*error*delta_time # updates/accumulates the error over time

    # step 4. calculate derivative term (d_term)
    d_term = 0
    if delta_time > 0: # prevent divide by 0 at start
        delta_error = error - prevError
        d_term = kd*delta_error/delta_time # determine the derivative value needed for updating the PID output value

    # step 5. calculate manipulated variable or correction value (MV)
    MV_correction_value = p_term+integral+d_term

    # step 6. return MV and values to be updated
    return MV_correction_value, error, integral
