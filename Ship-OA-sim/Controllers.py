import math
import numpy as np


class LowPassFilter:
    def __init__(self, alpha, n):
        self.alpha = alpha
        self.n = n
        self.prev_outputs = []

    def filter(self, input):
        if len(self.prev_outputs) == 0:
            # First input, output equals input
            output = input
        else:
            # Apply low-pass filter to input
            output = self.alpha * input + (1 - self.alpha) * self.prev_outputs[-1]

        # Update previous outputs
        if len(self.prev_outputs) >= self.n:
            self.prev_outputs.pop(0)
        self.prev_outputs.append(output)

        return output



class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.integral = 0.0


    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output



class Pid_vel_controller():
    # Receives CMD_VEL and outputs translational and angular accelerations
    def __init__(self, params):
        self.prior_thrust = [0, 0]
        self.Kp_lin  = params['Kp_lin']
        self.Ki_lin = params['Ki_lin']
        self.Kd_lin = params['Kd_lin']
        self.Kp_ang = params['Kp_ang']
        self.Ki_ang = params['Ki_ang']
        self.Kd_ang = params['Kd_ang']
        self.MAX_THRUST = params['MAX_THRUST']
        self.MIN_THRUST = params['MIN_THRUST']
        self.rot_thrust_weight = params['rot_thrust_weight']
        self.thrust_multiplier = params['thrust_multiplier']


    def pid_tune_velocity(self, ship_phys_status,cmd_vel,dt=1/144,log = False):
        # Unpack linear and angular velocities


        curr_lin_vel = np.linalg.norm(ship_phys_status['velocity'][0])
        curr_ang_vel = ship_phys_status['velocity'][1]
        cmd_lin_vel = np.linalg.norm(cmd_vel[0])
        cmd_ang_vel = cmd_vel[1]

        # Compute errors for linear and angular velocities
        error_lin_vel = cmd_lin_vel - curr_lin_vel
        error_ang_vel = cmd_ang_vel - curr_ang_vel

        # Create PID controllers for linear and angular velocities
        pid_lin = PIDController(self.Kp_lin, self.Ki_lin, self.Kd_lin)
        pid_ang = PIDController(self.Kp_ang, self.Ki_ang, self.Kd_ang)

        # Compute control outputs for linear and angular velocities
        accel_lin = pid_lin.compute(error_lin_vel, dt)*0.1
        accel_ang = pid_ang.compute(error_ang_vel, dt)*2


        TRight = ((accel_lin + (0.8 / 2 * accel_ang*self.rot_thrust_weight)) / 2)*self.thrust_multiplier
        TLeft = ((accel_lin - (0.8 / 2 * accel_ang*self.rot_thrust_weight)) / 2)*self.thrust_multiplier

        # Limit the thrust values to the maximum and minimum values
        TRight = max(min(TRight, self.MAX_THRUST), self.MIN_THRUST)
        TLeft = max(min(TLeft, self.MAX_THRUST), self.MIN_THRUST)

        Thrust = [TRight,TLeft]



        """
        Diff_x = TRight - self.prior_thrust[1]
        Diff_y = TLeft - self.prior_thrust[0]

        Diff_x = max(min(Diff_x , MAX_thrust_diff), -MAX_thrust_diff)
        Diff_y = max(min(Diff_y, MAX_thrust_diff), -MAX_thrust_diff)

        TRight = self.prior_thrust[1] + Diff_x
        TLeft = self.prior_thrust[0] + Diff_y
        """

        #if log == True:
         #   print("\rTLeft, TRight", TLeft, TRight, end="")

        # Example usage
        left_filter = LowPassFilter(alpha=10, n=20)

        right_filter = LowPassFilter(alpha=10, n=20)

        TLeft = left_filter.filter(TLeft)
        TRight = right_filter.filter(TRight)


        return [TLeft, TRight]