import math
import numpy as np


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
        self.Kp_lin = params['Kp_lin']
        self.Ki_lin = params['Ki_lin']
        self.Kd_lin = params['Kd_lin']
        self.Kp_ang = params['Kp_ang']
        self.Ki_ang = params['Ki_ang']
        self.Kd_ang = params['Kd_ang']
        self.MAX_THRUST = params['MAX_THRUST']
        self.MIN_THRUST = params['MIN_THRUST']
        self.rot_thrust_weight = params['rot_thrust_weight']
        self.thrust_multiplier = params['thrust_multiplier']
        self.ship_lat_acc_pos_lim = params['ship_lat_acc_pos_lim']
        self.ship_lat_acc_neg_lim = params['ship_lat_acc_neg_lim']
        self.ship_ax_acc_lim = params['ship_ax_acc_lim']


        self.prev_outputs1 = []
        self.prev_outputs2 = []

        self.alpha = 0.3
        self.n = 10


    def pid_tune_velocity(self, ship_phys_status,cmd_vel,dt=1/60,log = False):
        # Unpack linear and angular velocities


        curr_lin_vel = np.linalg.norm(ship_phys_status['velocity'][0])
        curr_ang_vel = ship_phys_status['velocity'][1]
        #print(curr_ang_vel)

        #cmd_lin_vel = np.dot(cmd_vel[0], np.array([math.cos(ship_phys_status['pose'][1]), math.sin(ship_phys_status['pose'][1])]))
        cmd_lin_vel = np.linalg.norm(cmd_vel[0])

        cmd_ang_vel = cmd_vel[1]

        # Compute errors for linear and angular velocities
        error_lin_vel = cmd_lin_vel - curr_lin_vel
        error_ang_vel = cmd_ang_vel - curr_ang_vel

        # Create PID controllers for linear and angular velocities
        pid_lin = PIDController(self.Kp_lin, self.Ki_lin, self.Kd_lin)
        pid_ang = PIDController(self.Kp_ang, self.Ki_ang, self.Kd_ang)

        # Compute control outputs for linear and angular velocities
        accel_lin = pid_lin.compute(error_lin_vel, dt)*100
        accel_ang = pid_ang.compute(error_ang_vel, dt)

        if cmd_ang_vel == 0:
            accel_ang = 0
        else:
            accel_ang = pid_ang.compute(error_ang_vel, dt)

        if cmd_lin_vel == 0:
            accel_lin = pid_lin.compute(error_lin_vel, dt)*20
            accel_ang = 0
            if abs(curr_lin_vel) <= 0.01:
                accel_lin = 0


        accel_lin = max(min(accel_lin, self.ship_lat_acc_pos_lim), -self.ship_lat_acc_neg_lim)

        TRight = ((accel_lin*self.thrust_multiplier + (accel_ang*self.rot_thrust_weight)))
        TLeft = ((accel_lin*self.thrust_multiplier - (accel_ang*self.rot_thrust_weight)))



        # Limit the thrust values to the maximum and minimum values
        TRight = max(min(TRight, self.MAX_THRUST), self.MIN_THRUST)
        TLeft = max(min(TLeft, self.MAX_THRUST), self.MIN_THRUST)



        # Example usage

        #TLeft = self.filter1(TLeft)
        #TRight = self.filter2(TRight)

        #if log == True:
            #print("\rcmd_vel, TLeft, TRight", cmd_vel, TLeft, TRight, end=" ")

        return [TLeft, TRight]


    def filter1(self, input):
        if len(self.prev_outputs1) <= 9:
            # First input, output equals input
            self.output1 = input
        else:
            # Apply low-pass filter to input
            self.output1 = self.alpha * input + (1 - self.alpha) * np.mean(self.prev_outputs1[0:-1])

        # Update previous outputs
        if len(self.prev_outputs1) >= self.n:
            self.prev_outputs1.pop(0)

        self.prev_outputs1.append(self.output1)

        return self.output1

    def filter2(self, input):
        if len(self.prev_outputs2) <= 9:
            # First input, output equals input
            self.output2 = input
        else:
            # Apply low-pass filter to input
            self.output2 = self.alpha * input + (1 - self.alpha) * np.mean(self.prev_outputs2[0:-1])

        # Update previous outputs
        if len(self.prev_outputs2) >= self.n:
            self.prev_outputs2.pop(0)

        self.prev_outputs2.append(self.output2)

        return self.output2