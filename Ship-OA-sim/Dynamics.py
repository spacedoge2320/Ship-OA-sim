import pygame
import math
import numpy as np
import Guidance_algorithms
import Controllers


class water_dynamics():
    def __init__(self, params):
        self.mass = params['mass']
        self.Izz = params['Izz']
        self.inline_drag_coefficient = params['inline_drag_coefficient']
        self.sideways_drag_coefficient = params['sideways_drag_coefficient']
        self.rotational_drag_coefficient = params['rotational_drag_coefficient']
        self.T2CL_dist = params['T2CL_dist']

    def dynamics(self, ship_phys_status, TR, TL, dt=1 / 60):
        # this method simulates the dynamics of a boat. input = thrusts of the thrusters, output = velocity
        # cmd_acc = [[velx,vely], axial_vel] x is relative.

        self.ship_phys_status = ship_phys_status

        ship_pose = self.ship_phys_status['pose']
        ship_vel = self.ship_phys_status['velocity']

        vtheta = ship_vel[1]

        # Thrust
        # Heading 방향으로 작용하는 합력 하나랑 토크
        thrust_force = (TL + TR) * np.array([math.cos(ship_pose[1]), math.sin(ship_pose[1])])
        thrust_torque = (-TL + TR) * self.T2CL_dist

        # DRAG
        # 속도 벡터와 heading 방향의 단위벡터를 내적하여 진행방향 속력 도출, 다시 heading 방향의 벡터화, vel_heading)
        # heading 반대의 속도 도출, heading 반대방향의 벡터로 변환, vel_norm_heading)
        # vel_heading 에 대해 inline_drag_coefficient 의 항력 적용, vel_norm_heading 에 대해 sideways_drag_coefficient 항력 적용
        # 두 항력 값 합쳐서 합력 도출

        if(np.linalg.norm(ship_vel[0]) > 0.0):

            [vel_heading, vel_norm_heading] = [np.array([0,0]),np.array([0,0])]

            heading = np.array([math.cos(ship_pose[1]), math.sin(ship_pose[1])])
            heading_angle = self.regularize_angle(math.atan2(heading[1],heading[0]))
            velocity_angle = self.regularize_angle(math.atan2(ship_vel[0][1],ship_vel[0][0]))
            delta_angle = velocity_angle-heading_angle

            while delta_angle > 1 * math.pi:
                delta_angle = delta_angle - 2 * math.pi
            while delta_angle < -1 * math.pi:
                delta_angle = delta_angle + 2 * math.pi

            if abs(delta_angle) > math.pi/2:
                heading[0],heading[1] = -heading[0],-heading[1]

                delta_angle = velocity_angle - heading_angle

                while delta_angle > 1 * math.pi:
                    delta_angle = delta_angle - 2 * math.pi
                while delta_angle < -1 * math.pi:
                    delta_angle = delta_angle + 2 * math.pi



            spd_heading = math.cos(-delta_angle)*np.linalg.norm(ship_vel[0])
            if np.linalg.norm(ship_vel[0]) > 0.01:
                spd_norm_heading = math.sin(delta_angle) * np.linalg.norm(ship_vel[0])
            else:
                spd_norm_heading = 0



            if delta_angle < -0.01 :
                dir_norm_heading = heading_angle+math.pi/2
                #print("  mode 1",delta_angle)
            elif delta_angle > 0.01 :
                dir_norm_heading = heading_angle-math.pi/2
                #print(" mode 2")
            else :
                dir_norm_heading = 0
                spd_norm_heading = 0
                #print(" mode 3")

            dir_norm_heading_vector = np.array([math.cos(dir_norm_heading), math.sin(dir_norm_heading)])

            debug1 = ((heading*spd_heading*spd_heading))
            drag_inline_force = - (self.inline_drag_coefficient * ((heading*spd_heading*spd_heading)))
            drag_sideways_force = (self.sideways_drag_coefficient * ((dir_norm_heading_vector*spd_norm_heading*spd_norm_heading)))

            #if np.linalg.norm(drag_sideways_force) > 1000:
                #drag_sideways_force = np.array([0.0, 0.0])

            if np.linalg.norm(drag_inline_force) > 10000:
                drag_inline_force = np.array([0.0, 0.0])

            #rint("\ncmd_vel, TLeft, TRight", dir_norm_heading, dir_norm_heading_vector, spd_norm_heading, end="")

            #print("\ncmd_vel, TLeft, TRight", dir_norm_heading_vector,velocity_angle,delta_angle, TL, TR, end="")

        else:
            drag_sideways_force = np.array([0.0,0.0])
            drag_inline_force = np.array([0.0,0.0])

        drag_lat_force = drag_inline_force + drag_sideways_force


        if (abs(ship_vel[1]) > 0):
            drag_torque = -self.rotational_drag_coefficient * vtheta * abs(vtheta)
        else:
            drag_torque = 0


        # TOTAL FORCE
        total_force = thrust_force + drag_lat_force
        total_torque = thrust_torque + drag_torque


        # ACCELERATION
        lat_accel = total_force / self.mass
        ax_accel = total_torque / self.Izz

        # VELOCITY
        lat_vel = ship_vel[0] + lat_accel * dt
        ax_vel = ship_vel[1] + ax_accel * dt



        self.ship_phys_status['velocity'] = [lat_vel, ax_vel]

        return self.ship_phys_status

    def regularize_angle(self,angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle <= -math.pi:
            angle += 2 * math.pi

        return angle