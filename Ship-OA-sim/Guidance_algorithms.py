import math
import numpy as np

class LOS_guidance():
    def __init__(self, params):
        self.ship_max_speed = params['ship_max_speed']
        self.ship_ax_vel_lim = params['ship_ax_vel_lim']
        self.ship_lat_acc_pos_lim = params['ship_lat_acc_pos_lim']
        self.ship_lat_acc_neg_lim = params['ship_lat_acc_neg_lim']
        self.ship_ax_acc_lim = params['ship_ax_acc_lim']
        self.T_max_thrust = params['T_max_thrust']
        self.T_min_thrust = params['T_min_thrust']

    def ship_los_guidance(self, ship_phys_status, target_pos, dt=1 / 144):

        ship_position = ship_phys_status['pose'][0]
        heading_in = ship_phys_status['pose'][1]
        ship_speed_in = np.linalg.norm(ship_phys_status['velocity'][0])
        ship_rot_spd_in = ship_phys_status['velocity'][1]

        def angle_between_vector_and_angle(vector, angle):
            # Calculate the angle between the vector and the x-axis
            vector_angle = math.atan2(vector[1], vector[0])

            # Calculate the difference between the vector angle and the given angle
            angle_diff = vector_angle - angle

            # Ensure that the angle difference is within the range (-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff <= -math.pi:
                angle_diff += 2 * math.pi

            # Return the absolute value of the angle difference
            return angle_diff

        ship_heading = heading_in
        ship_rot_spd = ship_rot_spd_in
        ship_speed = 0

        distance_to_target = math.sqrt(
            (target_pos[0] - ship_position[0]) ** 2 + (target_pos[1] - ship_position[1]) ** 2)

        if not (distance_to_target >= -1 and distance_to_target <= 10000):
            angle_variance = 0


        target_angle = (target_pos - ship_position)

        angle_variance = angle_between_vector_and_angle(target_angle, ship_heading)

        rotational_mul = angle_variance

        if not (angle_variance >= -10 and angle_variance <= 10):
            angle_variance = 0

        if (angle_variance) > (math.pi / 4):
            rotational_mul = abs((math.pi / 2))
        elif (angle_variance) < -(math.pi / 2):
            rotational_mul = - abs((math.pi / 2))

        # rotational controller
        if abs(angle_variance) > 0.01 and abs(ship_rot_spd) <= abs(self.ship_ax_vel_lim):
            ship_rot_spd = -(rotational_mul / (math.pi / 2))*10
        else:
            ship_rot_spd = 0

            # translational controller
        if distance_to_target > 3:
            ship_speed = self.ship_max_speed
        elif distance_to_target >= 1:
            ship_speed = self.ship_max_speed/2
        elif distance_to_target >= 0.5:
            ship_speed = self.ship_max_speed/4
        else:
            ship_speed = 0
            ship_rot_spd = 0



        ship_vel = ship_speed*np.array([math.cos(heading_in), math.sin(heading_in)])

        cmd_vel = [ship_vel, ship_rot_spd]

        #print("\rangle_variance, ship_rot_spd", np.degrees(heading_in), ship_rot_spd, end="")
        #print("\rTLeft, TRight", rTLeft.TRight, end="")


        return cmd_vel

class LOS_VO_guidance():

    def __init__(self, params):

        self.ship_max_speed = params['ship_max_speed']
        self.ship_ax_vel_lim = params['ship_ax_vel_lim']
        self.ship_lat_acc_pos_lim = params['ship_lat_acc_pos_lim']
        self.ship_lat_acc_neg_lim = params['ship_lat_acc_neg_lim']
        self.ship_ax_acc_lim = params['ship_ax_acc_lim']
        self.T_max_thrust = params['T_max_thrust']
        self.T_min_thrust = params['T_min_thrust']
        self.detection_range = 7
        self.spd_visual_multiplier = 5

        self.los = LOS_guidance(params)


        #parameters


    def ship_los_vo_guidance(self, phys_status, output_polygon, target_pos, sensor_data, dt=1 / 144):

        self.phys_status = phys_status # updates phys status
        self.output_polygon = output_polygon
        self.sensor_data = sensor_data # get sensor_data
        self.filtered_objects = self.sensor_simulator() # process the data

        #output_polygon =

        self.vo_circles = [] #position and radius
        self.vo_cones = []

        self.vo_cone_generator()




        #self.vo_cone =

        guidance_params = {
            "initial_ship_speed": 0.0,
            "ship_max_speed": 3.0,
            "ship_ax_vel_lim": 0.9,
            "ship_lat_acc_pos_lim": 0.9,
            "ship_lat_acc_neg_lim": 0.05,
            "ship_ax_acc_lim": 0.15,
            "T_max_thrust": self.T_max_thrust,
            "T_min_thrust": self.T_min_thrust,
            "dt": 1 / 144
        }

        cmd_vel = self.los.ship_los_guidance(self.phys_status, target_pos=target_pos, dt=1 / 144)


        return cmd_vel




    def behavior_tree (self):
        # First, this receives a database of ships in the world. the database is a dictionary
        # if there are no obstacles in range, it pursues the target using Line-of-sight guidance law
        # if there are VO cones in sight, it tries to evade the obstacle by maintaining speed and turning.
        #   it prefers angles closer to the target.
        #   if no trajectory is possible, it tries to decelerate
        #   it tries to rotate when it can not decelerate enough
        # it finally outputs CMD_vel, to be subscribed by
        pass

    def vo_cone_generator(self):
        ship_pos = self.phys_status["pose"][0]
        ship_vel = self.phys_status["velocity"][0]
        ship_spd = abs(np.linalg.norm(self.phys_status["velocity"][0]))

        self.vo_lines = []
        self.vo_lines.append([ship_pos,ship_pos+ship_vel*self.spd_visual_multiplier])

        circle_color = (200, 200, 200, 255)
        line_width = 1
        circle = [ship_pos, ship_spd*self.spd_visual_multiplier, circle_color, line_width]  # object circle position
        self.vo_circles.append(circle)  # circle representing collision distance of the object


        if self.filtered_objects != None:
            for key, object in self.filtered_objects.items():

                object_pos = object[0].phys_status["pose"][0]  # absolute object position
                object_vel = object[0].phys_status["velocity"][0]  # absolute object velocity
                object_radius = object[1] + object[2]  # object VO_circle radius
                circle_color = (200, 0, 0, 255)
                line_width = 1
                circle = [object_pos, object_radius, circle_color, line_width]  # object circle position
                self.vo_circles.append(circle)  # circle representing collision distance of the object
                pos_diff = object_pos - ship_pos

                object_distance = abs(np.linalg.norm(pos_diff))  # distance from the object to the circle

                rel_spd = np.linalg.norm(object[0].phys_status["velocity"][0]-ship_vel)
                time_till_collision = object_distance/rel_spd  # estimated time till collision, assuming constant relative velocity

                if object_distance == 0:  # for avoiding divide by zero error
                    object_distance = 0.1

                if object_radius >= object_distance:  # for avoiding divide by zero error
                    object_radius = object_distance - 0.0001

                tangent_angle = math.asin(object_radius/object_distance)

                object_angle = - math.atan2(pos_diff[0], pos_diff[1]) + math.pi/2

                start_rad = object_angle - tangent_angle
                end_rad = object_angle + tangent_angle

                object_velocity = object[0].phys_status["velocity"][0]

                #self.vo_cones.append([ship_pos, [start_rad, end_rad], object_distance*math.cos(tangent_angle)])

                self.vo_cones.append([ship_pos+object_velocity*self.spd_visual_multiplier, [start_rad, end_rad], object_distance])
                self.vo_circles.append(circle)
        pass


    def vo_theta_opacity(self):

        # when search spd = max_spd

        search_spd = self.ship_max_speed

        for vo_cone in self.vo_cones:
            angles = np.linspace(start=0, stop=2*math.pi, num=360, endpoint=False)
            for theta in angles:
                point = np.array([search_spd*math.cos(theta), search_spd*math.sin(theta)])









    def sensor_simulator(self):
        # receives dict(self.ships_database, **self.obstacle_database), which consists of dictionaries of objects(ship or obstacle).
        # extracts only {key, polygon} tuples using a for loop
        # then it filters only the keys of which its polygon is inside the detection range.
        # then it outputs {key: [polygon, phys_status]}  to a dictionary.
        ship_pose = self.phys_status["pose"]
        ship_polygon = self.output_polygon
        objects_database = self.sensor_data
        origin_point = ship_pose[0]



        filtered_objects = {}

        #print(objects_database)

        for key, object in objects_database.items():
            # calculate distance between origin and closest point of polygon
            poly_points = object.output_polygon
            if poly_points == None:
                return
            # print(key)
            object_pose = object.phys_status["pose"]
            object_distance = (np.linalg.norm(object_pose[0] - ship_pose[0]))
            object_vector = (object_pose[0] - ship_pose[0])

            #print(key, object_distance)

            # check if polygon is within detection range
            if object_distance <= self.detection_range:
                # print(poly_points)
                (max_angle, min_angle) = self.get_min_max_angles(poly_points)
                FOV = (max_angle - min_angle)

                while FOV > 1 * math.pi:
                    FOV = 2 * math.pi - FOV
                while FOV <= 0:
                    FOV += 2 * math.pi

                object_radius = (object_distance * math.tan(FOV / 2))
                self_radius = (self.get_largest_inner_product(ship_polygon, object_vector))/2

                filtered_objects[key] = [object, object_radius, self_radius ]
                #print(" key, FOV object_radius self_radius  : ", object_distance, math.degrees(FOV), object_radius, self_radius)




        return filtered_objects

    def get_min_max_angles(self,polygon):
        # Select the first point as the origin
        origin = self.phys_status["pose"][0]

        # Calculate the angle for each point relative to the origin
        angles = []
        for point in polygon:
            x, y = point[0]/40 - origin[0], point[1]/40 - origin[1]
            angle = math.atan2(y, x)
            if angle < 0:
                while angle < 0:
                    angle += 2 * math.pi

            if angle > 2 * math.pi:
                while angle > 2:
                    angle -= 2 * math.pi
            angles.append(angle)

        # Return the maximum and minimum angles
        max_angle = max(angles)
        min_angle = min(angles)

        #print("\n",math.degrees(max_angle), math.degrees(min_angle),end="")
        #print("\n", polygon, end="")

        return (max_angle, min_angle)

    def get_largest_inner_product(self,polygon, B):
        # Rotate B by pi/2
        B_rotated = np.array([-B[1], B[0]])

        # Normalize B_rotated
        A = B_rotated / np.linalg.norm(B_rotated)

        max_inner_product = -float('inf')

        for i, point1 in enumerate(polygon):
            for j, point2 in enumerate(polygon[i + 1:], i + 1):
                # Calculate the line vector between point1 and point2
                line_vector = np.array(point2) - np.array(point1)

                # Calculate the inner product of line_vector and A
                inner_product = np.dot(line_vector / np.linalg.norm(line_vector), A)

                # Update the max inner product if applicable
                if inner_product > max_inner_product:
                    max_inner_product = inner_product

        # Return the largest inner product
        return max_inner_product

    def tangent_lines(self, circle, origin):
        # Unpack circle coordinates and radius
        x_c, y_c = circle[0]
        r = circle[1]

        # Unpack origin coordinates
        x_o, y_o = origin

        # Calculate the distance between the origin and the center of the circle
        d = math.sqrt((x_c - x_o) ** 2 + (y_c - y_o) ** 2)

        # Check if the origin is inside the circle
        if d < r:
            print("Error: origin is inside the circle.")
            return None

        # Calculate the angle between the origin and the center of the circle
        theta = math.atan2(y_c - y_o, x_c - x_o)

        # Calculate the distance from the origin to the tangent point
        a = math.sqrt(d ** 2 - r ** 2)

        # Calculate the angles of the tangent lines
        alpha = math.asin(r / d)
        beta = theta - alpha
        gamma = theta + alpha

        # Return the two angles of the tangent lines
        return beta, gamma

    import math

    def is_point_inside_circle(self, point, circle):
        """
        Determines whether a point is inside a circle section
        """
        origin, start_angle, end_angle, radius = circle

        # Calculate angle of point relative to circle origin
        point_angle = math.atan2(point[1] - origin[1], point[0] - origin[0])
        if point_angle < 0:
            point_angle += 2 * math.pi

        # Check if point is within circle radius
        if math.dist(point, origin) > radius:
            result = False

            # Check if point is within circle arc
        if start_angle < end_angle:
            result = start_angle <= point_angle <= end_angle
        else:
            result = start_angle <= point_angle or point_angle <= end_angle


        if result == True:
            time_till_collision = radius / (math.dist(point, origin) / self.spd_visual_multiplier)
            return 1/time_till_collision
        if result == False:
            return  0