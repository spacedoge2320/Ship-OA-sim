import math
import numpy as np
from scipy.spatial import ConvexHull

class LOS_guidance():
    def __init__(self, params):
        self.ship_max_speed = params['ship_max_speed']
        self.ship_ax_vel_lim = params['ship_ax_vel_lim']
        self.ship_lat_acc_pos_lim = params['ship_lat_acc_pos_lim']
        self.ship_lat_acc_neg_lim = params['ship_lat_acc_neg_lim']
        self.ship_ax_acc_lim = params['ship_ax_acc_lim']
        self.T_max_thrust = params['T_max_thrust']
        self.T_min_thrust = params['T_min_thrust']

    def ship_los_guidance(self, ship_phys_status, target_pos, dt=1 / 60):

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
        if distance_to_target > 2:
            ship_speed = self.ship_max_speed
        elif distance_to_target >= 0.6:
            ship_speed = self.ship_max_speed/3
        elif distance_to_target >= 0.3:
            ship_speed = self.ship_max_speed/8
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
        self.detection_range = 12
        self.spd_visual_multiplier = 5

        self.los = LOS_guidance(params)
        self.vo_polygons = []



        #parameters

    def ship_los_vo_guidance(self, phys_status, output_polygon, target_pos, sensor_data, dt=1 / 60):
        #   First, this receives a database of ships in the world. the database is a dictionary
        #   if there are no obstacles in range, it pursues the target using Line-of-sight guidance law
        #   if there are VO cones in sight, it tries to evade the obstacle by maintaining speed and turning.
        #   it prefers angles closer to the target.
        #   if no trajectory is valid, it tries to decelerate to half the maximum speed
        #   if there is still no trajectory, it tries to stop.


        #setting up variables
        self.phys_status = phys_status  # updates phys status
        self.output_polygon = output_polygon
        self.sensor_data = sensor_data  # get sensor_data
        self.vo_lines = []
        self.vo_circles = []  # position and radius
        self.vo_cones = []
        self.vo_polygons = []
        ship_position = phys_status['pose'][0]
        heading_in = phys_status['pose'][1]
        ship_speed_in = np.linalg.norm(phys_status['velocity'][0])
        ship_rot_spd_in = phys_status['velocity'][1]
        ship_heading = heading_in
        ship_rot_spd = ship_rot_spd_in
        ship_speed = 0

        # recieving and processing the data
        self.filtered_objects_angle = self.sensor_simulator()  # process the data
        self.vo_cone_generator() #generates VO cone

        # target_information
        distance_to_target = math.sqrt((target_pos[0] - ship_position[0]) ** 2 + (target_pos[1] - ship_position[1]) ** 2)
        target_vector = (target_pos - ship_position)
        target_angle = self.regularize_angle(math.atan2(target_vector[1], target_vector[0]))

        # search for valid angle
        target_speed = self.ship_max_speed
        self.angle_opacity = self.vo_theta_opacity(target_angle, target_speed)
        angle_rad = math.radians(6*self.index_with_lowest_number(self.angle_opacity))
        angle_variance = self.regularize_angle(angle_rad - ship_heading)

        if abs(angle_variance) > math.pi/4:
            target_speed = self.ship_max_speed/4
            self.angle_opacity = self.vo_theta_opacity(target_angle, target_speed)
            angle_rad = math.radians(6 * self.index_with_lowest_number(self.angle_opacity))
            angle_variance = self.regularize_angle(angle_rad - ship_heading)

            if abs(angle_variance) > math.pi/2:
                target_speed = self.ship_max_speed / 10
                self.angle_opacity = self.vo_theta_opacity(target_angle, target_speed)
                angle_rad = math.radians(6 * self.index_with_lowest_number(self.angle_opacity))
                angle_variance = self.regularize_angle(angle_rad - ship_heading)

        for theta in range(0,60):
            point = np.array(
                [target_speed * math.cos(math.radians(theta*6)), target_speed * math.sin(math.radians(theta*6))])

            if  self.angle_opacity[theta] < 1/4:
                self.vo_circles.append([self.phys_status["pose"][0]+point,0.05, (50,255,50), 1])



        rotational_mul = angle_variance

        if not (angle_variance >= -10 and angle_variance <= 10):
            angle_variance = 0

        if (angle_variance) > (math.pi / 2):
            rotational_mul = abs((math.pi / 2))
        elif (angle_variance) < -(math.pi / 2):
            rotational_mul = - abs((math.pi / 2))

        # rotational controller
        if abs(angle_variance) > 0.01 and abs(ship_rot_spd) <= abs(self.ship_ax_vel_lim):
            ship_rot_spd = -(rotational_mul / (math.pi / 2))*10
        else:
            ship_rot_spd = 0

        # translational controller
        if distance_to_target > 2:
            ship_speed = target_speed
        elif distance_to_target >= 0.6:
            ship_speed = target_speed/3
        elif distance_to_target >= 0.3:
            ship_speed = target_speed/8
        else:
            ship_speed = 0
            ship_rot_spd = 0

        ship_vel = ship_speed*np.array([math.cos(heading_in), math.sin(heading_in)])
        cmd_vel = [ship_vel, ship_rot_spd]

        return cmd_vel

    def vo_cone_generator(self):
        ship_pos = self.phys_status["pose"][0]
        ship_vel = self.phys_status["velocity"][0]
        ship_spd = abs(np.linalg.norm(self.phys_status["velocity"][0]))

        #vessel velocity indicator
        self.vo_lines.append([ship_pos,ship_pos+ship_vel*self.spd_visual_multiplier])

        circle_color = (200, 200, 200, 255)
        line_width = 1
        circle = [ship_pos, self.detection_range, circle_color, line_width]  # object circle position
        self.vo_circles.append(circle)  # circle representing collision distance of the object


        if self.filtered_objects_angle != None:
            for key, object in self.filtered_objects_angle.items():

                object_pos = object[0].phys_status["pose"][0]  # absolute object position
                object_vel = object[0].phys_status["velocity"][0]  # absolute object velocity
                #object_radius = object[1] + object[2]  # object VO_circle radius
                circle_color = (200, 0, 0, 255)
                line_width = 1
                #circle = [object_pos, object_radius, circle_color, line_width]  # object circle position
                #self.vo_circles.append(circle)  # circle representing collision distance of the object
                pos_diff = object_pos - ship_pos

                object_distance = abs(np.linalg.norm(pos_diff))  # distance from the object to the circle

                rel_spd = np.linalg.norm(object[0].phys_status["velocity"][0]-ship_vel)
                time_till_collision = object_distance/rel_spd  # estimated time till collision, assuming constant relative velocity

                if object_distance == 0:  # for avoiding divide by zero error
                    object_distance = 0.1

                start_rad = object[1]
                end_rad = object[2]

                object_velocity = object[0].phys_status["velocity"][0]

                #self.vo_cones.append([ship_pos, [start_rad, end_rad], object_distance*math.cos(tangent_angle)])

                self.vo_cones.append([ship_pos+object_velocity*self.spd_visual_multiplier, [start_rad, end_rad], object_distance])

                self.vo_polygons.append(self.outer_polygon)

            #print(math.degrees(self.filtered_objects))
        pass

    def vo_theta_opacity(self, target_angle, spd_to_search):

        # when search spd = max_spd
        spd_to_search
        angle_opacity = np.linspace(start=0, stop=60, num=60, endpoint=False)

        for theta in range(0,60):

            delta_angle = target_angle - math.radians(theta*6)

            while delta_angle >= 1 * math.pi:
                delta_angle = delta_angle - 2 * math.pi
            while delta_angle <= -1 * math.pi:
                delta_angle = delta_angle + 2 * math.pi

            angle_diff_opacity = ((delta_angle/(math.pi))**2)*(1/20)

            angle_opacity[theta] = angle_diff_opacity

        for vo_cone in self.vo_cones:
            for theta in range(0, 60):
                point = np.array([spd_to_search*math.cos(math.radians(theta*6)), spd_to_search*math.sin(math.radians(theta*6))])

                opacity = self.is_point_inside_circle(point+self.phys_status["pose"][0], vo_cone)

                if abs(opacity) > 1/3:
                    opacity = 100

                if opacity < 1/20:
                    opacity = 0

                angle_opacity[theta]=angle_opacity[theta]+opacity*2

        return angle_opacity

    def sensor_simulator_legacy(self):
        # receives dict(self.ships_database, **self.obstacle_database), which consists of dictionaries of objects(ship or obstacle).
        # extracts only {key, polygon} tuples using a for loop
        # then it filters only the keys of which its polygon is inside the detection range.
        # then it outputs {key: [polygon, phys_status]}  to a dictionary.
        ship_pose = self.phys_status["pose"]
        ship_polygon = self.output_polygon
        objects_database = self.sensor_data
        origin_point = ship_pose[0]

        if not(objects_database):
            return

        filtered_objects = {}

        #print(objects_database)

        for key, object in objects_database.items():
            # calculate distance between origin and closest point of polygon

            # print(key)
            object_pose = object.phys_status["pose"]
            object_vector = (object_pose[0] - ship_pose[0])
            object_distance = (np.linalg.norm(object_vector))
            #print(key, object_vector)

            # check if polygon is within detection range
            if object_distance <= self.detection_range:
                # gets object polygonal information
                poly_points = object.output_polygon
                # if there are no polygons, return nothing
                if poly_points == None:
                    return

                #print(poly_points)
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

    def sensor_simulator(self):
        # receives dict(self.ships_database, **self.obstacle_database), which consists of dictionaries of objects(ship or obstacle).
        # extracts only {key, polygon} tuples using a for loop
        # then it filters only the keys of which its polygon is inside the detection range.
        # then it outputs {key: [polygon, phys_status]}  to a dictionary.
        ship_pose = self.phys_status["pose"]
        ship_polygon = self.output_polygon
        objects_database = self.sensor_data
        origin_point = ship_pose[0]

        if not (objects_database):
            return

        filtered_objects_angle = {}

        # print(objects_database)

        for key, object in objects_database.items():
            # calculate distance between origin and closest point of polygon

            # print(key)
            object_pose = object.phys_status["pose"]
            object_vector = (object_pose[0] - ship_pose[0])
            object_distance = (np.linalg.norm(object_vector))
            # print(key, object_vector)

            # check if polygon is within detection range
            if object_distance <= self.detection_range:
                # gets object polygonal information
                poly_points = object.output_polygon
                # if there are no polygons, return nothing
                if poly_points == None:
                    return
                # print(poly_points)
                #(max_angle, min_angle) = self.get_min_max_angles(object)
                [max_angle_point, min_angle_point], [start_angle, end_angle], self.outer_polygon = self.inflate_obstacles(object)


                self.vo_lines.append([self.phys_status["pose"][0],max_angle_point])
                self.vo_lines.append([self.phys_status["pose"][0], min_angle_point])

                filtered_objects_angle[key] = [object, start_angle, end_angle]
                # print(" key, FOV object_radius self_radius  : ", object_distance, math.degrees(FOV), object_radius, self_radius)
        return filtered_objects_angle

    def get_min_max_angles(self,polygon):
        # Select the first point as the origin
        origin = self.phys_status["pose"][0]

        # Calculate the angle for each point relative to the origin
        angles = []
        for point in polygon:
            x, y = point[0] - origin[0], point[1] - origin[1]
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

    def inflate_obstacles(self,object):
        # Select the first point as the origin
        origin = self.phys_status["pose"][0]
        ship_polygon = self.output_polygon
        # Calculate the angle for each point relative to the origin
        self.vo_polygons = []

        object_vector = (object.phys_status["pose"][0] - self.phys_status["pose"][0])
        avg_angle = math.atan2(object_vector[1],object_vector[0])

        if avg_angle < 0:
            while avg_angle < 0:
                avg_angle += 2 * math.pi

        if avg_angle > 2 * math.pi:
            while avg_angle > 2:
                avg_angle -= 2 * math.pi

        #centers the ship polygon
        ship_polygon_centered = []
        for point in self.output_polygon:
            ship_polygon_centered.append((point - self.phys_status["pose"][0])*2)

        inflated_points = []
        for point in object.output_polygon:
            for point2 in ship_polygon_centered:
                inflated_point = point + point2
                inflated_points.append(inflated_point)

        hull = ConvexHull(inflated_points)
        outer_polygon = [inflated_points[i] for i in hull.vertices]

        angles = []
        for point in outer_polygon:
            x, y = point[0] - origin[0], point[1] - origin[1]
            vector = np.array([x,y])
            angle = math.atan2(y, x)

            if angle < 0:
                while angle < 0:
                    angle += 2 * math.pi

            if angle > 2 * math.pi:
                while angle >  2 * math.pi:
                    angle -= 2 * math.pi

            delta_angle = angle-avg_angle

            if delta_angle < - math.pi:
                while delta_angle < -math.pi:
                    delta_angle += 2 * math.pi

            if delta_angle > math.pi:
                while delta_angle > math.pi:
                    delta_angle -= 2 * math.pi


            angles.append(delta_angle)

        # Return the maximum and minimum angles
        max_angle = max(angles)
        min_angle = min(angles)


        max_angle_index = angles.index(max_angle)
        min_angle_index = angles.index(min_angle)

        max_angle = min(max(angles), math.pi*(3/4))
        min_angle = max(min(angles),-math.pi*(3/4))

        max_angle_point = outer_polygon[max_angle_index]
        min_angle_point = outer_polygon[min_angle_index]

        #print("\n",math.degrees(max_angle), math.degrees(min_angle),end="")
        #print("\n", polygon, end="")

        return ([max_angle_point,min_angle_point],[avg_angle+min_angle,avg_angle+max_angle],outer_polygon)

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

    def is_point_inside_circle(self, point, cone):
        """
        Determines whether a point is inside a circle section
        """
        origin,[start_angle, end_angle], radius = cone

        # Calculate angle of point relative to circle origin
        point_angle = math.atan2(point[1] - origin[1], point[0] - origin[0])

        start_angle = self.regularize_angle(start_angle)
        end_angle = self.regularize_angle(end_angle)
        point_angle = self.regularize_angle(point_angle)


        if end_angle >= start_angle:
            result = point_angle >= start_angle and point_angle <= end_angle
        elif end_angle <= start_angle: # 0도 넘어갈 때
            result = point_angle >= start_angle or point_angle <= end_angle

        if result == True:
            time_till_collision = radius / (math.dist(point, origin)*1)
            return 1/time_till_collision

        if result == False:
            return 0

    def angle_between_vector_and_angle(self, vector, angle):
        # Calculate the angle between the vector and the x-axis
        vector_angle = math.atan2(vector[1], vector[0])

        # Calculate the difference between the vector angle and the given angle
        angle_diff = vector_angle - angle

        # Ensure that the angle difference is within the range (-pi, pi]

        angle_diff = self.regularize_angle(angle_diff)

        # Return the absolute value of the angle difference
        return angle_diff

    def index_with_lowest_number(self, numbers):
        """
        Returns the index of the lowest number in a list
        """
        lowest_index = 0
        for i in range(1, len(numbers)):
            if numbers[i] < numbers[lowest_index]:
                lowest_index = i
        return lowest_index


    def regularize_angle(self,angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle <= -math.pi:
            angle += 2 * math.pi

        return angle

