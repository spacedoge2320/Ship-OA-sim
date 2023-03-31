import pygame
import math
import numpy as np
import Ship_class
import Obstacle_class
import threading
import time

class Logic:
    # This will run in another thread
    def __init__(self, size, speed=2):
        # Private fields -> Only to be edited locally
        self._size = size
        self._direction = np.array([0, 1])  # [x, y] vector, underscored because we want this to be private
        self._speed = speed

        # Threaded fields -> Those accessible from other threads
        self.position = np.array(size) / 2
        self.input_list = []  # A list of commands to queue up for execution

        self.ships_database = {
            "main_ship": Ship_class.USV(initial_ship_pos_scaled=np.array([2.0, 12.0]), ship_heading=0,
                                        target_pos_scaled=np.array([1.0, 20.0]), ship_color=self.color_ship1,
                                        guidance_type=2),
            "ship1": Ship_class.USV(initial_ship_pos_scaled=np.array([46.0, 14.0]), ship_heading=3.14,
                                    target_pos_scaled=np.array(self.rescaler2([2.0, 12.0])), ship_color=self.color_ship2, guidance_type=2),
            #"ship2": Ship_class.USV(initial_ship_pos_scaled=np.array([46.0, 8.0]), ship_heading=3.14,
             #                       target_pos_scaled=np.array(self.rescaler2([2.0, 12.0])),
              #                      ship_color=self.color_ship2, guidance_type=2),
            #"ship3": Ship_class.USV(initial_ship_pos_scaled=np.array([46.0, 2.0]), ship_heading=3.14,
             #                       target_pos_scaled=np.array(self.rescaler2([2.0, 12.0])),
              #                      ship_color=self.color_ship2, guidance_type=2)
        }

        # A lock ensures that nothing else can edit the variable while we're changing it
        self.lock = threading.Lock()

    def _loop(self):
        time.sleep(0.5)  # Wait a bit to let things load
        # We're just going to kill this thread with the main one so it's fine to just loop forever
        while True:
            # Check for commands
            time.sleep(0.01)  # Limit the logic loop running to every 10ms

            if len(self.input_list) > 0:

                with self.lock:  # The lock is released when we're done
                    # If there is a command we pop it off the list
                    key = self.input_list.pop(0).key

                if key == pygame.K_w:
                    self._direction = np.array([0, -1])
                elif key == pygame.K_a:
                    self._direction = np.array([-1, 0])
                elif key == pygame.K_s:
                    self._direction = np.array([0, 1])
                elif key == pygame.K_d:
                    self._direction = np.array([1, 0])

            with self.lock:  # Again we call the lock because we're editing
                self.position += self._direction * self._speed

            if self.position[0] < 0 \
                    or self.position[0] > self._size[0] \
                    or self.position[1] < 0 \
                    or self.position[1] > self._size[1]:
                break  # Stop updating

    def start_loop(self):
        # We spawn a new thread using our _loop method, the loop has no additional arguments,
        # We call daemon=True so that the thread dies when main dies
        threading.Thread(target=self._loop,
                         args=(),
                         daemon=True).start()



class Simulation1:
    def __init__(self):
        pygame.init()
        # set up the window
        self.screen_width, self.screen_height = 1920, 1080
        flags = pygame.FULLSCREEN
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height),vsync=0)
        pygame.display.set_caption('SHIP SIM V1.0.0')

        # set up the clock
        self.clock = pygame.time.Clock()
        self.tick = 60

        # define colors
        self.color_path = (200, 200, 200)
        self.color_sea = (0, 119, 190)
        self.color_ship1 = (211, 211, 211)
        self.color_ship2 = (150, 51, 50)
        self.color_marker = (250, 250, 250)
        self.color_obstacle = (50, 50, 50)

        self.vo_layer = pygame.Surface((self.screen_width, self.screen_height), pygame.SRCALPHA)
        self.vo_layer.set_alpha(255)  # alpha level

        # initialize ships
        self.ships_database = {
            "main_ship": Ship_class.USV(initial_ship_pos_scaled=np.array([2.0, 12.0]), ship_heading=0,
                                        target_pos_scaled=np.array([1.0, 20.0]), ship_color=self.color_ship1,
                                        guidance_type=2),
            #"ship1": Ship_class.USV(initial_ship_pos_scaled=np.array([1.0, 20.0]), ship_heading=0.0,
                                #    target_pos_scaled=np.array([20.0, 1.0]), ship_color=self.color_ship2, guidance_type=2)
        }

        # initialize obstacles
        self.obstacle_database = {
            "obstacle_1": Obstacle_class.wall(initial_pos_scaled=np.array([12.0, 6.0]), heading=3.14 / 2,
                                              obstacle_color=self.color_obstacle),
            "obstacle_2": Obstacle_class.wall(initial_pos_scaled=np.array([20.0, 20.0]), heading=3.14 / 2,
                                              obstacle_color=self.color_obstacle),
            "obstacle_3": Obstacle_class.wall(initial_pos_scaled=np.array([28.0, 6.0]), heading=3.14 / 2,
                                              obstacle_color=self.color_obstacle),
            "obstacle_4": Obstacle_class.wall(initial_pos_scaled=np.array([36.0, 20.0]), heading=3.14 / 2,
                                              obstacle_color=self.color_obstacle),
            "obstacle_5": Obstacle_class.wall(initial_pos_scaled=np.array([20.0, 0.7]), heading=0,
                                              obstacle_color=self.color_obstacle),
            "obstacle_6": Obstacle_class.wall(initial_pos_scaled=np.array([28.0, 26.5]), heading=0,
                                              obstacle_color=self.color_obstacle),



        }
        self.font = pygame.font.SysFont('Arial', 24)

    def loop(self):
        # set up the game loop
        target_pos = [1800, 100]

        running = True
        while running:

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONUP:
                    target_pos = pygame.mouse.get_pos()

            self.screen.fill(self.color_sea)  # fills the background
            self.vo_layer.fill((255, 255, 255, 0))  # fills the background

            object_database = self.object_publisher()  # returns info of the objects(ships and obstacles) in the world.

            for key, obstacle in list(self.obstacle_database.items()):
                obstacle.polygon()
                self.draw_obstacle(obstacle)

            # Draw ships
            for key, ship_obj in list(self.ships_database.items()):
                original_object_database = object_database
                # print("object_database",object_database)
                # del object_database[key]
                sensor_data = object_database
                # print("sensor_data",sensor_data)
                object_database = original_object_database

                if key == "main_ship":
                    ship_obj.move_ship(target_pos_scaled=self.rescaler2(target_pos), sensor_data=sensor_data, log=True)
                else:
                    ship_obj.move_ship(sensor_data=sensor_data)

                if key == "main_ship":
                    self.draw_ship(ship_obj)

                self.visualize_vo(ship_obj)

            # Draw obstacles
            self.draw_target(target_pos)

            fps = self.clock.get_fps()
            fps_text = self.font.render(f"FPS: {fps:.2f}", True, (255, 255, 255))

            # Draw the FPS counter to the screen
            self.screen.blit(fps_text, (10, 10))

            # update the screen
            pygame.display.flip()
            # wait for the next frame
            self.clock.tick(self.tick)

        pygame.quit()

    def draw_target(self, target_pos):
        pygame.draw.line(self.screen, self.color_marker, (target_pos[0] - 10, target_pos[1]),
                         (target_pos[0] + 10, target_pos[1]), 2)
        pygame.draw.line(self.screen, self.color_marker, (target_pos[0], target_pos[1] - 10),
                         (target_pos[0], target_pos[1] + 10), 2)

    def draw_ship(self, ship_obj):
        # draw ship path
        ship_pos_list = ship_obj.ship_pos_list

        scaled_points2 = []

        for points in ship_pos_list:
            scaled_points2.append(self.rescaler(points))

        pygame.draw.aalines(self.screen, self.color_path, False, scaled_points2, 1)

        scaled_points = []
        for points in ship_obj.output_polygon:
            scaled_points.append(self.rescaler(points))

        # draw ship polygon
        pygame.draw.polygon(self.screen, ship_obj.ship_color, scaled_points)

    def visualize_vo(self, ship_obj):

        # draw VO circle
        # print("ship_obj",ship_obj.vo_circle)
        if ship_obj.vo_circles != None:

            for circle in ship_obj.vo_circles:
                pygame.draw.circle(self.vo_layer, circle[2], self.rescaler(circle[0]), circle[1] * 40, circle[3])
            # draw VO cone
            # pygame.draw.polygon(self.screen, (100, 0, 0), ship_obj.vo_cone)
            for cone in ship_obj.vo_cones:
                # pygame.draw.line(self.screen, (255, 0, 0), cone[0] * 40, (cone[0] + np.array([math.cos(cone[1][0])*cone[2], math.cos(cone[1][0])*cone[2]]))*40, 1)
                # VO cone
                polygon_points = [cone[0]]
                arc_angle = np.linspace(start=cone[1][0], stop=cone[1][1], num=60, endpoint=False)

                for angle in arc_angle:
                    polygon_points.append(
                        np.array([cone[0][0] + math.cos(angle) * cone[2], cone[0][1] + math.sin(angle) * cone[2]]))
                scaled_points = []
                for points in polygon_points:
                    scaled_points.append(self.rescaler(points))

                #pygame.draw.polygon(self.vo_layer, (255, 153, 0, 120), scaled_points)
                pygame.draw.polygon(self.vo_layer, (255, 153, 0, 255), scaled_points, 2)

            for lines in ship_obj.vo_lines:
                pygame.draw.line(self.vo_layer, lines[2], self.rescaler(lines[0]), self.rescaler(lines[1]), 2)

            for polygon in ship_obj.vo_polygons:
                scaled_points = []
                for points in polygon:
                    scaled_points.append(self.rescaler(points))
                pygame.draw.polygon(self.vo_layer, (0, 250, 0), scaled_points, 1)

            self.screen.blit(self.vo_layer, (0, 0))  # (0,0) are the top-left coordinates

    def draw_obstacle(self, obstacle):
        # draw obstacle polygon

        scaled_points = []
        for points in obstacle.output_polygon:
            scaled_points.append(self.rescaler(points))

        pygame.draw.polygon(self.screen, obstacle.obstacle_color, scaled_points)

    def object_publisher(self):
        object_database = dict(self.ships_database, **self.obstacle_database)
        return object_database

    def rescaler(self, position_to_scale):
        return [position_to_scale[0] * 40, -position_to_scale[1] * 40 + self.screen_height]

    def rescaler2(self, screen_position):
        return [screen_position[0] / 40, -screen_position[1] / 40 + self.screen_height / 40]


class Simulation2:
    def __init__(self):
        pygame.init()
        # set up the window
        self.screen_width, self.screen_height = 1920, 1080
        flags = pygame.FULLSCREEN
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height),vsync=0)
        pygame.display.set_caption('SHIP SIM V1.0.0')
        self.vo_layer = pygame.Surface((self.screen_width, self.screen_height), pygame.SRCALPHA)
        self.vo_layer.set_alpha(255)  # alpha level

        # set up the clock
        self.clock = pygame.time.Clock()
        self.tick = 60

        # define colors
        self.color_path = (200, 200, 200)
        self.color_sea = (0, 119, 190)
        self.color_ship1 = (211, 211, 211)
        self.color_ship2 = (150, 51, 50)
        self.color_marker = (250, 250, 250)
        self.color_obstacle = (50, 50, 50)

        # initialize ships
        self.ships_database = {
            "main_ship": Ship_class.USV(initial_ship_pos_scaled=np.array([2.0, 2.0]), ship_heading=0,
                                        target_pos_scaled=np.array([22.0, 20.0]), ship_color=self.color_ship1,
                                        guidance_type=2),
            "ship1": Ship_class.USV(initial_ship_pos_scaled=np.array([2.0, 26.0]), ship_heading=0,
                                    target_pos_scaled=np.array([46.0, 2.0]), ship_color=self.color_ship2, guidance_type=2),
            "ship2": Ship_class.USV(initial_ship_pos_scaled=np.array([46.0, 2.0]), ship_heading=3.14,
                                   target_pos_scaled=np.array([2.0, 26.0]),
                                   ship_color=self.color_ship2, guidance_type=2),
            "ship3": Ship_class.USV(initial_ship_pos_scaled=np.array([46.0, 26.0]), ship_heading=3.14,
                                    target_pos_scaled=np.array([2.0, 2.0]),
                                    ship_color=self.color_ship2, guidance_type=2)
        }

        # initialize obstacles
        self.obstacle_database = {
            #"obstacle_1": Obstacle_class.wall(initial_pos_scaled=np.array([12.0, 6.0]), heading=3.14 / 2,
             #                                 obstacle_color=self.color_obstacle),
            #"obstacle_2": Obstacle_class.wall(initial_pos_scaled=np.array([20.0, 20.0]), heading=3.14 / 2,
             #                                 obstacle_color=self.color_obstacle),
            #"obstacle_3": Obstacle_class.wall(initial_pos_scaled=np.array([28.0, 6.0]), heading=3.14 / 2,
             #                                 obstacle_color=self.color_obstacle),
            #"obstacle_4": Obstacle_class.wall(initial_pos_scaled=np.array([36.0, 20.0]), heading=3.14 / 2,
             #                                 obstacle_color=self.color_obstacle),
            #"obstacle_5": Obstacle_class.wall(initial_pos_scaled=np.array([20.0, 0.7]), heading=0,
             #                                 obstacle_color=self.color_obstacle),
            #"obstacle_6": Obstacle_class.wall(initial_pos_scaled=np.array([28.0, 26.5]), heading=0,
             #                                 obstacle_color=self.color_obstacle),
        }
        self.font = pygame.font.SysFont('Arial', 24)

    def loop(self):
        # set up the game loop
        target_pos = [1800, 100]

        running = True
        while running:

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONUP:
                    target_pos = pygame.mouse.get_pos()

            self.screen.fill(self.color_sea)  # fills the background
            self.vo_layer.fill((255,255,255,0))  # fills the background

            object_database = self.object_publisher()  # returns info of the objects(ships and obstacles) in the world.

            for key, obstacle in list(self.obstacle_database.items()):
                obstacle.polygon()
                self.draw_obstacle(obstacle)


            # Draw ships
            for key, ship_obj in list(self.ships_database.items()):
                original_object_database = object_database
                # print("object_database",object_database)
                #del object_database[key]
                sensor_data = object_database
                # print("sensor_data",sensor_data)
                object_database = original_object_database

                if key == "main_ship":
                    ship_obj.move_ship(target_pos_scaled=self.rescaler2(target_pos), sensor_data=sensor_data, log=True)
                else:
                    ship_obj.move_ship(sensor_data=sensor_data)

                if key == "main_ship":
                    self.visualize_vo(ship_obj)

                self.draw_ship(ship_obj)


            # Draw obstacles
            self.draw_target(target_pos)

            fps = self.clock.get_fps()
            fps_text = self.font.render(f"FPS: {fps:.2f}", True, (255, 255, 255))

            # Draw the FPS counter to the screen
            self.screen.blit(fps_text, (10, 10))


            # update the screen
            pygame.display.flip()
            # wait for the next frame
            self.clock.tick(self.tick)

        pygame.quit()

    def draw_target(self, target_pos):
        pygame.draw.line(self.screen, self.color_marker, (target_pos[0] - 10, target_pos[1]),
                         (target_pos[0] + 10, target_pos[1]), 2)
        pygame.draw.line(self.screen, self.color_marker, (target_pos[0], target_pos[1] - 10),
                         (target_pos[0], target_pos[1] + 10), 2)

    def draw_ship(self, ship_obj):
        # draw ship path
        ship_pos_list = ship_obj.ship_pos_list

        scaled_points2 = []

        for points in ship_pos_list:
            scaled_points2.append(self.rescaler(points))

        pygame.draw.aalines(self.screen, self.color_path, False, scaled_points2, 1)

        scaled_points = []
        for points in ship_obj.output_polygon:
            scaled_points.append(self.rescaler(points))

        # draw ship polygon
        pygame.draw.polygon(self.screen, ship_obj.ship_color, scaled_points)

    def visualize_vo(self, ship_obj):


        # draw VO circle
        # print("ship_obj",ship_obj.vo_circle)
        if ship_obj.vo_circles != None:

            for circle in ship_obj.vo_circles:
                pygame.draw.circle(self.vo_layer, circle[2], self.rescaler(circle[0]), circle[1]*40, circle[3])
            # draw VO cone

            # pygame.draw.polygon(self.screen, (100, 0, 0), ship_obj.vo_cone)
            for cone in ship_obj.vo_cones:
                # pygame.draw.line(self.screen, (255, 0, 0), cone[0] * 40, (cone[0] + np.array([math.cos(cone[1][0])*cone[2], math.cos(cone[1][0])*cone[2]]))*40, 1)
                #VO cone
                polygon_points = [cone[0]]
                arc_angle = np.linspace(start=cone[1][0], stop=cone[1][1], num=60, endpoint=False)

                for angle in arc_angle:
                    polygon_points.append(np.array([cone[0][0] + math.cos(angle) * cone[2], cone[0][1] + math.sin(angle) * cone[2]]))
                scaled_points = []
                for points in polygon_points:
                    scaled_points.append(self.rescaler(points))

                pygame.draw.polygon(self.vo_layer, (255, 153, 0, 80), scaled_points)
                pygame.draw.polygon(self.vo_layer, (255, 153, 0, 200), scaled_points,2)

            for lines in ship_obj.vo_lines:
                pygame.draw.line(self.vo_layer, lines[2], self.rescaler(lines[0]), self.rescaler(lines[1]), 2)

            for polygon in ship_obj.vo_polygons:
                scaled_points = []
                for points in polygon:
                    scaled_points.append(self.rescaler(points))
                pygame.draw.polygon(self.vo_layer, (0,250,0), scaled_points,1)


            self.screen.blit(self.vo_layer, (0, 0))  # (0,0) are the top-left coordinates

    def draw_obstacle(self, obstacle):
        # draw obstacle polygon

        scaled_points = []
        for points in obstacle.output_polygon:
            scaled_points.append(self.rescaler(points))

        pygame.draw.polygon(self.screen, obstacle.obstacle_color, scaled_points)

    def object_publisher(self):
        object_database = dict(self.ships_database, **self.obstacle_database)
        return object_database

    def rescaler(self, position_to_scale):
        return [position_to_scale[0]*40, -position_to_scale[1]*40 + self.screen_height]

    def rescaler2(self, screen_position):
        return [screen_position[0]/40, -screen_position[1]/40 + self.screen_height/40]
