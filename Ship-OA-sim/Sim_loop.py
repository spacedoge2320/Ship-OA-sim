import pygame
import math
import numpy as np
import Ship_class
import Obstacle_class


class Simulation:
    def __init__(self):
        pygame.init()
        # set up the window
        screen_width, screen_height = 1000, 1000
        self.screen = pygame.display.set_mode((screen_width, screen_height))
        pygame.display.set_caption('SHIP SIM V1.0.0')

        # set up the clock
        self.clock = pygame.time.Clock()
        self.tick = 144

        # define colors
        self.color_path = (200, 200, 200)
        self.color_sea = (0, 119, 190)
        self.color_ship1 = (211, 211, 211)
        self.color_ship2 = (204, 51, 0)
        self.color_marker = (250, 250, 250)
        self.color_obstacle = (50, 50, 50)

        # initialize ships
        self.ships_database = {
            "main_ship": Ship_class.USV(initial_ship_pos_scaled=np.array([900.0, 900.0]), ship_heading=3.14,
                                        target_pos_scaled=np.array([100.0, 100.0]), ship_color=self.color_ship1,guidance_type = 2),
            "ship1": Ship_class.USV(initial_ship_pos_scaled=np.array([800.0, 800.0]), ship_heading=0.0,
                                    target_pos_scaled=np.array([900.0, 900.0]), ship_color=self.color_ship2)
        }

        # initialize obstacles
        self.obstacle_database = {
            "obstacle_1": Obstacle_class.square_obstacle(initial_pos_scaled=np.array([100.0, 900.0]), heading=3.14,
                                                obstacle_color=self.color_obstacle)
            #"2": Obstacle_class.square_obstacle(initial_pos_scaled=np.array([900.0, 100.0]), heading=0.0,
                                              #  obstacle_color=self.color_obstacle)
        }
        self.font = pygame.font.SysFont('Arial', 24)

    def loop(self):
        # set up the game loop
        target_pos = [500, 500]

        running = True
        while running:



            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONUP:
                    target_pos = pygame.mouse.get_pos()

            self.screen.fill(self.color_sea)  # fills the background

            object_database = self.object_publisher()  # returns info of the objects(ships and obstacles) in the world.

            # Draw ships
            for key, ship_obj in list(self.ships_database.items()):
                original_object_database = object_database
                #print("object_database",object_database)
                del object_database[key]
                sensor_data = object_database
                #print("sensor_data",sensor_data)
                object_database = original_object_database

                if key == "main_ship":
                    ship_obj.move_ship(target_pos_scaled=target_pos, sensor_data=sensor_data, log=True)
                else:
                    ship_obj.move_ship(target_pos_scaled=[100, 100])
                self.draw_ship(ship_obj)

                if key == "main_ship":
                    self.visualize_vo(ship_obj)

            # Draw obstacles
            for key, obstacle in list(self.obstacle_database.items()):
                obstacle.polygon()
                self.draw_obstacle(obstacle)

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
        pygame.draw.aalines(self.screen, self.color_path, False, ship_pos_list, 1)

        # draw ship polygon
        pygame.draw.polygon(self.screen, ship_obj.ship_color, ship_obj.output_polygon)

    def visualize_vo(self, ship_obj):
        vo_layer = pygame.Surface((1000, 1000), pygame.SRCALPHA)
        vo_layer.set_alpha(255)  # alpha level

        #draw VO circle
        #print("ship_obj",ship_obj.vo_circle)
        if ship_obj.vo_circles != []:

            for circle in ship_obj.vo_circles:
                pygame.draw.circle(vo_layer, (200, 0, 0, 255), circle[0]*40, circle[1]*40, 2)
            # draw VO cone
            #pygame.draw.polygon(self.screen, (100, 0, 0), ship_obj.vo_cone)
            for cone in ship_obj.vo_cones:
                #pygame.draw.line(self.screen, (255, 0, 0), cone[0] * 40, (cone[0] + np.array([math.cos(cone[1][0])*cone[2], math.cos(cone[1][0])*cone[2]]))*40, 1)

                pygame.draw.line(vo_layer, (255, 153, 51), cone[0] * 40, np.array([cone[0][0] + math.cos(cone[1][0])*cone[2],cone[0][1] + math.sin(cone[1][0])*cone[2]])*40, 2)
                pygame.draw.line(vo_layer, (255, 153, 51), cone[0] * 40, np.array([cone[0][0] + math.cos(cone[1][1])*cone[2],cone[0][1] + math.sin(cone[1][1])*cone[2]]) * 40, 2)

            for lines in ship_obj.vo_lines:
                pygame.draw.line(vo_layer, (51, 51, 255), lines[0] * 40, lines[1] * 40, 2)

        self.screen.blit(vo_layer, (0, 0))  # (0,0) are the top-left coordinates





    def draw_obstacle(self, obstacle):
        # draw obstacle polygon
        pygame.draw.polygon(self.screen, obstacle.obstacle_color, obstacle.output_polygon)

    def object_publisher(self):
        object_database = dict(self.ships_database, **self.obstacle_database)
        return object_database

