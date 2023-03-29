import pygame
import math
import numpy as np
import Ship_class
import Sim_loop
import cProfile


sim_loop_1 = Sim_loop.Simulation()



#cProfile.run('sim_loop_1.loop()', 'profile_results1.out')
sim_loop_1.loop()



