



class OA():

    def __init__(self):
        self.obstace_range = 400
        self.ship_max_speed = 1.0
        self.ship_rot_spd = 0.0
        self.ship_rot_lim = 0.015
        self.ship_accel_pos_lim = 0.1
        self.ship_accel_neg_lim = 0.1
        self.ship_rot_accel_lim = 0.00015
        self.ship_points = [(20, 0), (10, -10), (-20, -10), (-20, 10), (10, 10)]
        self.ship_color = ship_color
        self.ship_pos_list = [ship_pos.tolist()]


        #parameters

    def behavior_tree (self):
        # First, this receives a database of ships in the world. the database is a dictionary
        # if there are no obstacles in range, it pursues the target using Line-of-sight guidance law
        # if there are VO cones in sight, it tries to evade the obstacle by maintaining speed and turning.
        #   it prefers angles closer to the target.
        #   if no trajectory is possible, it tries to decelerate
        #   it tries to rotate when it can not decelerate enough
        # it finally outputs CMD_vel, to be subscribed by



    def sensing(self):
        if

