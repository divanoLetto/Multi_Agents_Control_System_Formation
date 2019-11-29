from FormationNote import *
from Robot import *

class Simulation:
    def __init__(self):
        self.num_robots = 4
        self.formation_note = SquareNote
        self.robot_type = RobotDisplacementSingleIntegrator
        self.side = 10

        self.current_time = 0
        self.max_time = 15
        self.step_time = 0.01

        self.stop_execution = False
        self.formation_changed = False

        # Una struct che conserva punti e linee dell'ultima free formation
        self.free_formation_struct_to_submit = None  # deve essere fatto il submit per essere confermata e salvata
        self.free_formation_saves = {}  # dizionario che mantiene traccia di tutte le free form salvate

        # parametri per il settaggio della velocità desiderata nel double integrator displacement
        self.desidered_vel_x = 0
        self.desidered_vel_y = 0
        self.desidered_vel_z = 0
        self.desidered_velocity_changed = False

        self.show_formation_lines = True
        self.show_trajectory_lines = True
        self.show_poins_line = True

        self.three_d_plot = False

        self.initial_condition = 0  # 0=random point, 1=choosen point, 2=selected point from plot

    def get_num_robots(self):
        return self.num_robots
    def set_num_robots(self, num_robots):
        self.num_robots = num_robots

    def get_formation_note(self):
        return self.formation_note
    def set_formation_note(self, formation_note):
        self.formation_note = formation_note

    def get_robot_type(self):
        return self.robot_type
    def set_robot_type(self, robot_type):
        self.robot_type = robot_type

    def get_side(self):
        return self.side
    def set_side(self, side):
        self.side = side

    def get_time(self):
        return self.current_time
    def get_max_time(self):
        return self.max_time
    def set_max_time(self, max_t):
        self.max_time = max_t
    def get_step_time(self):
        return self.step_time
    def set_step_time(self, step_t):
        self.step_time = step_t

    def get_des_vel_x(self):
        return self.desidered_vel_x
    def set_des_vel_x(self, vel_x):
        self.desidered_vel_x = vel_x
    def get_des_vel_y(self):
        return self.desidered_vel_y
    def set_des_vel_y(self, vel_y):
        self.desidered_vel_y = vel_y
    def get_des_vel_z(self):
        return self.desidered_vel_z
    def set_des_vel_z(self, vel_z):
        self.desidered_vel_z = vel_z

    def update_time(self):
        x = self.current_time + self.step_time
        self.current_time = float("{0:.2f}".format(x))

    def reset_times(self):
        self.current_time = 0

    def set_initial_condition(self, num):
        self.initial_condition = num
    def get_initial_condition(self):
        return self.initial_condition


