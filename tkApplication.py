import matplotlib
from matplotlib.backends._backend_tk import NavigationToolbar2Tk
matplotlib.use('TkAgg')
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import tkinter as tk  # python 2.7
from tkinter import messagebox
import sys
import random
from Simulation import *
from matplot_functions import *
from Factory import *
from PositionTable import *

from mpl_toolkits.mplot3d import Axes3D
from Formation3D import SquareFormationDisplacementSingleIntegrator3D
from Robot3D import RobotDisplacementSingleIntegrator3D
from myGlobalEnviroment3D import myGlobalEnviroment3D
from PositionTable3D import PositionTable3D


class Application(tk.Frame):
    def __init__(self, master=None):
        tk.Frame.__init__(self, master)
        self.simulation = Simulation()
        self.root = master
        self.createWidgets()
        self.makeFullScreen()

    def createWidgets(self):
        self.set_2D_plot()

        info_frame = tk.Frame(self.master, bg='white', highlightbackground="black", highlightthickness=1, padx=4, pady=4)
        row1 = tk.Frame(info_frame)
        row2 = tk.Frame(info_frame)
        row3 = tk.Frame(info_frame)
        row4 = tk.Frame(info_frame)
        row5 = tk.Frame(info_frame)
        row6 = tk.Frame(info_frame)
        label_1 = tk.Label(row1, width=15, bg='white', text="Formation : ", anchor='w').pack(side=tk.LEFT)
        label_2 = tk.Label(row2, width=15, height=4, bg='white', text="Control law : ", anchor='w').pack(side=tk.LEFT)
        label_3 = tk.Label(row3, width=15, bg='white', text="Robots' number : ", anchor='w').pack(side=tk.LEFT)
        label_4 = tk.Label(row4, width=15, bg='white', text="Time : ", anchor='w').pack(side=tk.LEFT)
        label_5 = tk.Label(row5, width=15, bg='white', text="Step time : ", anchor='w').pack(side=tk.LEFT)
        label_6 = tk.Label(row6, width=15, bg='white', text="Max time : ", anchor='w').pack(side=tk.LEFT)
        self.var_formation = tk.StringVar()
        self.var_control_law = tk.StringVar()
        self.var_robot_number = tk.StringVar()
        self.var_time = tk.StringVar()
        self.var_step_time = tk.StringVar()
        self.var_max_time = tk.StringVar()
        label_1_1 = tk.Label(row1, width=23, textvariable=self.var_formation).pack(side=tk.RIGHT, padx=3)
        label_2_1 = tk.Message(row2, width=180, anchor=tk.E, justify=tk.LEFT, textvariable=self.var_control_law).pack(side=tk.LEFT, padx=3)
        label_3_1 = tk.Label(row3, width=23, textvariable=self.var_robot_number).pack(side=tk.RIGHT, padx=3)
        label_4_1 = tk.Label(row4, width=23, textvariable=self.var_time).pack(side=tk.RIGHT, padx=3)
        label_5_1 = tk.Label(row5, width=23, textvariable=self.var_step_time).pack(side=tk.RIGHT, padx=3)
        label_6_1 = tk.Label(row6, width=23, textvariable=self.var_max_time).pack(side=tk.RIGHT, padx=3)

        row1.pack(padx=10, pady=10)
        row2.pack(padx=10, pady=10, fill=tk.BOTH)
        row3.pack(padx=10, pady=10)
        row4.pack(padx=10, pady=10)
        row5.pack(padx=10, pady=10)
        row6.pack(padx=10, pady=10)
        info_frame.grid(row=0, column=0, padx=15)

        menubar = tk.Menu(self.master)
        self.master.config(menu=menubar)

        executionMenu = tk.Menu(menubar)
        menubar.add_cascade(label="Execution", menu=executionMenu)
        executionMenu.add_command(label="Start", command=lambda: self.onStart())
        executionMenu.add_command(label="Stop", command=lambda: self.stop_execution())
        executionMenu.add_command(label="Time setting", command=lambda: self.set_time_setting())
        executionMenu.add_command(label="Exit", command=lambda: self.onExit())

        self.formationMenu = tk.Menu(menubar)
        menubar.add_cascade(label="Formation", menu=self.formationMenu)
        self.subFormationMenu_select = tk.Menu(self.formationMenu)
        self.subFormationMenu_select.add_command(label="Square", command=lambda: self.set_formation_square())
        self.subFormationMenu_select.add_command(label="Linear", command=lambda: self.set_formation_linear())
        self.formationMenu.add_cascade(label='Select formation', menu=self.subFormationMenu_select, underline=0)
        self.formationMenu.add_command(label="Make free formation", command=lambda: self.make_free_formation())

        controlLawMenu = tk.Menu(menubar)
        menubar.add_cascade(label="Control law", menu=controlLawMenu)
        self.subControlLawMenu_select = tk.Menu(controlLawMenu)
        self.subControlLawMenu_select.add_command(label="Displacement Single Integrator ", command=lambda: self.set_robot_type(RobotDisplacementSingleIntegrator))
        self.subControlLawMenu_select.add_command(label="Displacement Double Integrator ", command=lambda: self.set_robot_type(RobotDisplacementDoubleIntegrator))
        self.subControlLawMenu_select.add_command(label="Displacement Unicycle ", command=lambda: self.set_robot_type(RobotDisplacementUnicycle))
        self.subControlLawMenu_select.add_command(label="Distance Single Integrator ", command=lambda: self.set_robot_type(RobotDistanceSingleIntegrator))
        self.subControlLawMenu_select.add_command(label="Distance Double Integrator ", command=lambda: self.set_robot_type(RobotDistanceDoubleIntegrator))
        self.subControlLawMenu_select.add_command(label="Distance Unicycle ", command=lambda: self.set_robot_type(RobotDistanceUnicycle))
        controlLawMenu.add_cascade(label='Control law ', menu=self.subControlLawMenu_select, underline=0)

        robotsMenu = tk.Menu(menubar)
        menubar.add_cascade(label="Robots", menu=robotsMenu)
        robotsMenu.add_command(label="Number robots", command=lambda: self.set_num_robots())
        robotsMenu.add_command(label="Desidered velocity", command=lambda: self.set_desidered_velocity())
        self.subInitialConditionMenu_select = tk.Menu(controlLawMenu)
        self.subInitialConditionMenu_select.add_command(label="Random points", command=lambda: self.set_init_cond_rand_points())
        self.subInitialConditionMenu_select.add_command(label="Select points on plot", command=lambda: self.set_init_cond_input())
        robotsMenu.add_cascade(label='Initial condition ', menu=self.subInitialConditionMenu_select)

        plotMenu = tk.Menu(menubar)
        menubar.add_cascade(label="Plot", menu=plotMenu)
        plotMenu.add_command(label="Show/Hide edges", command=lambda: self.show_hide_formation())
        plotMenu.add_command(label="Show/Hide trajectory", command=lambda: self.show_hide_trajectory())
        plotMenu.add_command(label="Show/Hide robots", command=lambda: self.show_hide_points())
        plotMenu.add_separator()
        plotMenu.add_command(label="2D plot", command=lambda: self.set_2D_plot())
        plotMenu.add_command(label="3D plot", command=lambda: self.set_3D_plot())

        self.make_window_num_robots()  # make num robot window
        self.window_num_robots.withdraw()  # hide num robot window

        self.make_window_desidered_velocity()  # make num robot window
        self.window_desidered_velocity.withdraw()  # hide num robot window

        self.make_window_time_setting()
        self.window_time_setting.withdraw()

        self.count_free_formation = 1  # counter per salvare le free formation

        self.info_frame_update()

    def set_2D_plot(self):
        self.stop_execution()
        if hasattr(self, 'fig'):
            tmp = self.fig
            self.fig = plt.figure(figsize=(7, 7))
            plt.close(tmp)
        else:
            self.fig = plt.figure(figsize=(7, 7))

        ax = self.fig.add_axes([0.1, 0.1, 0.8, 0.8], polar=False)
        ax.set_xlim(xmin=0, xmax=50)
        ax.set_ylim(ymin=0, ymax=50)
        plt.axis('scaled')
        canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        canvas.get_tk_widget().grid(row=0, column=1, padx=4)
        canvas.draw()

        toolbar_frame = tk.Frame(self.master)
        toolbar_frame.grid(row=1, column=1, pady=0)
        self.toolbar = NavigationToolbar2Tk(canvas, toolbar_frame)
        self.fig.canvas.draw()
        self.remove_3D_navbar_command()
        self.simulation.three_d_plot = False

    def set_3D_plot(self):
        self.stop_execution()
        tmp = self.fig
        self.fig = plt.figure(figsize=(7, 7))
        plt.close(tmp)
        canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        canvas.get_tk_widget().grid(row=0, column=1, padx=4)
        canvas.draw()
        ax = self.fig.add_subplot(111, projection='3d')
        ax.set_xlim3d(0, 50)
        ax.set_ylim3d(0, 50)
        ax.set_zlim3d(0, 50)

        toolbar_frame = tk.Frame(self.master)
        toolbar_frame.grid(row=1, column=1, pady=0)
        self.toolbar = NavigationToolbar2Tk(canvas, toolbar_frame)
        self.fig.canvas.draw()
        self.add_3D_navbar_command()
        self.simulation.three_d_plot = True

    def add_3D_navbar_command(self):
        self.set_robot_type(RobotDisplacementSingleIntegrator)
        if self.simulation.three_d_plot == False:  # se non era già vero
            self.subFormationMenu_select.add_command(label="Cube", command=lambda: self.set_formation_cube())
            if hasattr(self, 'formationMenu'):
                self.formationMenu.delete("Make free formation")
            if hasattr(self, 'subControlLawMenu_select'):
                self.subControlLawMenu_select.delete("Displacement Unicycle ")
                self.subControlLawMenu_select.delete("Distance Unicycle ")
            if hasattr(self, 'subInitialConditionMenu_select'):
                self.subInitialConditionMenu_select.delete("Select points on plot")
            self.simulation.set_initial_condition(0)

    def remove_3D_navbar_command(self):
        if hasattr(self, 'var_formation'):
            self.set_formation_square()
        if self.simulation.three_d_plot == True:
            if hasattr(self, 'subFormationMenu_select'):
                a = self.subFormationMenu_select.index("Cube")
                self.subFormationMenu_select.delete("Cube")
            if hasattr(self, 'formationMenu'):
                self.formationMenu.add_command(label="Make free formation", command=lambda: self.make_free_formation())
            if hasattr(self, 'subControlLawMenu_select'):
                self.subControlLawMenu_select.delete("Distance Single Integrator ")
                self.subControlLawMenu_select.delete("Distance Double Integrator ")

                self.subControlLawMenu_select.add_command(label="Displacement Unicycle ", command=lambda: self.set_robot_type(RobotDisplacementUnicycle))
                self.subControlLawMenu_select.add_command(label="Distance Single Integrator ", command=lambda: self.set_robot_type(RobotDistanceSingleIntegrator))
                self.subControlLawMenu_select.add_command(label="Distance Double Integrator ", command=lambda: self.set_robot_type(RobotDistanceDoubleIntegrator))
                self.subControlLawMenu_select.add_command(label="Distance Unicycle ", command=lambda: self.set_robot_type(RobotDistanceUnicycle))
            if hasattr(self, 'subInitialConditionMenu_select'):
                self.subInitialConditionMenu_select.add_command(label="Select points on plot",
                                                                command=lambda: self.set_init_cond_input())

    def makeFullScreen(self):
        root = self.master
        root.attributes('-fullscreen', True)
        root.bind('<Escape>', lambda e: root.destroy())

    def info_frame_update(self):
        formation_note = str(self.simulation.get_formation_note().getName())
        robot_type = str(self.simulation.get_robot_type().getName())
        num_robots = str(self.simulation.get_num_robots())
        current_time = str(self.simulation.get_time())
        step_time = str(self.simulation.get_step_time())
        max_time = str(self.simulation.get_max_time())
        # lato = str(self.simulatiotion.get_side())

        self.var_formation.set(formation_note)
        self.var_control_law.set(robot_type)
        self.var_robot_number.set(num_robots)
        self.var_time.set(current_time)
        self.var_step_time.set(step_time)
        self.var_max_time.set(max_time)

    def make_window_num_robots(self):
        print("make window num robots funtion")
        self.window_num_robots = tk.Toplevel(self.master)
        self.window_num_robots.title("Number robots")
        row = tk.Frame(self.window_num_robots)
        label_num_rob = tk.Label(row, width=22, text="Robots' number : ", anchor='w')
        self.entry_num_robot = tk.Entry(row)
        num_robots = self.simulation.get_num_robots()
        self.entry_num_robot.insert(0, str(num_robots))
        row.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)
        label_num_rob.pack(side=tk.LEFT)
        self.entry_num_robot.pack(side=tk.RIGHT, expand=tk.YES, fill=tk.X)
        b1 = tk.Button(self.window_num_robots, text='Submit ', command=lambda: self.submit_num_robots())
        b1.pack(side=tk.LEFT, padx=5, pady=5)
        self.window_num_robots.protocol('WM_DELETE_WINDOW', self.close_num_robots_window)  # per nascondere e non cancellare

    def make_window_desidered_velocity(self):
        print("make window desidered velocity funtion")
        self.window_desidered_velocity = tk.Toplevel(self.master)
        self.window_desidered_velocity.title("Double integrator desidered velocity")
        row = tk.Frame(self.window_desidered_velocity)
        label_des_vel_x = tk.Label(row, width=22, text="Vel x : ", anchor='w')
        self.entry_des_vel_x = tk.Entry(row)
        des_vel_x = self.simulation.get_des_vel_x()
        self.entry_des_vel_x.insert(0, str(des_vel_x))
        row.pack(side=tk.TOP, fill=tk.X, padx=8, pady=5)
        label_des_vel_x.pack(side=tk.LEFT)
        self.entry_des_vel_x.pack(side=tk.RIGHT, expand=tk.YES, fill=tk.X)

        row2 = tk.Frame(self.window_desidered_velocity)
        label_des_vel_y = tk.Label(row2, width=22, text="Vel y : ", anchor='w')
        self.entry_des_vel_y = tk.Entry(row2)
        des_vel_y = self.simulation.get_des_vel_y()
        self.entry_des_vel_y.insert(0, str(des_vel_y))
        row2.pack(side=tk.TOP, fill=tk.X, padx=8, pady=5)
        label_des_vel_y.pack(side=tk.LEFT)
        self.entry_des_vel_y.pack(side=tk.RIGHT, expand=tk.YES, fill=tk.X)

        b1 = tk.Button(self.window_desidered_velocity, text='Submit ', command=lambda: self.submit_desidered_velocities())
        b1.pack(side=tk.LEFT, padx=5, pady=5)
        self.window_desidered_velocity.protocol('WM_DELETE_WINDOW', self.close_desidered_velocity_window)  # per nascondere e non cancellare

    def make_window_make_free_formation(self):
        print("make window free formation maker")
        self.window_make_free_formation = tk.Toplevel(self.master)
        self.window_make_free_formation.title("Free formation")

        self.fig_free = plt.figure(figsize=(6, 6))
        ax = self.fig_free.add_axes([0.2, 0.2, 0.6, 0.6])
        ax.set_xlim(xmin=0, xmax=50)
        ax.set_ylim(ymin=0, ymax=50)
        plt.axis('scaled')
        canvas = FigureCanvasTkAgg(self.fig_free, master=self.window_make_free_formation)
        canvas.get_tk_widget().pack(fill=tk.BOTH)

        row = tk.Frame(self.window_make_free_formation)
        label = tk.Label(row, width=22, text="Name : ", anchor='w')
        self.entry_name_free_formation = tk.Entry(row)
        label.pack(side=tk.LEFT)
        self.entry_name_free_formation.pack(side=tk.LEFT)
        row.pack(side=tk.LEFT, padx=10, pady=10)
        self.button_free_formation_submit = tk.Button(self.window_make_free_formation, text='Submit ', command=lambda: self.submit_make_free_formation())
        self.button_free_formation_submit.pack(side=tk.RIGHT, padx=10, pady=10)
        self.button_free_formation_submit["state"] = tk.DISABLED
        self.entry_name_free_formation["state"] = tk.DISABLED
        self.window_make_free_formation.protocol('WM_DELETE_WINDOW', self.close_free_formation_window)   # per nascondere e non cancellare
        canvas.draw()

    def make_window_time_setting(self):
        print("make window num robots funtion")
        self.window_time_setting = tk.Toplevel(self.master)
        self.window_time_setting.title("Time setting")
        row = tk.Frame(self.window_time_setting)
        label_step_time = tk.Label(row, width=22, text="Step time : ", anchor='w')
        self.entry_step_time = tk.Entry(row)
        step_t = self.simulation.get_step_time()
        self.entry_step_time.insert(0, str(step_t))
        row.pack(side=tk.TOP, fill=tk.X, padx=8, pady=5)
        label_step_time.pack(side=tk.LEFT)
        self.entry_step_time.pack(side=tk.RIGHT, expand=tk.YES, fill=tk.X)

        row2 = tk.Frame(self.window_time_setting)
        label_max_time = tk.Label(row2, width=22, text="Max time : ", anchor='w')
        self.entry_max_time = tk.Entry(row2)
        max_t = self.simulation.get_max_time()
        self.entry_max_time.insert(0, str(max_t))
        row2.pack(side=tk.TOP, fill=tk.X, padx=8, pady=5)
        label_max_time.pack(side=tk.LEFT)
        self.entry_max_time.pack(side=tk.RIGHT, expand=tk.YES, fill=tk.X)

        b1 = tk.Button(self.window_time_setting, text='Submit ', command=lambda: self.submit_time_setting())
        b1.pack(side=tk.LEFT, padx=5, pady=5)
        self.window_time_setting.protocol('WM_DELETE_WINDOW', self.close_time_setting_window)  # per nascondere e non cancellare

    def close_num_robots_window(self):
        print("window num robots close")
        self.window_num_robots.withdraw()
        num_robots = self.simulation.get_num_robots()
        self.entry_num_robot.delete(0, tk.END)  # delete not chosen value
        self.entry_num_robot.insert(0, str(num_robots))  # reset correct value

    def close_desidered_velocity_window(self):
        self.window_desidered_velocity.withdraw()
        des_vel_x = self.simulation.get_des_vel_x()
        des_vel_y = self.simulation.get_des_vel_y()
        self.entry_des_vel_x.delete(0, tk.END)  # delete not chosen value
        self.entry_des_vel_x.insert(0, str(des_vel_x))  # reset correct value
        self.entry_des_vel_y.delete(0, tk.END)
        self.entry_des_vel_y.insert(0, str(des_vel_y))

    def close_free_formation_window(self):
        self.window_make_free_formation.destroy()
        plt.figure(self.fig.number)  # set the current figure as the main figure

    def close_time_setting_window(self):
        print("close time setting window function")
        self.window_time_setting.withdraw()
        step_t = self.simulation.get_step_time()
        max_t = self.simulation.get_max_time()
        self.entry_step_time.delete(0, tk.END)  # delete not chosen value
        self.entry_step_time.insert(0, str(step_t))  # reset correct value
        self.entry_max_time.delete(0, tk.END)  # delete not chosen value
        self.entry_max_time.insert(0, str(max_t))  # reset correct value

    def onExit(self):
        print("Exit function")
        self.quit()

    def set_formation_linear(self):
        print("set formation note function")
        self.simulation.set_formation_note(LinearNote)
        self.var_formation.set(self.simulation.formation_note.getName())  # update info frame
        self.simulation.formation_changed = True

    def set_formation_square(self):
        print("set formation note function")
        self.simulation.set_formation_note(SquareNote)
        self.var_formation.set(self.simulation.formation_note.getName())  # update info frame
        self.simulation.num_robots = 4  # reset the number of robots according to the formation
        self.var_robot_number.set(self.simulation.num_robots)
        self.simulation.formation_changed = True

    def set_formation_free(self, string_name, num_robots):
        self.simulation.set_formation_note(FreeNote)
        self.simulation.num_robots = num_robots  # reset the number of robots according to the formation
        self.var_robot_number.set(self.simulation.num_robots)
        full_string_name = self.simulation.formation_note.getName() + " - " + str(string_name)
        self.var_formation.set(full_string_name)  # update info frame
        self.simulation.formation_changed = True

    def set_formation_cube(self):
        self.simulation.set_formation_note(CubeNote)
        self.var_formation.set(self.simulation.formation_note.getName())  # update info frame
        self.simulation.num_robots = 8  # reset the number of robots according to the formation
        self.var_robot_number.set(self.simulation.num_robots)
        self.simulation.formation_changed = True

    def make_free_formation(self):

        self.make_window_make_free_formation()  # creo una nuova interfaccia

        print("make free formation function")
        self.window_make_free_formation.deiconify()
        plt.figure(self.fig_free.number)
        num_robots = self.simulation.num_robots
        self.simulation.free_formation_struct_to_submit = mat_make_free_formation(num_robots, self.button_free_formation_submit, self.entry_name_free_formation)
        self.var_formation.set(self.simulation.formation_note.getName())  # update info frame

    def set_robot_type(self, robot_type):
        self.simulation.set_robot_type(robot_type)
        self.var_control_law.set(self.simulation.robot_type.getName())  # update info frame

    def set_num_robots(self):
        self.window_num_robots.deiconify()

    def set_desidered_velocity(self):
        self.window_desidered_velocity.deiconify()

    def set_time_setting(self):
        self.window_time_setting.deiconify()

    def submit_num_robots(self):
        print("Submit num robots function")
        value = int(self.entry_num_robot.get())
        self.simulation.set_num_robots(value)
        self.window_num_robots.withdraw()
        plt.figure(self.fig.number)  # set the current figure as the main figure
        self.var_robot_number.set(value)

    def submit_desidered_velocities(self):
        print("Submit desidered velocities function")
        value_x = float(self.entry_des_vel_x.get())
        value_y = float(self.entry_des_vel_y.get())
        self.simulation.set_des_vel_x(value_x)
        self.simulation.set_des_vel_y(value_y)
        self.window_desidered_velocity.withdraw()
        self.simulation.desidered_velocity_changed = True
        plt.figure(self.fig.number)  # set the current figure as the main figure

    def submit_make_free_formation(self):
        print("Submit make free formation function")
        value = self.entry_name_free_formation.get()
        print("The name of the formation is: " + value)
        if value == "":
            value = "untitled"+str(self.count_free_formation)
            self.count_free_formation += 1

        tmp = self.simulation.free_formation_struct_to_submit  # take the temporary struct of point and lines
        self.simulation.free_formation_saves[value] = tmp  # save it in the dictionary of the simulation
        num_robots = len(tmp["points"])
        self.simulation.free_formation_struct_to_submit = None  # delete the temporary struct

        self.entry_name_free_formation.delete(0, tk.END)  # resetta il nome nella window delle free form
        self.simulation.set_formation_note(FreeNote)
        self.var_formation.set(self.simulation.formation_note.getName() + " - " + str(value))
        self.subFormationMenu_select.add_command(label=value, command=lambda: self.set_formation_free(value, num_robots))
        self.window_make_free_formation.withdraw()

    def submit_time_setting(self):
        print("submit time setting function")
        value_step_t = float(self.entry_step_time.get())
        value_max_t = float(self.entry_max_time.get())
        self.simulation.set_step_time(value_step_t)
        self.simulation.set_max_time(value_max_t)
        self.window_time_setting.withdraw()
        plt.figure(self.fig.number)  # set the current figure as the main figure
        self.var_step_time.set(value_step_t)
        self.var_max_time.set(value_max_t)

    def stop_execution(self):
        self.simulation.stop_execution = True
    def set_init_cond_rand_points(self):
        self.simulation.set_initial_condition(0)
    def set_init_cond_input(self):
        self.simulation.set_initial_condition(1)

    def show_hide_formation(self):
        if self.simulation.show_formation_lines:
            self.simulation.show_formation_lines = False
        else:
            self.simulation.show_formation_lines = True

    def show_hide_trajectory(self):
        if self.simulation.show_trajectory_lines:
            self.simulation.show_trajectory_lines = False
        else:
            self.simulation.show_trajectory_lines = True

    def show_hide_points(self):
        if self.simulation.show_poins_line:
            self.simulation.show_poins_line = False
        else:
            self.simulation.show_poins_line = True

    def initial_condition(self):
        num_robots = self.simulation.num_robots
        pts = []
        if self.simulation.get_initial_condition() == 0:
            pts = []
            if not self.simulation.three_d_plot:
                for i in range(num_robots):
                    randx = random.randint(10, 40)
                    randy = random.randint(10, 40)
                    pts.append([randx, randy])
            else:
                for i in range(num_robots):
                    randx = random.randint(10, 40)
                    randy = random.randint(10, 40)
                    randz = random.randint(10, 40)
                    pts.append([randx, randy, randz])
        elif self.simulation.get_initial_condition() == 1:
            tellme("Select robots start positions with mouse")
            self.fig.canvas.draw()
            pts = np.asarray(plt.ginput(num_robots, timeout=-1, show_clicks=True))
        return pts

    def calcolate_formation(self, bool_three_d):
        num_robots = self.simulation.get_num_robots()
        formation_note = self.simulation.get_formation_note()
        robot_type = self.simulation.get_robot_type()
        lato = self.simulation.get_side()
        free_formation_structs_saves = self.simulation.free_formation_saves
        last_free_formation_name = self.var_formation.get().replace('Free formation - ', '')
        if not bool_three_d:
            formation = make_formation_from_robot_and_note(formation_note=formation_note, robot_type=robot_type,
                                                           lato=lato, num_robots=num_robots,
                                                           free_formation_saves=free_formation_structs_saves,
                                                           last_free_formation_name=last_free_formation_name,
                                                           vel_x=self.simulation.desidered_vel_x,
                                                           vel_y=self.simulation.desidered_vel_y)
        else:
            formation = make_formation_from_robot_and_note_3D(formation_note=formation_note, robot_type=robot_type,
                                                              lato=lato, num_robots=num_robots,
                                                              free_formation_saves=free_formation_structs_saves,
                                                              last_free_formation_name=last_free_formation_name,
                                                              vel_x=self.simulation.desidered_vel_x,
                                                              vel_y=self.simulation.desidered_vel_y,
                                                              vel_z=self.simulation.desidered_vel_z)
        print("Num robots: " + str(num_robots))
        print("Formation note: " + formation_note.getName())
        print("Robot type: " + str(robot_type))
        print("The formation is: " + str(formation))
        return formation

    def make_position_tables(self, robots_vector, bool_three_d):
        if not bool_three_d:
            positions_table = PositionTable(robots_vector, canvas=self.fig.canvas, ax=self.fig.axes[0])
        else:
            positions_table = PositionTable3D(robots_vector, canvas=self.fig.canvas, ax=self.fig.axes[0])
        return positions_table

    def make_global_space(self, robots_vector):
        myGlobalEnviroment(robots_vector)  # crea un istanza di spazio globale
        myGlobalEnviroment3D(robots_vector)  # crea un istanza di spazio globale 3D

    def onStart(self):
        self.stop_execution()
        print("start function")
        bool_three_d = self.simulation.three_d_plot
        num_robots = self.simulation.get_num_robots()
        formation = self.calcolate_formation(bool_three_d)

        #self.simulation.num_robots = 8
        #num_robots = self.simulation.get_num_robots()
        #formation = CubeFormationDistanceDoubleIntegrator3D(self.simulation.side)

        if formation.uncompatibility_number_robots(self.simulation.num_robots):
            messagebox.showwarning("Error", "The formation chosen is incompatible with the selected number of robots")
            return

        plt.figure(self.fig.number)  # set the current figure
        self.fig.canvas.draw()
        pts = self.initial_condition()
        print("robotsVector inizialing")
        robots_vector = make_robots_from_points(pts=pts, num_robots=num_robots, formation=formation, bool_three_d=bool_three_d)

        bonus_points = []  # vettore che tiene traccia di tutti i punti oltre quelli della linea
        self.fig.canvas.draw()

        self.make_global_space(robots_vector)
        commander = findCommander(robots_vector)
        commander.makeSpanningTree()
        commander.makeCostraint()

        positions_table = self.make_position_tables(robots_vector, bool_three_d)
        reset_simulation_setting(self.simulation)  # resetta a False il flag cambia formazione
        robots_vector = sorted(robots_vector, key=lambda item: item.role)

        print("calcolating...")
        while self.simulation.get_time() < self.simulation.get_max_time() and self.simulation.stop_execution is False:
            self.simulation.update_time()
            self.var_time.set(self.simulation.current_time)
            positions_table.plotRobotsLive(robots_vector, self.simulation.show_formation_lines, self.simulation.show_trajectory_lines, self.simulation.show_poins_line)
            updateRobots(robots_vector, self.simulation.get_step_time())
            if self.simulation.formation_changed:
                print("formation changed, calcolating new formation in start function")
                self.simulation.formation_changed = False
                formation = self.calcolate_formation(bool_three_d)
                if formation.uncompatibility_number_robots(num_robots):
                    messagebox.showwarning("Error", "The formation chosen is incompatible with the selected number of robots")
                else:
                    commander.makeSpanningTree()
                    commander.setFormation(formation)
                    commander.makeCostraint()
            if self.simulation.desidered_velocity_changed:
                print("velocity changed detected")
                self.simulation.desidered_velocity_changed = False
                commander.change_velocity_and_propagate(self.simulation.desidered_vel_x, self.simulation.desidered_vel_y)
        self.fig.canvas.draw()
        reset_plot_setting(positions_table, bonus_points)
        reset_simulation_time_setting(self.simulation)


def makeTkApplication():
    root = tk.Tk()
    app = Application(master=root)
    app.mainloop()
