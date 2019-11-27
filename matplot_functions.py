from LineBuilder import *
import tkinter as tk


def tellme(s):
    print(s)
    plt.title(s, fontsize=16)
    plt.draw()


def mat_make_free_formation(num_robots, button, entry):
    struct_points_lines = line_builder_formation(num_robots)
    button["state"] = tk.NORMAL
    entry["state"] = tk.NORMAL
    return struct_points_lines
