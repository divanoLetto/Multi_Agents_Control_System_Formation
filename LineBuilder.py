from matplotlib import pyplot as plt
from matplotlib.widgets import TextBox
import time
import numpy as np
from Formation import *
from Formation import FreeFormationDisplacementSingleIntegrator
from FormationNote import *
from utils import *


class LineBuilderPoint:
    def __init__(self, line, num_robots):
        self.line = line
        self.xs = list(line.get_xdata())
        self.ys = list(line.get_ydata())
        self.cid = line.figure.canvas.mpl_connect('button_press_event', self)
        self.count = 0
        self.num_robots = num_robots

    def __call__(self, event):
        if self.count < self.num_robots:
            print('click', event)
            if event.inaxes != self.line.axes: return
            self.xs.append(event.xdata)
            self.ys.append(event.ydata)
            self.line.set_data(self.xs, self.ys)
            self.line.figure.canvas.draw()
            self.count += 1

    def deleteLine(self):
        self.line.remove()


class LineBuilderLine:
    def __init__(self, line, ptsFormation):
        self.line = line
        self.xs = list(line.get_xdata())
        self.ys = list(line.get_ydata())
        self.cid = line.figure.canvas.mpl_connect('button_press_event', self)
        self.boleano = True
        self.ptsFormation = ptsFormation
        self.connectLineasFormations = []
        self.lineRemember = []
        self.count = 0
        self.currentPoint = None

    def get_line_remember(self):
        return self.lineRemember

    def __call__(self, event):
        if event.button != 3 and self.boleano:
            # print('click', event)
            if event.inaxes != self.line.axes: return
            maybePoint = is_close_to_a_point(self.ptsFormation, [event.xdata, event.ydata])
            if maybePoint != [] and are_different_point(maybePoint, self.currentPoint):
                if self.count == 0:
                    self.currentPoint = maybePoint
                else:
                    self.lineRemember.append([self.currentPoint[0], maybePoint[0]])
                    self.currentPoint = maybePoint
                self.xs.append(event.xdata)
                self.ys.append(event.ydata)
                print("itsapoint!")
                self.line.set_data(self.xs, self.ys)
                self.line.figure.canvas.draw()
                self.count += 1
        else:
            self.boleano = False

    def deleteLine(self):
        self.line.remove()


def line_builder_formation(num_robots):
    print("line builder formation function")
    axes = plt.gca()
    line_builder_distance_list = []#vettore dlle linee usato per cancellarle alla fine
    axes.set_title("You will set the formation for " + str(num_robots) + " robots, click to set line segment")
    plt.gcf().canvas.draw()
    line, = axes.plot([], [],  marker='o', linestyle='None', linewidth=2, markersize=5)  # empty line
    linebuilder = LineBuilderPoint(line, num_robots)
    ptsFormation = np.asarray(plt.ginput(num_robots, timeout=-1))

    line_builder_distance_list.append(linebuilder)  # todo useless?
    list = []
    for i in range(len(ptsFormation)):
        elem = [i, ptsFormation[i]]
        list.append(elem)
    ptsFormation = list
    boleano = False
    axes.set_title("You will set the costraints between points")
    line_remember_total = []

    while boleano is False:
        axes.set_title("Click left to make costraint, click right to stop")
        plt.gcf().canvas.draw()
        line, = axes.plot([], [], linestyle='solid', linewidth=2)  # empty line
        linebuilder = LineBuilderLine(line, ptsFormation)
        justForFun = np.asarray(plt.ginput(200, timeout=-1, mouse_stop=3, mouse_pop=2))
        axes.set_title("Click left to add more costraint, press the keybar to stop")
        plt.gcf().canvas.draw()
        line_remember_total.extend(linebuilder.get_line_remember())
        line_builder_distance_list.append(linebuilder)
        boleano = plt.waitforbuttonpress()

    axes.set_title("Submit the formation")
    plt.gcf().canvas.draw()
    print(ptsFormation)
    print(line_remember_total)

    struct ={}
    struct["points"] = ptsFormation
    struct["lines"] = line_remember_total
    return struct
