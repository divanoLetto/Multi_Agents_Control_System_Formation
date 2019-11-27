import matplotlib.pyplot as plt
import time
import setting

class PositionTable:
    def __init__(self, robotVectors, canvas, ax):
        self.canvas = canvas
        self.ax = ax
        self.table = {}
        self.initPlot(robotVectors)

    def initPlot(self, robotVectors):
        print("init position table plot function")
        self.xdatas = []
        self.ydatas = []
        self.lines = []
        self.colors = ['green', 'red', 'blue', 'purple', 'yellow', 'orange', 'brown', 'purple', 'pink']

        self.xdata_last = []
        self.ydata_last = []
        self.line_last_position = []

        self.structure_lines = []

        for i in range(len(robotVectors)):
            self.xdatas.append([])
            self.ydatas.append([])
            line, = self.ax.plot(self.xdatas[i], self.ydatas[i], marker='o', color=self.colors[i % 9], linestyle='dashed', linewidth=2, markersize=4)
            self.lines.append(line)

            self.xdata_last.append([])
            self.ydata_last.append([])
            last_line, = self.ax.plot(self.xdata_last[i], self.ydata_last[i],  marker='o', markerfacecolor='white', markersize=7, zorder=5)
            self.line_last_position.append(last_line)

    def plotRobotsLive(self, robotsVector, show_formation_lines):

        hash_position = {}
        i = 0
        for robot in robotsVector:
            robot_position = robot.getAbsolutePos()
            hash_position[robot] = robot_position
            self.xdatas[i].append(robot_position[0])
            self.ydatas[i].append(robot_position[1])
            self.lines[i].set_xdata(self.xdatas[i])
            self.lines[i].set_ydata(self.ydatas[i])

            self.xdata_last[i].clear()
            self.ydata_last[i].clear()
            self.xdata_last[i].append(robot_position[0])
            self.ydata_last[i].append(robot_position[1])
            self.line_last_position[i].set_xdata(self.xdata_last[i])
            self.line_last_position[i].set_ydata(self.ydata_last[i])
            i += 1

        while self.structure_lines:
            l = self.structure_lines.pop()
            l.remove()
            del l
        if show_formation_lines:
            lines_formation_structure = []
            for robot1 in robotsVector:
                for robot2 in robotsVector:
                    if robot2 in robot1.neighbour and [robot2, robot1] not in lines_formation_structure:
                        lines_formation_structure.append([robot1, robot2])
            for struct in lines_formation_structure:
                form_structure_data_x = [ hash_position[struct[0]][0], hash_position[struct[1]][0] ]
                form_structure_data_y = [ hash_position[struct[0]][1], hash_position[struct[1]][1] ]
                form_structure_line, = self.ax.plot(form_structure_data_x, form_structure_data_y, linewidth=1, color="black", zorder=2)
                form_structure_line.set_xdata(form_structure_data_x)
                form_structure_line.set_ydata(form_structure_data_y)
                self.structure_lines.append(form_structure_line)

        self.canvas.draw()
        self.canvas.flush_events()
        #time.sleep(setting.delay_time)

    def delete_lines(self):
        while self.lines:
            l = self.lines.pop(0)
            l.remove()
            del l
        while self.line_last_position:
            l = self.line_last_position.pop(0)
            l.remove()
            del l
        while self.structure_lines:
            l = self.structure_lines.pop(0)
            l.remove()
            del l


