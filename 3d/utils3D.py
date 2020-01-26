import time
import setting
import matplotlib.pyplot as plt
import math


def findCommander(robotVector):
    for robot in robotVector:
        if robot.isCommander():
            return robot
    else:
        return None


def distance_between_point(a, b):
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def find_robot_by_role(roboVector, role):
    for robot in roboVector:
        if robot.role == role:
            return robot
    print("No robot with the desidered role")
    return None


def are_different_point(punto1, punto2):
    if punto2 is None:
        return True
    if punto1[0] == punto2[0]:
        return False
    else:
        return True


def updateRobots(robotVector, stepTime):
    for i in robotVector:
        i.calculateControl(stepTime)
    for i in robotVector:
        i.updatePosition()


def reset_plot_setting(positions_table, bonus_points):
    delete_lines(positions_table, bonus_points)


def delete_lines(positions_table, bonus_points):
    positions_table.delete_lines()
    while bonus_points:
        p = bonus_points.pop(0)
        p[0].remove()
        del p


def are_not_connected(robot_list, robot):
    for elem in robot_list:
        if elem == robot:
            return False
    return True


def rotate_point(rotate_center, point, alpha):
    new_x = ((point[0] - rotate_center[0]) * math.cos(alpha)) - ((point[1] - rotate_center[1]) * math.sin(alpha)) + rotate_center[0]
    new_y = ((point[0] - rotate_center[0]) * math.sin(alpha)) + ((point[1] - rotate_center[1]) * math.cos(alpha)) + rotate_center[1]
    new_z = point[2]
    return [new_x, new_y, new_z]


def rotate_point_planxz(rotate_center, point, alpha):
    new_x = ((point[0] - rotate_center[0]) * math.cos(alpha)) - ((point[2] - rotate_center[2]) * math.sin(alpha)) + rotate_center[0]
    new_y = point[1]
    new_z = ((point[0] - rotate_center[0]) * math.sin(alpha)) + ((point[2] - rotate_center[2]) * math.cos(alpha)) + rotate_center[2]
    return [new_x, new_y, new_z]

def rotate_point_planyz(rotate_center, point, alpha):
    new_x = point[0]
    new_y = ((point[1] - rotate_center[1]) * math.cos(alpha)) - ((point[2] - rotate_center[2]) * math.sin(alpha)) + rotate_center[1]
    new_z = ((point[1] - rotate_center[1]) * math.sin(alpha)) + ((point[2] - rotate_center[2]) * math.cos(alpha)) + rotate_center[2]
    return [new_x, new_y, new_z]

def get_matrices_from_points(robotVector, points, baricenter):
    matrixs = []
    alpha = 0

    #fig = plt.figure()
    #ax = plt.axes(projection='3d')
    #for point in points:
    #    ax.scatter3D(point[0], point[1], point[2], color='gray');

    for i in range(7):
        matrix = []
        for robot in robotVector:
            robot_roles_assigment = []
            for point in points:
                rotate_p = rotate_point(baricenter, point, alpha)
                robot_role = distance_between_point(robot.getAbsolutePos(), rotate_p)
                robot_roles_assigment.append(robot_role)
            matrix.append(robot_roles_assigment)
        matrixs.append(matrix)
        alpha += (math.pi/4)

    #fig.show()
    alpha = 0
    for i in range(7):
        matrix = []
        for robot in robotVector:
            robot_roles_assigment = []
            for point in points:
                rotate_p = rotate_point_planyz(baricenter, point, alpha)
                robot_role = distance_between_point(robot.getAbsolutePos(), rotate_p)
                robot_roles_assigment.append(robot_role)
            matrix.append(robot_roles_assigment)
        matrixs.append(matrix)
        alpha += (math.pi/4)
    #fig.show()
    alpha = 0
    for i in range(7):
        matrix = []
        for robot in robotVector:
            robot_roles_assigment = []
            for point in points:
                rotate_p = rotate_point_planxz(baricenter, point, alpha)
                robot_role = distance_between_point(robot.getAbsolutePos(), rotate_p)
                robot_roles_assigment.append(robot_role)
            matrix.append(robot_roles_assigment)
        matrixs.append(matrix)
        alpha += (math.pi/4)
    #fig.show()
    return matrixs
