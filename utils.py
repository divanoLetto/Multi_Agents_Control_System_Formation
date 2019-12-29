import setting
import math
import random

def findCommander(robotVector):
    for robot in robotVector:
        if robot.isCommander():
            return robot
    else:
        return None


def distance_between_point(a, b):
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)


def is_close_to_a_point(pntFormation, pnt):
    min_distance = math.inf
    min_point = None
    bool_found = False
    for punto in pntFormation:
        if distance_between_point(punto[1], pnt) < setting.min_drawable_point_tollerance:
            if distance_between_point(punto[1], pnt) < min_distance:
                min_distance = distance_between_point(punto[1], pnt)
                min_point = punto
                bool_found = True
    if bool_found:
        return min_point
    else:
        return []


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


def reset_simulation_time_setting(simulation):
    simulation.reset_times()
    simulation.stop_execution = False


def reset_simulation_setting(simulation):
    simulation.formation_changed = False
    simulation.stop_execution = False
    simulation.desidered_velocity_changed = False


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
    return [new_x, new_y]


def get_matrices_from_points(robotVector, points, baricenter):
        matrixs = []
        alpha = 0
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

        return matrixs

def make_robots_from_points(pts, num_robots, formation, bool_three_d):
    robots_vector = []
    i = 0
    rand = random.randint(0, num_robots - 1)  # scegli il commander
    if not bool_three_d:
        for pnt in pts:
            if i == rand:
                robot = formation.specifyFormationRobot(pnt[0], pnt[1])
                robot.makeCommander(formation)
            else:
                robot = formation.specifyFormationRobot(pnt[0], pnt[1])
            robots_vector.append(robot)
            i += 1
    else:
        for pnt in pts:
            if i == rand:
                robot = formation.specifyFormationRobot(pnt[0], pnt[1], pnt[2])
                robot.makeCommander(formation)
            else:
                robot = formation.specifyFormationRobot(pnt[0], pnt[1], pnt[2])
            robots_vector.append(robot)
            i += 1
    return robots_vector
