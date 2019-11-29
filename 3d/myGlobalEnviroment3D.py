import math
import numpy as np


class myGlobalEnviroment3D:

    globalRobotVector = None

    def __init__(self, robotVector):
        myGlobalEnviroment3D.globalRobotVector = robotVector

    @staticmethod
    def distanceBetweenRobots(robot1, robot2):
        sub_vector = np.subtract(robot2.getAbsolutePos(), robot1.getAbsolutePos())
        return math.sqrt((sub_vector[0]) ** 2 + (sub_vector[1]) ** 2 + (sub_vector[2]) ** 2)

    @staticmethod
    def thereIsRobotInRange(robot1, range):
        founds = []
        for robot2 in myGlobalEnviroment3D.globalRobotVector:
            if robot1 != robot2 and myGlobalEnviroment3D.distanceBetweenRobots(robot1,robot2) < range:
                founds.append(robot2)
        return founds

    @staticmethod
    def get_vers_robot_base(robot1, robot2):

        v = np.subtract(robot2.getAbsolutePos(),  robot1.getAbsolutePos())
        v_x = v[0]
        v_y = v[1]
        v_z = v[2]
        vector = [float('%.8f' % v_x), float('%.8f' % v_y), float('%.8f' % v_z)]
        # vector = [int(v_x),int(v_y),int(v_z) ]

        length = np.linalg.norm(vector)
        unit_v = vector / length

        r_matrix = robot1.get_rotation_matrix()
        r_matrix_inverse = np.linalg.inv(r_matrix)
        
        vers_robot_base = r_matrix_inverse.dot(unit_v)

        # length = np.linalg.norm(vers_robot_base)
        # vers_robot_base = vers_robot_base / length

        tmp = vers_robot_base[0]**2+vers_robot_base[1]**2 + vers_robot_base[2] ** 2

        return vers_robot_base

