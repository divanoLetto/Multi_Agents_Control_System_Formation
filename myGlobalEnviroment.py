import math
import numpy as np
import random

class myGlobalEnviroment:

    globalRobotVector = None

    def __init__(self, robotVector):
        myGlobalEnviroment.globalRobotVector = robotVector

    @staticmethod
    def distanceBetweenRobots(robot1, robot2):
        pos_r1 = robot1.getAbsolutePos()
        pos_r2 = robot2.getAbsolutePos()
        return math.sqrt((pos_r2[0] - pos_r1[0]) ** 2 + (pos_r2[1] - pos_r1[1]) ** 2)

    @staticmethod
    def thereIsRobotInRange(robot1, range):
        founds = []
        for robot2 in myGlobalEnviroment.globalRobotVector:
            if (robot1!=robot2 and myGlobalEnviroment.distanceBetweenRobots(robot1,robot2) < range):
                founds.append(robot2)
        return founds

    @staticmethod
    def getRelativeCosThetaBetweenRobot(robot1, robot2):   # todo deprecated?
        absCosTheta = myGlobalEnviroment.getAbsCosThetaBetweenRobot(robot1, robot2)  # sensori di angolo
        absSinTheta = myGlobalEnviroment.getAbsSinThetaBetweenRobot(robot1, robot2)  # sensori di angolo

        vectorAbsBase = [absCosTheta, absSinTheta]
        C = np.array([[math.cos(robot1.theta), -math.sin(robot1.theta)], [math.sin(robot1.theta), math.cos(robot1.theta)]])
        Cinverse = np.linalg.inv(C)
        vectorRobot1base = Cinverse.dot(vectorAbsBase)
        return vectorRobot1base[0]

    @staticmethod
    def getRelativeSinThetaBetweenRobot(robot1, robot2):   # todo deprecated?
        absCosTheta = myGlobalEnviroment.getAbsCosThetaBetweenRobot(robot1, robot2)  # sensori di angolo
        absSinTheta = myGlobalEnviroment.getAbsSinThetaBetweenRobot(robot1, robot2)  # sensori di angolo

        vectorAbsBase = [absCosTheta, absSinTheta]
        C = np.array(
            [[math.cos(robot1.theta), -math.sin(robot1.theta)], [math.sin(robot1.theta), math.cos(robot1.theta)]])
        Cinverse = np.linalg.inv(C)
        vectorRobot1base = Cinverse.dot(vectorAbsBase)
        return vectorRobot1base[1]

    @staticmethod
    def get_relative_cos_sin_theta_between_robots(robot1, robot2):
        abs_cos_sin_theta = myGlobalEnviroment.get_absolute_cos_sin_theta_between_robots(robot1, robot2)
        vectorAbsBase = abs_cos_sin_theta
        # C = np.array([[math.cos(robot1.theta), -math.sin(robot1.theta)], [math.sin(robot1.theta), math.cos(robot1.theta)]])
        # Cinverse = np.linalg.inv(C)
        Cinverse = np.array([[math.cos(robot1.theta), +math.sin(robot1.theta)], [-math.sin(robot1.theta), math.cos(robot1.theta)]])
        vectorRobot1base = Cinverse.dot(vectorAbsBase)

        return vectorRobot1base

    @staticmethod
    def getAbsCosThetaBetweenRobot(robot1, robot2):  # todo deprecated?
        absDistance = abs(myGlobalEnviroment.distanceBetweenRobots(robot1, robot2))
        if absDistance == 0:
            print("min_fisic_tollerance_distance in funz getAbsCosThetaBetweenRobot")
            rand = random.uniform(-1, 1)
            return rand
        cosAlpha = (robot2.getAbsolutePosX() - robot1.getAbsolutePosX()) / absDistance
        return cosAlpha

    @staticmethod
    def getAbsSinThetaBetweenRobot(robot1, robot2):   # todo deprecated?
        absDistance = abs(myGlobalEnviroment.distanceBetweenRobots(robot1, robot2))
        if absDistance == 0:
            print("min_fisic_tollerance_distance in funz getAbsSinThetaBetweenRobot")
            rand = random.uniform(-1, 1)
            return rand
        sinAlpha = (robot2.getAbsolutePosY() - robot1.getAbsolutePosY()) / absDistance
        return sinAlpha

    @staticmethod
    def get_absolute_cos_sin_theta_between_robots(robot1, robot2):
        absDistance = abs(myGlobalEnviroment.distanceBetweenRobots(robot1, robot2))
        if absDistance == 0:
            print("min_fisic_tollerance_distance in funz get_absolute_cos_sin_theta_between_robots")
            rand_cos = random.uniform(-1, 1)
            rand_sin = math.sqrt(1-rand_cos**2)
            return [rand_cos, rand_sin]
        r1_pos = robot1.getAbsolutePos()
        r2_pos = robot2.getAbsolutePos()
        diff = np.subtract(r2_pos, r1_pos)
        cosAlpha = (diff[0]) / absDistance
        sinAlpha = (diff[1]) / absDistance
        return [cosAlpha, sinAlpha]