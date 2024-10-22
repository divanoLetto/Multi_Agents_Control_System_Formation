import numpy as np
import matplotlib.pyplot as plt
import random
import math
from myGlobalEnviroment import *
from abc import ABC, abstractmethod

min_fisic_tollerance_distance = 0.5


class Robot(ABC):
    def __init__(self, posX, posY):
        self.posX = posX
        self.posY = posY
        self.deltaX = 0
        self.deltaY = 0
        self.neighbour = []
        self.role = None
        self.hashRole = {}
        self.formation = None
        self.kp = 1
        self.error = None

    def __str__(self):
        string = "Robot with role " + str(self.role)  # + " in position " + str(self.getAbsolutePosX()) + "," +str(self.getAbsolutePosY())
        return string

    def getPosX(self):
        return self.posX
    def getPosY(self):
        return self.posY
    def getAbsolutePos(self):
        return [self.getAbsolutePosX(), self.getAbsolutePosY()]
    def get_error(self):
        return self.error

    def disconnect(self):
        self.neighbour = []

    @abstractmethod
    def getAbsolutePosX(self):
        pass
    @abstractmethod
    def getAbsolutePosY(self):
        pass

    def set_desiderd_velx(self, velx):
        print("Not avaible with this control law robot")

    def set_desiderd_vely(self, velY):
        print("Not avaible with this control law robot")

    @staticmethod
    def makeRandomRobot():
        pass

    @staticmethod
    def connect(robot1, robot2):
        robot1.neighbour.append(robot2)
        robot2.neighbour.append(robot1)

    def setRole(self, integerRole):
        self.role = integerRole

    @abstractmethod
    def calculateControl(self, stepTime):
        pass

    def updatePosition(self):
        self.posX = self.getPosX()+self.deltaX
        self.posY = self.getPosY()+self.deltaY
        self.deltaX = 0
        self.deltaY = 0

    @staticmethod
    def getName():
       raise NotImplementedError

    # Pacchetto commander
    def makeCommander(self, formation):  # si puo' mettere un controllo con una variabile statica sul numero dei comman
        self.setFormation(formation)

    def setFormation(self, formation):
        self.formation = formation

    def makeSpanningTree(self):
        if self.isCommander():
            self.formation.makeSpanningTree(self)
        else:
            raise PermissionError

    def makeCostraint(self):
        if self.isCommander():
            self.formation.makeFormationCostraint(self)
        else:
            raise PermissionError

    def isCommander(self):
        if self.formation:
            return True
        return False

    def change_velocity_and_propagate(self, velX, velY):
        print("Not avaible with this control law robot")


# DISPLACEMENT CONTROLLED ROBO
class RobotDisplacement(Robot, ABC):

    def __init__(self, posX, posY):
        super().__init__(posX, posY)

    def getAbsolutePosX(self):
        return self.getPosX()
    def getAbsolutePosY(self):
        return self.getPosY()

    def calculateControl(self, stepTime):
        self.calculateControlDisplacement(stepTime)

    @abstractmethod
    def calculateControlDisplacement(self, stepTime):
        pass

    def desiredDistanceX(self, robot):
        return self.hashRole[robot.role][0]
    def desiredDistanceY(self, robot):
        return self.hashRole[robot.role][1]


class RobotDisplacementSingleIntegrator(RobotDisplacement):

    def __init__(self, posX, posY):
        super().__init__(posX, posY)
        self.kp = 2

    def calculateControlDisplacement(self, stepTime):
        uX = self.kp * sum((z.getPosX() - self.getPosX() - self.desiredDistanceX(z)) for z in self.neighbour)
        uY = self.kp * sum((z.getPosY() - self.getPosY() - self.desiredDistanceY(z)) for z in self.neighbour)

        """
        if abs(uX) > setting.limit_ux_si:
            uX = setting.limit_ux_si * sign(uX)
        if abs(uY) > setting.limit_uy_si:
            uY = setting.limit_uy_si * sign(uY)
        print("The uX control of the DpSI robot with role " + str(self.role) + " is " + str(uX))
        print("The uY control of the DpSI robot with role " + str(self.role) + " is " + str(uY))
        """
        self.deltaX = uX * stepTime
        self.deltaY = uY * stepTime

        self.error = math.sqrt(sum(np.linalg.norm([z.getPosX() - self.getPosX() - self.desiredDistanceX(z), z.getPosY() - self.getPosY() - self.desiredDistanceY(z)])**2 for z in self.neighbour))

    @staticmethod
    def makeRandomRobot():
        posXr = random.randint(0, 50)
        posYr = random.randint(0, 50)
        robot = RobotDisplacementSingleIntegrator(posXr, posYr)
        return robot

    @staticmethod
    def getName():
        return "Robot displacement single integrator"


class RobotDisplacementDoubleIntegrator(RobotDisplacement):

    def __init__(self, posX, posY, velX=0, velY=0):
        super().__init__(posX, posY)
        self.velX = velX
        self.velY = velY
        self.deltaVelX = 0
        self.deltaVelY = 0
        self.kp = 2
        self.kv = 2

    def getVelX(self):
        return self.velX
    def getVelY(self):
        return self.velY

    def calculateControlDisplacement(self, stepTime):
        uX = self.kp * sum((z.getPosX() - self.getPosX() - self.desiredDistanceX(z)) for z in self.neighbour) + self.kv * sum((z.getVelX() - self.getVelX() + self.desiredVelocityX(z)) for z in self.neighbour)
        uY = self.kp * sum((z.getPosY() - self.getPosY() - self.desiredDistanceY(z)) for z in self.neighbour) + self.kv * sum((z.getVelY() - self.getVelY() + self.desiredVelocityY(z)) for z in self.neighbour)
        self.deltaX = self.velX * stepTime + (uX * (stepTime**2) / 2)
        self.deltaY = self.velY * stepTime + (uY * (stepTime**2) / 2)
        self.deltaVelX = uX * stepTime
        self.deltaVelY = uY * stepTime

        self.error = math.sqrt(sum(np.linalg.norm([z.getPosX() - self.getPosX() - self.desiredDistanceX(z), z.getPosY() - self.getPosY() - self.desiredDistanceY(z)])**2 for z in self.neighbour))
        """
        if abs(self.deltaVelX) > setting.limit_ux_si:
            self.deltaVelX = setting.limit_ux_si * sign(self.deltaVelX)
        if abs(self.deltaVelY) > setting.limit_uy_si:
            self.deltaVelY = setting.limit_uy_si * sign(self.deltaVelY)
        
        print("The uX control DpDI of the robot with role " + str(self.role) + " is " + str(uX))
        print("The uY control DpDI of the robot with role " + str(self.role) + " is " + str(uY))
        """

    def updatePosition(self):
        self.posX = self.getPosX() + self.deltaX
        self.posY = self.getPosY() + self.deltaY
        self.velX = self.getVelX() + self.deltaVelX
        self.velY = self.getVelY() + self.deltaVelY
        self.deltaX = 0
        self.deltaY = 0
        self.deltaVelX = 0
        self.deltaVelY = 0

    def desiredDistanceX(self, robot):
        return self.hashRole[robot.role][0][0]
    def desiredDistanceY(self, robot):
        return self.hashRole[robot.role][0][1]

    def desiredVelocityX(self, robot):
        return self.hashRole[robot.role][1][0]
    def desiredVelocityY(self, robot):
        return self.hashRole[robot.role][1][1]

    @staticmethod
    def makeRandomRobot():
        posXr = random.randint(0, 50)
        posYr = random.randint(0, 50)
        robot = RobotDisplacementDoubleIntegrator(posXr, posYr)
        return robot

    @staticmethod
    def getName():
        return "Robot displacement double integrator"

    def change_velocity_and_propagate(self, velX, velY):
        self.set_desiderd_velx(velX)
        self.set_desiderd_vely(velY)
        for near in self.neighbour:
            near.set_desiderd_velx(velX)
            near.set_desiderd_vely(velY)

    def set_desiderd_velx(self, velx):
        for k1, v in self.hashRole.items():
            v[1][0] = int(velx)

    def set_desiderd_vely(self, vely):
        for k1, v in self.hashRole.items():
            v[1][1] = int(vely)


class RobotDisplacementUnicycle(RobotDisplacement):

    def __init__(self, posX, posY, startAplha=0):
        super().__init__(posX, posY)
        self.alpha = startAplha
        self.deltaAlpha = 0
        self.kp = 2

    def calculateControlDisplacement(self, stepTime):
        linearux = self.kp * sum((z.getPosX() - self.getPosX() - self.desiredDistanceX(z)) for z in self.neighbour)
        linearuy = self.kp * sum((z.getPosY() - self.getPosY() - self.desiredDistanceY(z)) for z in self.neighbour)

        l = 1
        v = math.cos(-self.alpha) * linearux - math.sin(-self.alpha) * linearuy
        w = (math.sin(-self.alpha) * linearux + math.cos(-self.alpha) * linearuy) / l

        self.deltaX = v * stepTime * math.cos(self.alpha)
        self.deltaY = v * stepTime * math.sin(self.alpha)

        self.deltaAlpha = w * stepTime

        self.error = math.sqrt(sum(np.linalg.norm([z.getPosX() - self.getPosX() - self.desiredDistanceX(z), z.getPosY() - self.getPosY() - self.desiredDistanceY(z)])**2 for z in self.neighbour))
        """
        print("The v control of the DpU robot with role " + str(self.role) + " is " + str(v))
        print("The w control of the DpU robot with role " + str(self.role) + " is " + str(w))
        """
    def updatePosition(self):
        super().updatePosition()
        self.alpha += self.deltaAlpha
        self.deltaAlpha = 0
        print("the new theta is " + str(self.alpha))

    @staticmethod
    def makeRandomRobot():
        posXr = random.randint(0, 50)
        posYr = random.randint(0, 50)
        theta = random.random()*2* math.pi
        robot = RobotDisplacementUnicycle(posXr, posYr, theta)
        return robot

    @staticmethod
    def getName():
        return "Robot displacement unicycle"


# DISTANCE CONTROLED ROBOT
class RobotDistance(Robot, ABC):
    def __init__(self, startX, startY, theta=random.random() * 2 * math.pi):
        super().__init__(0, 0)
        self.startX = startX
        self.startY = startY
        self.theta = theta

    def getAbsolutePosX(self):
        return self.startX + self.getPosX()*math.cos(self.theta) - self.getPosY()*math.sin(self.theta)
    def getAbsolutePosY(self):
        return self.startY + self.getPosY()*math.cos(self.theta) + self.getPosX()*math.sin(self.theta)

    def calculateControl(self, stepTime):
        self.calculateControlDistance(stepTime)

    @abstractmethod
    def calculateControlDistance(self, stepTime):
        pass


class RobotDistanceSingleIntegrator(RobotDistance):
    def __init__(self, startX, startY, theta=random.random() * 2 * math.pi):
        super().__init__(startX, startY, theta)
        self.kp = 1

    @staticmethod
    def makeRandomRobot():
        startXr = random.randint(0, 50)
        startYr = random.randint(0, 50)
        startTheta = random.random() * 2 * math.pi

        robot = RobotDistanceSingleIntegrator(startXr, startYr, startTheta)
        return robot

    # todo aumentare performance
    def calculateControlDistance(self, stepTime):
        uX = 0
        uY = 0

        for n in self.neighbour:
            absDistance = abs(myGlobalEnviroment.distanceBetweenRobots(self, n))  # sensore di distanza
            if absDistance == 0:
                print("min_fisic_tollerance_distance in RobotDistanceSingleIntegrator")
                absDistance = min_fisic_tollerance_distance

            cos_sin_theta = myGlobalEnviroment.get_relative_cos_sin_theta_between_robots(self, n)
            u = self.kp * 4*(absDistance**2 - self.hashRole[n.role]**2)/absDistance - 2*(absDistance**2 - self.hashRole[n.role]**2)**2/absDistance**3
            uX += u * cos_sin_theta[0]
            uY += u * cos_sin_theta[1]
        """
        print("The uX control of the DtSI robot with role " + str(self.role) + " is " + str(uX))
        print("The uY control of the DtSI robot with role " + str(self.role) + " is " + str(uY))
        """
        self.deltaX = uX * stepTime
        self.deltaY = uY * stepTime

        self.error = 0
        for n in self.neighbour:
            self.error += (abs(abs(myGlobalEnviroment.distanceBetweenRobots(self, n))-self.hashRole[n.role]))**2
        self.error = math.sqrt(self.error)

    @staticmethod
    def getName():
        return "Robot distance single integrator"


class RobotDistanceDoubleIntegrator(RobotDistance):

    def __init__(self, startX, startY, theta=random.random() * 2 * math.pi, velX=0, velY=0):
        super().__init__(startX, startY, theta)
        self.velX = velX
        self.velY = velY
        self.deltaVelX = 0
        self.deltaVelY = 0
        self.kp = 4
        self.kv = 12

    def getVelX(self):
        return self.velX
    def getVelY(self):
        return self.velY

    def updatePosition(self):
        self.posX = self.getPosX() + self.deltaX
        self.posY = self.getPosY() + self.deltaY
        self.velX = self.getVelX() + self.deltaVelX
        self.velY = self.getVelY() + self.deltaVelY
        self.deltaX = 0
        self.deltaY = 0
        self.deltaVelX = 0
        self.deltaVelY = 0

    def calculateControlDistance(self, stepTime):
        uX = 0
        uY = 0

        for n in self.neighbour:
            absDistance = abs(myGlobalEnviroment.distanceBetweenRobots(self, n))  # sensore di distanza
            if absDistance == 0:
                print("min_fisic_tollerance_distance in RobotDistanceDoubleIntegrator")
                absDistance = min_fisic_tollerance_distance

            cos_sin_theta = myGlobalEnviroment.get_relative_cos_sin_theta_between_robots(self, n)

            u = (4 * (absDistance ** 2 - self.desideredDistance(n) ** 2) / absDistance) - (2 * (absDistance ** 2 - self.desideredDistance(n) ** 2) ** 2 / absDistance ** 3)
            uX += u * cos_sin_theta[0]
            uY += u * cos_sin_theta[1]

        uX -= self.Dx()
        uY -= self.Dy()
        """
        print("The uX control of the DtDI robot with role " + str(self.role) + " is " + str(uX))
        print("The uY control of the DtDI robot with role " + str(self.role) + " is " + str(uY))
        """
        self.deltaX = self.velX * stepTime + (uX * (stepTime ** 2) / 2)
        self.deltaY = self.velY * stepTime + (uY * (stepTime ** 2) / 2)
        self.deltaVelX = uX * stepTime
        self.deltaVelY = uY * stepTime

        self.error = 0
        for n in self.neighbour:
            self.error += abs(abs(myGlobalEnviroment.distanceBetweenRobots(self, n)) - self.desideredDistance(n)) ** 2
        self.error = math.sqrt(self.error)

    def Dx(self):
        return self.kv * self.getVelX()
    def Dy(self):
        return self.kv * self.getVelY()

    def desideredDistance(self, n):
        return self.hashRole[n.role][0][0]

    @staticmethod
    def makeRandomRobot():
        startXr = random.randint(0, 50)
        startYr = random.randint(0, 50)
        startTheta = random.randint(0, 90)
        robot = RobotDistanceDoubleIntegrator(startXr, startYr, startTheta)
        return robot

    @staticmethod
    def getName():
        return "Robot distance double integrator"


class RobotDistanceUnicycle(RobotDistance):

    def __init__(self, posX, posY, theta=0, startAplha=0):
        super().__init__(posX, posY, theta)
        self.alpha = startAplha
        self.deltaAlpha = 0
        self.kp = 1

    def updatePosition(self):
        super().updatePosition()
        self.alpha += self.deltaAlpha
        self.deltaAlpha = 0

    @staticmethod
    def makeRandomRobot():
        posXr = random.randint(0, 50)
        posYr = random.randint(0, 50)
        theta = random.random() * 2 * math.pi
        alpha = random.random() * 2 * math.pi
        robot = RobotDistanceUnicycle(posXr, posYr, theta=theta, startAplha=alpha)
        return robot

    def calculateControlDistance(self, stepTime):
        linearux = 0
        linearuy = 0

        for n in self.neighbour:
            absDistance = abs(myGlobalEnviroment.distanceBetweenRobots(self, n))  # sensore di distanza
            if absDistance == 0:
                print("min_fisic_tollerance_distance in RobotDistanceUnycicle")
                absDistance = min_fisic_tollerance_distance

            cos_sin_theta = myGlobalEnviroment.get_relative_cos_sin_theta_between_robots(self, n)

            u = self.kp * (4*(absDistance**2 - self.hashRole[n.role]**2)/absDistance) - (2*(absDistance**2 - self.hashRole[n.role]**2)**2/absDistance**3)
            linearux += u * cos_sin_theta[0]
            linearuy += u * cos_sin_theta[1]

        l = 1
        v = math.cos(-self.alpha) * linearux - math.sin(-self.alpha) * linearuy
        w = (math.sin(-self.alpha) * linearux + math.cos(-self.alpha) * linearuy) / l

        self.deltaX = v * stepTime * math.cos(self.alpha)
        self.deltaY = v * stepTime * math.sin(self.alpha)

        self.deltaAlpha = w * stepTime

        self.error = 0
        for n in self.neighbour:
            self.error += abs(abs(myGlobalEnviroment.distanceBetweenRobots(self, n)) - self.hashRole[n.role]) ** 2
        self.error = math.sqrt(self.error)
        """
        print("The v control of the DtU robot with role " + str(self.role) + " is " + str(v))
        print("The w control of the DtU robot with role " + str(self.role) + " is " + str(w))
        """

    @staticmethod
    def getName():
        return "Robot distance unicycle"
