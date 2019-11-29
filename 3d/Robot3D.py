import numpy as np
import matplotlib.pyplot as plt
import random
import setting
import math
from abc import ABC, abstractmethod
from myGlobalEnviroment3D import *

min_fisic_tollerance_distance = 0.5


class Robot3D(ABC):
    def __init__(self, posX, posY, posZ):
        self.posX = posX
        self.posY = posY
        self.posZ = posZ
        self.deltaX = 0
        self.deltaY = 0
        self.deltaZ = 0
        self.neighbour = []
        self.role = None
        self.hashRole = {}
        self.formation = None

    def __str__(self):
        string = "Robot with role " + str(self.role) # + " in position " + str(self.getAbsolutePosX()) + "," +str(self.getAbsolutePosY() + "," +str(self.getAbsolutePosZ()))
        return string

    def getPosX(self):
        return self.posX
    def getPosY(self):
        return self.posY
    def getPosZ(self):
        return self.posZ

    def disconnect(self):
        self.neighbour = []

    def getAbsolutePos(self):
        return [self.getAbsolutePosX(), self.getAbsolutePosY(), self.getAbsolutePosZ()]
    @abstractmethod
    def getAbsolutePosX(self):
        pass
    @abstractmethod
    def getAbsolutePosY(self):
        pass
    @abstractmethod
    def getAbsolutePosZ(self):
        pass

    def set_desiderd_velx(self, velx):
        print("Not avaible with this control law robot")
    def set_desiderd_vely(self, velY):
        print("Not avaible with this control law robot")
    def set_desiderd_velz(self, velY):
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
        self.posZ = self.getPosZ() + self.deltaZ
        self.deltaX = 0
        self.deltaY = 0
        self.deltaZ = 0


    @staticmethod
    def getName():
       raise NotImplementedError

    # Pacchetto commander
    def makeCommander(self, formation):
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

    def change_velocity_and_propagate(self, velX, velY, velZ):  # todo useless?
        pass


# DISPLACEMENT CONTROLLED ROBO
class RobotDisplacement3D(Robot3D, ABC):

    def __init__(self, posX, posY, posZ):
        super().__init__(posX, posY, posZ)

    def getAbsolutePosX(self):
        return self.getPosX()
    def getAbsolutePosY(self):
        return self.getPosY()
    def getAbsolutePosZ(self):
        return self.getPosZ()

    def calculateControl(self, stepTime):
        self.calculateControlDisplacement(stepTime)

    @abstractmethod
    def calculateControlDisplacement(self, stepTime):
        pass

    def desiredDistanceX(self, robot):
        return self.hashRole[robot.role][0]
    def desiredDistanceY(self, robot):
        return self.hashRole[robot.role][1]
    def desiredDistanceZ(self, robot):
        return self.hashRole[robot.role][2]


class RobotDisplacementSingleIntegrator3D(RobotDisplacement3D):

    def __init__(self, posX, posY, role=None):
        super().__init__(posX, posY, role)
        self.kp = 2

    def calculateControlDisplacement(self, stepTime):
        uX = self.kp* sum((z.getPosX() - self.getPosX() - self.desiredDistanceX(z)) for z in self.neighbour)
        uY = self.kp * sum((z.getPosY() - self.getPosY() - self.desiredDistanceY(z)) for z in self.neighbour)
        uZ = self.kp* sum((z.getPosZ() - self.getPosZ() - self.desiredDistanceZ(z)) for z in self.neighbour)
        """
        print("The uZ control of the DpSI robot with role " + str(self.role) + " is " + str(uZ))
        print("The uX control of the DpSI robot with role " + str(self.role) + " is " + str(uX))
        print("The uY control of the DpSI robot with role " + str(self.role) + " is " + str(uY))
        """
        self.deltaX = uX * stepTime
        self.deltaY = uY * stepTime
        self.deltaZ = uZ * stepTime

    @staticmethod
    def makeRandomRobot():
        posXr = random.randint(0, 50)
        posYr = random.randint(0, 50)
        posZr = random.randint(0, 50)
        robot = RobotDisplacementSingleIntegrator3D(posXr, posYr, posZr)
        return robot

    @staticmethod
    def getName():
        return "Robot displacement single integrator"


class RobotDisplacementDoubleIntegrator3D(RobotDisplacement3D):

    def __init__(self, posX, posY, posZ, velX=0, velY=0, velZ=0):
        super().__init__(posX, posY, posZ)
        self.velX = velX
        self.velY = velY
        self.velZ = velZ
        self.deltaVelX = 0
        self.deltaVelY = 0
        self.deltaVelZ = 0
        self.kp = 2
        self.kv = 2

    def getVelX(self):
        return self.velX
    def getVelY(self):
        return self.velY
    def getVelZ(self):
        return self.velZ

    def calculateControlDisplacement(self, stepTime):
        uX = self.kp * sum((z.getPosX() - self.getPosX() - self.desiredDistanceX(z)) for z in self.neighbour) + self.kv * sum((z.getVelX() - self.getVelX() + self.desiredVelocityX(z)) for z in self.neighbour)
        uY = self.kp * sum((z.getPosY() - self.getPosY() - self.desiredDistanceY(z)) for z in self.neighbour) + self.kv * sum((z.getVelY() - self.getVelY() + self.desiredVelocityY(z)) for z in self.neighbour)
        uZ = self.kp * sum((z.getPosZ() - self.getPosZ() - self.desiredDistanceZ(z)) for z in self.neighbour) + self.kv * sum((z.getVelZ() - self.getVelZ() + self.desiredVelocityZ(z)) for z in self.neighbour)
        self.deltaX = self.velX * stepTime + (uX * (stepTime**2) / 2)
        self.deltaY = self.velY * stepTime + (uY * (stepTime**2) / 2)
        self.deltaZ = self.velZ * stepTime + (uZ * (stepTime**2) / 2)
        self.deltaVelX = uX * stepTime
        self.deltaVelY = uY * stepTime
        self.deltaVelZ = uZ * stepTime
        """
        if abs(self.deltaVelX) > setting.limit_ux_si:
            self.deltaVelX = setting.limit_ux_si * sign(self.deltaVelX)
        if abs(self.deltaVelY) > setting.limit_uy_si:
            self.deltaVelY = setting.limit_uy_si * sign(self.deltaVelY)
        
        print("The uX control DpDI of the robot with role " + str(self.role) + " is " + str(uX))
        print("The uY control DpDI of the robot with role " + str(self.role) + " is " + str(uY))
        """
        self.rememberControl = [uX, uY]

    def updatePosition(self):
        self.posX = self.getPosX() + self.deltaX
        self.posY = self.getPosY() + self.deltaY
        self.posZ = self.getPosZ() + self.deltaZ
        self.velX = self.getVelX() + self.deltaVelX
        self.velY = self.getVelY() + self.deltaVelY
        self.velZ = self.getVelZ() + self.deltaVelZ
        self.deltaX = 0
        self.deltaY = 0
        self.deltaZ = 0
        self.deltaVelX = 0
        self.deltaVelY = 0
        self.deltaVelZ = 0

    def desiredDistanceX(self, robot):
        return self.hashRole[robot.role][0][0]
    def desiredDistanceY(self, robot):
        return self.hashRole[robot.role][0][1]
    def desiredDistanceZ(self, robot):
        return self.hashRole[robot.role][0][2]

    def desiredVelocityX(self, robot):
        return self.hashRole[robot.role][1][0]
    def desiredVelocityY(self, robot):
        return self.hashRole[robot.role][1][1]
    def desiredVelocityZ(self, robot):
        return self.hashRole[robot.role][1][2]

    @staticmethod
    def makeRandomRobot():
        posXr = random.randint(0, 50)
        posYr = random.randint(0, 50)
        posZr = random.randint(0, 50)
        robot = RobotDisplacementDoubleIntegrator3D(posXr, posYr, posZr)
        return robot

    @staticmethod
    def getName():
        return "Robot displacement double integrator"

    def change_velocity_and_propagate(self, velX, velY, velZ):
        self.set_desiderd_velx(velX)
        self.set_desiderd_vely(velY)
        self.set_desiderd_velz(velZ)
        for near in self.neighbour:
            near.set_desiderd_velx(velX)
            near.set_desiderd_vely(velY)
            near.set_desiderd_vely(velZ)

    def set_desiderd_velx(self, velx):
        for k1, v in self.hashRole.items():
            v[1][0] = int(velx)

    def set_desiderd_vely(self, vely):
        for k1, v in self.hashRole.items():
            v[1][1] = int(vely)

    def set_desiderd_velz(self, velz):
        for k1, v in self.hashRole.items():
            v[1][2] = int(velz)


class RobotDistance3D(Robot3D, ABC):
    def __init__(self, startX, startY, startZ, theta_x=random.random() * 2 * math.pi, theta_y=random.random() * 2 * math.pi, theta_z=random.random() * 2 * math.pi):
        super().__init__(0, 0, 0)
        self.startX = startX
        self.startY = startY
        self.startZ = startZ
        self.theta_x = 0  # TODO thoose are not working!!!!!!!!!!!!!!
        self.theta_y = 0
        self.theta_z = 0

        a = self.theta_x  # todo are this...this?
        b = self.theta_y
        c = self.theta_z

        matrA = [[1, 0, 0],
                 [0, math.cos(a), -math.sin(a)],
                 [0, math.sin(a), math.cos(a)]]
        matrB = [[math.cos(b), 0, -math.sin(b)],
                 [0, 1, 0],
                 [math.sin(b), 0, math.cos(b)]]
        matrC = [[math.cos(c), -math.sin(c), 0],
                 [math.sin(c), math.cos(c), 0],
                 [0, 0, 1]]

        self.r_matrix = np.dot(np.dot(matrA, matrB), matrC)

    def get_rotation_matrix(self):
        return self.r_matrix

    def getAbsolutePosX(self):  # non usare, lenta computazione
        struct = self.getAbsolutePos()
        return struct[0]
    def getAbsolutePosY(self):  # non usare, lenta computazione
        struct = self.getAbsolutePos()
        return struct[1]
    def getAbsolutePosZ(self):  # non usare, lenta computazione
        struct = self.getAbsolutePos()
        return struct[2]
    def getAbsolutePos(self):
        struct_x_y_z = [[self.posX], [self.posY], [self.posZ]]
        # inv_matrix = np.linalg.inv(self.r_matrix)
        struct_new_x_y_z = self.r_matrix.dot(struct_x_y_z)
        abs_pos = np.add(struct_new_x_y_z, [[self.startX], [self.startY], [self.startZ]])
        list = []
        for elem in abs_pos:
             list.append(elem[0])
        return list

    def calculateControl(self, stepTime):
        self.calculateControlDistance(stepTime)

    @abstractmethod
    def calculateControlDistance(self, stepTime):
        pass


class RobotDistanceSingleIntegrator3D(RobotDistance3D):
    def __init__(self, startX, startY, startZ, theta_x=random.random() * 2 * math.pi, theta_y=random.random() * 2 * math.pi, theta_z=random.random() * 2 * math.pi):
        super().__init__(startX, startY, startZ, theta_x, theta_y, theta_z)
        self.kp = 1

    def calculateControlDistance(self, stepTime):
        uX = 0
        uY = 0
        uZ = 0

        for n in self.neighbour:
            absDistance = abs(myGlobalEnviroment3D.distanceBetweenRobots(self, n))  # sensore di distanza
            versor = myGlobalEnviroment3D.get_vers_robot_base(self, n)
            if absDistance == 0:
                print("min_fisic_tollerance_distance in RobotDistanceSingleIntegrator")
                absDistance = min_fisic_tollerance_distance

            u_0 = self.kp * 4*(absDistance**2 - self.hashRole[n.role]**2)/absDistance - 2*(absDistance**2 - self.hashRole[n.role]**2)**2/absDistance**3
            u_1 = self.kp * 2 * ((absDistance - self.hashRole[n.role]) * absDistance - (absDistance - self.hashRole[n.role])**2)/(absDistance)
            u_2 = self.kp * (absDistance - self.hashRole[n.role]) * absDistance
            u = u_0
            uX += u * versor[0]
            uY += u * versor[1]
            uZ += u * versor[2]
        """
        print("The uX control of the DtSI robot with role " + str(self.role) + " is " + str(uX))
        print("The uY control of the DtSI robot with role " + str(self.role) + " is " + str(uY))
        print("The uZ control of the DtSI robot with role " + str(self.role) + " is " + str(uZ))
        """
        self.deltaX = uX * stepTime
        self.deltaY = uY * stepTime
        self.deltaZ = uZ * stepTime

    @staticmethod
    def makeRandomRobot():
        startXr = random.randint(0, 50)
        startYr = random.randint(0, 50)
        startZr = random.randint(0, 50)
        startTheta_x = random.random() * 2 * math.pi
        startTheta_y = random.random() * 2 * math.pi
        startTheta_z = random.random() * 2 * math.pi

        robot = RobotDistanceSingleIntegrator3D(startXr, startYr, startZr, startTheta_x, startTheta_y, startTheta_z)
        return robot

    @staticmethod
    def getName():
        return "Robot distance single integrator"


class RobotDistanceDoubleIntegrator3D(RobotDistance3D):

    def __init__(self, startX, startY, startZ, theta_x=random.random() * 2 * math.pi, theta_y=random.random() * 2 * math.pi, theta_z=random.random() * 2 * math.pi, velX=0, velY=0, velZ=0):
        super().__init__(startX, startY, startZ, theta_x, theta_y, theta_z)
        self.velX = velX
        self.velY = velY
        self.velZ = velZ
        self.deltaVelX = 0
        self.deltaVelY = 0
        self.deltaVelZ = 0
        self.kp = 1
        self.kv = 10

    def getVelX(self):
        return self.velX
    def getVelY(self):
        return self.velY
    def getVelZ(self):
        return self.velY

    def updatePosition(self):
        self.posX = self.getPosX() + self.deltaX
        self.posY = self.getPosY() + self.deltaY
        self.posZ = self.getPosZ() + self.deltaZ
        self.velX = self.getVelX() + self.deltaVelX
        self.velY = self.getVelY() + self.deltaVelY
        self.velZ = self.getVelZ() + self.deltaVelZ
        self.deltaX = 0
        self.deltaY = 0
        self.deltaZ = 0
        self.deltaVelX = 0
        self.deltaVelY = 0
        self.deltaVelZ = 0

    def calculateControlDistance(self, stepTime):
        uX = 0
        uY = 0
        uZ = 0
        for n in self.neighbour:
            absDistance = abs(myGlobalEnviroment3D.distanceBetweenRobots(self, n))  # sensore di distanza
            versor = myGlobalEnviroment3D.get_vers_robot_base(self, n)
            if absDistance == 0:
                print("min_fisic_tollerance_distance in RobotDistanceSingleIntegrator")
                absDistance = min_fisic_tollerance_distance

            u_0 = (4 * (absDistance ** 2 - self.desideredDistance(n) ** 2) / absDistance) - (2 * (absDistance ** 2 - self.desideredDistance(n) ** 2) ** 2 / absDistance ** 3)
            u_2 = self.kp * (absDistance - self.desideredDistance(n)) * absDistance
            u = u_2
            uX += u * versor[0]
            uY += u * versor[1]
            uZ += u * versor[2]

        uX -= self.Dx()
        uY -= self.Dy()
        uZ -= self.Dz()
        """
        print("The uX control of the DtDI robot with role " + str(self.role) + " is " + str(uX))
        print("The uY control of the DtDI robot with role " + str(self.role) + " is " + str(uY))
        print("The uZ control of the DtDI robot with role " + str(self.role) + " is " + str(uZ))
        """
        self.deltaX = self.velX * stepTime + (uX * (stepTime ** 2) / 2)
        self.deltaY = self.velY * stepTime + (uY * (stepTime ** 2) / 2)
        self.deltaZ = self.velZ * stepTime + (uZ * (stepTime ** 2) / 2)
        self.deltaVelX = uX * stepTime
        self.deltaVelY = uY * stepTime
        self.deltaVelZ = uZ * stepTime

    def Dx(self):
        return self.kv * self.getVelX()
    def Dy(self):
        return self.kv * self.getVelY()
    def Dz(self):
        return self.kv * self.getVelZ()

    def desideredDistance(self, n):
        return self.hashRole[n.role][0][0]

    @staticmethod
    def makeRandomRobot():
        startXr = random.randint(0, 50)
        startYr = random.randint(0, 50)
        startZr = random.randint(0, 50)
        startTheta_x = random.random() * 2 * math.pi
        startTheta_y = random.random() * 2 * math.pi
        startTheta_z = random.random() * 2 * math.pi
        robot = RobotDistanceDoubleIntegrator3D(startXr, startYr, startZr, startTheta_x, startTheta_y, startTheta_z)
        return robot

    @staticmethod
    def getName():
        return "Robot distance double integrator"




