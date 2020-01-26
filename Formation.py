from numpy import iinfo
from myGlobalEnviroment import *
from abc import ABC, abstractmethod
from Robot import *
from utils import *
from munkres import Munkres, print_matrix
import numpy


class Formation(ABC):
    def __init__(self, numRobots):
        self.numRobots = numRobots

    def makeFormationCostraint(self, robotCommander):
        pass

    def setFormationCostraint(self, robotVector):
        pass

    def makeSpanningTree(self, robotCommander):
        robots = []
        robotCommander.neighbour.clear()
        robotCommander.hashRole.clear()
        robots.append(robotCommander)

        i = 0
        robotCommander.setRole(i)
        n = self.numRobots

        start_range = 5
        ranges = []
        for j in range(n):
            ranges.append(start_range)

        while len(robots) < n:
            near_robots_struct = []
            for rob in robots:
                close_rob = myGlobalEnviroment.thereIsRobotInRange(rob, ranges[rob.role])
                for r in close_rob:
                    near_robots_struct.append([rob, r])  # della forma robot che ha trovato - robot trovato
                ranges[rob.role] += 1
            for rob_struct in near_robots_struct:
                if are_not_connected(robot_list=robots, robot=rob_struct[1]):
                    rob_struct[1].neighbour.clear()
                    Robot.connect(rob_struct[0], rob_struct[1])
                    rob_struct[1].setRole(i + 1)
                    robotCommander.hashRole.clear()
                    robots.append(rob_struct[1])
                    i += 1

    @staticmethod
    def specifyFormationRobot(posX, posY):
        pass

    def uncompatibility_number_robots(self, num_robots):
        if self.numRobots == num_robots:
            return False
        print("Incompatible number of robots")
        return True


class FormationDisplacement(Formation, ABC):

    def make_munkres(self, robotVector):  
        matrix = self.calc_matrix_cost(robotVector)
        m = Munkres()
        indexes = m.compute(matrix)
        print_matrix(matrix, msg='Lowest cost through this matrix:')
        total = 0
        for row, column in indexes:
            value = matrix[row][column]
            total += value
            print(f'({row}, {column}) -> {value}')
        print(f'total cost: {total}')
        self.reset_roles_connections(robotVector, indexes)

    def makeFormationCostraint(self, robotCommander):
        robotVector = []

        list = []
        robotVector.append(robotCommander)
        list.append(robotCommander)
        while list != []:
            robot = list.pop()
            for elem in robot.neighbour:
                if elem not in robotVector:
                    list.append(elem)
                    robotVector.append(elem)
        self.make_munkres(robotVector)
        self.setFormationCostraint(robotVector)

    def calc_matrix_cost(self, robotVector):
        pass

    def reset_roles_connections(self, robotVector, indexes):
        i = 0
        for robot in robotVector:
            robot.setRole(indexes[i][1])
            i += 1

    def calcolate_baricenter(self, robotVector):
        xG = 0
        yG = 0
        for naer in robotVector:
            xG += naer.getAbsolutePosX()
            yG += naer.getAbsolutePosY()
        xG = xG/self.numRobots
        yG = yG / self.numRobots
        return [xG, yG]


class SquareFormationDisplacement(FormationDisplacement):
    def __init__(self, side):
        super().__init__(4)
        self.side = side

    def calc_matrix_cost(self, robot_vector):

        robot_vector = sorted(robot_vector, key=lambda item: item.role)
        baricenter = self.calcolate_baricenter(robot_vector)
        points = []
        punto0 = [baricenter[0] - self.side/2, baricenter[1] - self.side/2]
        punto1 = [baricenter[0] - self.side/2, baricenter[1] + self.side/2]
        punto2 = [baricenter[0] + self.side/2, baricenter[1] + self.side/2]
        punto3 = [baricenter[0] + self.side/2, baricenter[1] - self.side/2]
        points.append(punto0)
        points.append(punto1)
        points.append(punto2)
        points.append(punto3)

        matrix = []
        for robot in robot_vector:
            robot_roles_assigment = []
            for point in points:
                robot_role = distance_between_point(robot.getAbsolutePos(), point)
                robot_roles_assigment.append(robot_role)
            matrix.append(robot_roles_assigment)

        return matrix


class SquareFormationDisplacementSingleIntegrator(SquareFormationDisplacement):

    def setFormationCostraint(self, robotVector):
        robotVector = sorted(robotVector, key=lambda item: item.role)

        robotVector[0].disconnect()
        robotVector[1].disconnect()
        robotVector[2].disconnect()
        robotVector[3].disconnect()
        Robot.connect(robotVector[0], robotVector[1])
        Robot.connect(robotVector[1], robotVector[2])
        Robot.connect(robotVector[2], robotVector[3])

        robotVector[0].hashRole[1] = [0, self.side]
        robotVector[1].hashRole[0] = [0, -self.side]
        robotVector[1].hashRole[2] = [self.side, 0]
        robotVector[2].hashRole[1] = [-self.side, 0]
        robotVector[2].hashRole[3] = [0, -self.side]
        robotVector[3].hashRole[2] = [0, self.side]

    def makeFormationRobot(self):
        return RobotDisplacementSingleIntegrator.makeRandomRobot()

    @staticmethod
    def specifyFormationRobot(posX, posY):
        return RobotDisplacementSingleIntegrator(posX, posY)


class SquareFormationDisplacementDoubleIntegrator(SquareFormationDisplacement):
    def __init__(self, side, desVelX=0, desVelY=0):
        super().__init__(4)
        self.side = side
        self.desVelX = desVelX
        self.desVelY = desVelY

    def setFormationCostraint(self, robotVector):
        robotVector = sorted(robotVector, key=lambda item: item.role)

        robotVector[0].disconnect()
        robotVector[1].disconnect()
        robotVector[2].disconnect()
        robotVector[3].disconnect()
        Robot.connect(robotVector[0], robotVector[1])
        Robot.connect(robotVector[1], robotVector[2])
        Robot.connect(robotVector[2], robotVector[3])

        robotVector[0].hashRole[1] = [[0, self.side], [self.desVelX, self.desVelY]]
        robotVector[1].hashRole[0] = [[0, -self.side], [self.desVelX, self.desVelY]]
        robotVector[1].hashRole[2] = [[self.side, 0], [self.desVelX, self.desVelY]]
        robotVector[2].hashRole[1] = [[-self.side, 0], [self.desVelX, self.desVelY]]
        robotVector[2].hashRole[3] = [[0, -self.side], [self.desVelX, self.desVelY]]
        robotVector[3].hashRole[2] = [[0, self.side], [self.desVelX, self.desVelY]]

    def makeFormationRobot(self):
        return RobotDisplacementDoubleIntegrator.makeRandomRobot()

    @staticmethod
    def specifyFormationRobot( posX, posY):
        return RobotDisplacementDoubleIntegrator(posX, posY)


class SquareFormationDisplacementUnicycle(SquareFormationDisplacementSingleIntegrator):

    def makeFormationRobot(self):
        return RobotDisplacementUnicycle.makeRandomRobot()

    @staticmethod
    def specifyFormationRobot(posX, posY):
        return RobotDisplacementUnicycle(posX, posY)


class LinearFormationDisplacement(FormationDisplacement):
    def __init__(self, side, num_robots):
        super().__init__(num_robots)
        self.side = side

    def calc_matrix_cost(self, robotVector):
        baricenter = self.calcolate_baricenter(robotVector)

        if len(robotVector)%2 != 0:
            starting_point_x = baricenter[0] - math.floor(len(robotVector)/2) * self.side
        else:
            starting_point_x = baricenter[0] - self.side/2 - (len(robotVector) / 2) * self.side
        points = []

        for i in range(len(robotVector)):
            if i == 0:
                point = [starting_point_x, baricenter[1]]
            else:
                point = [points[i-1][0]+self.side, points[i-1][1]]
            points.append(point)

        matrix = []
        for robot in robotVector:
            robot_roles_assigment = []
            for point in points:
                robot_role = distance_between_point(robot.getAbsolutePos(), point)
                robot_roles_assigment.append(robot_role)
            matrix.append(robot_roles_assigment)

        return matrix


class LinearHorizontalFormationDisplacementSingleIntegrator(LinearFormationDisplacement):

    def setFormationCostraint(self, robotVector):
        robotVector = sorted(robotVector, key=lambda item: item.role)

        for i in range(len(robotVector)):
            robotVector[i].disconnect()
        for i in range(len(robotVector) - 1):
            Robot.connect(robotVector[i], robotVector[i+1])

        for i in range(len(robotVector)):
            if i == 0:
                robotVector[i].hashRole[i + 1] = [self.side, 0]
            elif i == (len(robotVector)-1):
                robotVector[i].hashRole[i - 1] = [-self.side, 0]
            else:
                robotVector[i].hashRole[i+1] = [self.side, 0]
                robotVector[i].hashRole[i-1] = [-self.side, 0]

    def makeFormationRobot(self):
        return RobotDisplacementSingleIntegrator.makeRandomRobot()

    @staticmethod
    def specifyFormationRobot( posX, posY):
        return RobotDisplacementSingleIntegrator(posX, posY)


class LinearHorizontalFormationDisplacementDoubleIntegrator(LinearFormationDisplacement):
    def __init__(self, side, num_robots, desVelX=0, desVelY=0):
        super().__init__(side, num_robots)
        self.desVelX = desVelX
        self.desVelY = desVelY

    def setFormationCostraint(self, robotVector):
        robotVector = sorted(robotVector, key=lambda item: item.role)

        for i in range(len(robotVector)):
            robotVector[i].disconnect()
        for i in range(len(robotVector) - 1):
            Robot.connect(robotVector[i], robotVector[i+1])

        for i in range(len(robotVector)):
            if i == 0:
                robotVector[i].hashRole[i + 1] = [[self.side, 0], [self.desVelX, self.desVelY]]
            elif i == (len(robotVector) - 1):
                robotVector[i].hashRole[i - 1] = [[-self.side, 0], [self.desVelX, self.desVelY]]
            else:
                robotVector[i].hashRole[i + 1] = [[self.side, 0], [self.desVelX, self.desVelY]]
                robotVector[i].hashRole[i - 1] = [[-self.side, 0], [self.desVelX, self.desVelY]]

    def makeFormationRobot(self):
        return RobotDisplacementDoubleIntegrator.makeRandomRobot()

    @staticmethod
    def specifyFormationRobot(posX, posY):
        return RobotDisplacementDoubleIntegrator(posX, posY)


class LinearHorizontalFormationDisplacementUnicycle(LinearHorizontalFormationDisplacementSingleIntegrator):

    def makeFormationRobot(self):
        return RobotDisplacementUnicycle.makeRandomRobot()

    @staticmethod
    def specifyFormationRobot(posX, posY):
        return RobotDisplacementUnicycle(posX, posY)


class FreeFormationDisplacement(FormationDisplacement, ABC):
    def __init__(self, ptsFormation, lines):
        num_robots = len(ptsFormation)
        super().__init__(num_robots)
        self.hash_role = {}
        self.ptsFormation = ptsFormation
        self.lines = lines  # salva le linee per la creazione dei vincoli tra punti
        for i in range(num_robots):
            self.hash_role[i] = {}

    def calcolate_formation_baricenter(self):
        xG = 0
        yG = 0
        for i in self.ptsFormation:
            xG += i[1][0]
            yG += i[1][1]
        xG = xG / self.numRobots
        yG = yG / self.numRobots
        return [xG, yG]

    def calc_matrix_cost(self, robotVector):
        robotBaricenter = self.calcolate_baricenter(robotVector)
        formationBaricenter = self.calcolate_formation_baricenter()
        difference = numpy.subtract(formationBaricenter, robotBaricenter)
        points = []
        for i in range(len(self.ptsFormation)):
            p_i = self.ptsFormation[i][1] - difference
            points.append(p_i)

        matrix = []
        for robot in robotVector:
            robot_roles_assigment = []
            for point in points:
                robot_role = distance_between_point(robot.getAbsolutePos(), point)
                robot_roles_assigment.append(robot_role)
            matrix.append(robot_roles_assigment)

        return matrix


class FreeFormationDisplacementSingleIntegrator(FreeFormationDisplacement):

    def __init__(self, ptsFormation, lines):
        super().__init__(ptsFormation, lines)

    def setFormationCostraint(self, robot_vector):
        robot_vector = sorted(robot_vector, key=lambda item: item.role)

        for robot in robot_vector:  # todo responsabilità del makeSpanningTree
            robot.disconnect()

        for robot in robot_vector:
            for linea in self.lines:
                if linea[0] == robot.role:
                    punto1 = None
                    punto2 = None
                    for punto in self.ptsFormation:
                        if punto[0] == linea[0]:
                            punto1 = punto
                        elif punto[0] == linea[1]:
                            punto2 = punto
                    dx = punto2[1][0] - punto1[1][0]
                    dy = punto2[1][1] - punto1[1][1]
                    robot.hashRole[linea[1]] = [dx, dy]
                    robot2 = find_robot_by_role(robot_vector, linea[1])
                    robot2.hashRole[linea[0]] = [-dx, -dy]  # setto i vincolo anche nel verso opposto
                    Robot.connect(robot, robot2)

    def makeFormationRobot(self):
        return RobotDisplacementSingleIntegrator.makeRandomRobot()

    @staticmethod
    def specifyFormationRobot( posX, posY):
        return RobotDisplacementSingleIntegrator(posX, posY)


class FreeFormationDisplacementDoubleIntegrator(FreeFormationDisplacement):

    def __init__(self, ptsFormation, lines, desVelX=0, desVelY=0):
        super().__init__(ptsFormation, lines)
        self.desVelX = desVelX
        self.desVelY = desVelY

    def setFormationCostraint(self, robot_vector):

        robot_vector = sorted(robot_vector, key=lambda item: item.role)

        for robot in robot_vector:
            robot.disconnect()

        for robot in robot_vector:
            for linea in self.lines:
                if linea[0] == robot.role:
                    punto1 = None
                    punto2 = None
                    for punto in self.ptsFormation:
                        if punto[0] == linea[0]:
                            punto1 = punto
                        elif punto[0] == linea[1]:
                            punto2 = punto
                    dx = punto2[1][0] - punto1[1][0]
                    dy = punto2[1][1] - punto1[1][1]
                    robot.hashRole[linea[1]] = [[dx, dy], [self.desVelX, self.desVelY]]
                    robot2 = find_robot_by_role(robot_vector, linea[1])
                    robot2.hashRole[linea[0]] = [[-dx, -dy], [self.desVelX, self.desVelY]]  # setto i vincolo anche nel verso opposto
                    Robot.connect(robot, robot2)

    def makeFormationRobot(self):
        return RobotDisplacementDoubleIntegrator.makeRandomRobot()

    @staticmethod
    def specifyFormationRobot(posX, posY):
        return RobotDisplacementDoubleIntegrator(posX, posY)


class FreeFormationDisplacementUnicycle(FreeFormationDisplacementSingleIntegrator):

    def makeFormationRobot(self):
        return RobotDisplacementUnicycle.makeRandomRobot()

    @staticmethod
    def specifyFormationRobot(posX, posY):
        return RobotDisplacementUnicycle(posX, posY)


class FormationDistance(Formation, ABC):

    def makeFormationCostraint(self, robotCommander):
        robotVector = []

        list = []
        robotVector.append(robotCommander)
        list.append(robotCommander)
        while list != []:
            robot = list.pop()
            for elem in robot.neighbour:
                if elem not in robotVector:
                    list.append(elem)
                    robotVector.append(elem)
        self.make_munkres(robotVector)
        self.setFormationCostraint(robotVector)

    def make_munkres(self, robotVector):
        matrixs = self.calc_matrixs_cost(robotVector)
        best_idexes = []
        min = math.inf
        i = 0
        for matrix in matrixs:
            i += 1
            m = Munkres()
            indexes = m.compute(matrix)
            print_matrix(matrix, msg='Lowest cost through this matrix:')
            total = 0
            for row, column in indexes:
                value = matrix[row][column]
                total += value
                print(f'({row}, {column}) -> {value}')
            print(f'total cost: {total}')
            if total < min:
                min = total
                best_idexes = indexes
        #print("the best matrix was the number: " + str(i))
        self.reset_roles_connections(robotVector, best_idexes)

    def calcolate_baricenter(self, robotVector):
        xG = 0
        yG = 0
        for naer in robotVector:
            xG += naer.getAbsolutePosX()
            yG += naer.getAbsolutePosY()
        xG = xG/self.numRobots
        yG = yG / self.numRobots
        return [xG, yG]

    def calc_matrixs_cost(self, robotVector):
        pass

    def reset_roles_connections(self, robotVector, indexes):
        i = 0
        for robot in robotVector:
            robot.setRole(indexes[i][1])
            i += 1

    def set_hash_role(self, robots_vector, self_role, target_role, distance):
        robots_vector[self_role].hashRole[target_role] = distance
        robots_vector[target_role].hashRole[self_role] = distance


class SquareFormationDistance(FormationDistance):
    def __init__(self, side):
        super().__init__(4)
        self.side = side

    def calc_matrixs_cost(self, robotVector):

        baricenter = self.calcolate_baricenter(robotVector)
        points = []
        punto0 = [baricenter[0] - self.side / 2, baricenter[1] - self.side / 2]
        punto1 = [baricenter[0] - self.side / 2, baricenter[1] + self.side / 2]
        punto2 = [baricenter[0] + self.side / 2, baricenter[1] + self.side / 2]
        punto3 = [baricenter[0] + self.side / 2, baricenter[1] - self.side / 2]
        points.append(punto0)
        points.append(punto1)
        points.append(punto2)
        points.append(punto3)

        return get_matrices_from_points(robotVector, points, baricenter)


class SquareFormationDistanceSingleIntegrator(SquareFormationDistance):
    def __init__(self, side):
        super().__init__(4)
        self.side = side

    def setFormationCostraint(self, robotVector):
        robotVector = sorted(robotVector, key=lambda item: item.role)

        robotVector[0].disconnect()
        robotVector[1].disconnect()
        robotVector[2].disconnect()
        robotVector[3].disconnect()
        Robot.connect(robotVector[0], robotVector[1])
        Robot.connect(robotVector[0], robotVector[2])
        Robot.connect(robotVector[0], robotVector[3])
        Robot.connect(robotVector[1], robotVector[2])
        Robot.connect(robotVector[1], robotVector[3])
        Robot.connect(robotVector[2], robotVector[3])

        robotVector[0].hashRole[1] = self.side
        robotVector[0].hashRole[2] = self.side*math.sqrt(2)
        robotVector[0].hashRole[3] = self.side
        robotVector[1].hashRole[0] = self.side
        robotVector[1].hashRole[2] = self.side
        robotVector[1].hashRole[3] = self.side*math.sqrt(2)
        robotVector[2].hashRole[0] = self.side*math.sqrt(2)
        robotVector[2].hashRole[1] = self.side
        robotVector[2].hashRole[3] = self.side
        robotVector[3].hashRole[0] = self.side
        robotVector[3].hashRole[1] = self.side*math.sqrt(2)
        robotVector[3].hashRole[2] = self.side

    def makeFormationRobot(self):
        return RobotDistanceSingleIntegrator.makeRandomRobot()

    @staticmethod
    def specifyFormationRobot(posX, posY):
        return RobotDistanceSingleIntegrator(posX, posY)


class SquareFormationDistanceDoubleIntegrator(SquareFormationDistance):

    def __init__(self, side, desVelX=0, desVelY=0):
        super().__init__(4)
        self.side = side
        self.desVelX = desVelX
        self.desVelY = desVelY

    def setFormationCostraint(self, robotVector):
        robotVector = sorted(robotVector, key=lambda item: item.role)

        robotVector[0].disconnect()
        robotVector[1].disconnect()
        robotVector[2].disconnect()
        robotVector[3].disconnect()
        Robot.connect(robotVector[0], robotVector[1])
        Robot.connect(robotVector[0], robotVector[2])
        Robot.connect(robotVector[0], robotVector[3])
        Robot.connect(robotVector[1], robotVector[2])
        Robot.connect(robotVector[1], robotVector[3])
        Robot.connect(robotVector[2], robotVector[3])

        robotVector[0].hashRole[1] = [[self.side], [self.desVelX, self.desVelY]]
        robotVector[0].hashRole[2] = [[self.side * math.sqrt(2)], [self.desVelX, self.desVelY]]
        robotVector[0].hashRole[3] = [[self.side], [self.desVelX, self.desVelY]]
        robotVector[1].hashRole[0] = [[self.side], [self.desVelX, self.desVelY]]
        robotVector[1].hashRole[2] = [[self.side], [self.desVelX, self.desVelY]]
        robotVector[1].hashRole[3] = [[self.side * math.sqrt(2)], [self.desVelX, self.desVelY]]
        robotVector[2].hashRole[0] = [[self.side * math.sqrt(2)], [self.desVelX, self.desVelY]]
        robotVector[2].hashRole[1] = [[self.side], [self.desVelX, self.desVelY]]
        robotVector[2].hashRole[3] = [[self.side], [self.desVelX, self.desVelY]]
        robotVector[3].hashRole[0] = [[self.side], [self.desVelX, self.desVelY]]
        robotVector[3].hashRole[1] = [[self.side * math.sqrt(2)], [self.desVelX, self.desVelY]]
        robotVector[3].hashRole[2] = [[self.side], [self.desVelX, self.desVelY]]

    def makeFormationRobot(self):
        return RobotDistanceDoubleIntegrator.makeRandomRobot()

    @staticmethod
    def specifyFormationRobot(posX, posY):
        return RobotDistanceDoubleIntegrator(posX, posY)


class SquareFormationDistanceUnicycle(SquareFormationDistanceSingleIntegrator):
    def makeFormationRobot(self):
        return RobotDistanceUnicycle.makeRandomRobot()

    @staticmethod
    def specifyFormationRobot(posX, posY):
        return RobotDistanceUnicycle(posX, posY)


class LinearFormationDistance(FormationDistance):
    def __init__(self, side, num_robots):
        super().__init__(num_robots)
        self.side = side

    def calc_matrixs_cost(self, robotVector):
        baricenter = self.calcolate_baricenter(robotVector)

        if len(robotVector) % 2 != 0:
            starting_point_x = baricenter[0] - math.floor(len(robotVector) / 2) * self.side
        else:
            starting_point_x = baricenter[0] - self.side / 2 - (len(robotVector) / 2) * self.side
        points = []

        for i in range(len(robotVector)):
            if i == 0:
                point = [starting_point_x, baricenter[1]]
            else:
                point = [points[i - 1][0] + self.side, points[i - 1][1]]
            points.append(point)
        return get_matrices_from_points(robotVector, points, baricenter)


class LinearHorizontalFormationDistanceSingleIntegrator(LinearFormationDistance):

    def setFormationCostraint(self, robotVector):
        robotVector = sorted(robotVector, key=lambda item: item.role)

        for i in range(len(robotVector)):
            robotVector[i].disconnect()
        for i in range(len(robotVector) - 1):
            Robot.connect(robotVector[i], robotVector[i + 1])
            if i + 2 < len(robotVector):
                Robot.connect(robotVector[i], robotVector[i + 2])
            if i + 3 < len(robotVector):
                Robot.connect(robotVector[i], robotVector[i + 3])

        for i in range(len(robotVector) - 1):
            self.set_hash_role(robotVector, i, i + 1, self.side)
            if i + 2 < len(robotVector):
                self.set_hash_role(robotVector, i, i + 2, self.side * 2)
            if i + 3 < len(robotVector):
                self.set_hash_role(robotVector, i, i + 3, self.side * 3)

    def makeFormationRobot(self):
        return RobotDistanceSingleIntegrator.makeRandomRobot()

    @staticmethod
    def specifyFormationRobot(posX, posY):
        return RobotDistanceSingleIntegrator(posX, posY)


class LinearHorizontalFormationDistanceDoubleIntegrator(LinearFormationDistance):
    def __init__(self, side, num_robots, desVelX=0, desVelY=0):
        super().__init__(side=side, num_robots=num_robots)
        self.desVelX = desVelX
        self.desVelY = desVelY

    def setFormationCostraint(self, robotVector):
        robotVector = sorted(robotVector, key=lambda item: item.role)

        for i in range(len(robotVector)):
            robotVector[i].disconnect()
        for i in range(len(robotVector) - 1):
            Robot.connect(robotVector[i], robotVector[i + 1])
            if i + 2 < len(robotVector):
                Robot.connect(robotVector[i], robotVector[i + 2])
            if i + 3 < len(robotVector):
                Robot.connect(robotVector[i], robotVector[i + 3])

        for i in range(len(robotVector) - 1):
            self.set_hash_role(robotVector, i, i + 1, [[self.side], [self.desVelX, self.desVelY]])
            if i + 2 < len(robotVector):
                self.set_hash_role(robotVector, i, i + 2, [[self.side * 2], [self.desVelX, self.desVelY]])
            if i + 3 < len(robotVector):
                self.set_hash_role(robotVector, i, i + 3, [[self.side * 3], [self.desVelX, self.desVelY]])

    def makeFormationRobot(self):
        return RobotDistanceDoubleIntegrator.makeRandomRobot()

    @staticmethod
    def specifyFormationRobot(posX, posY):
        return RobotDistanceDoubleIntegrator(posX, posY)


class LinearHorizontalFormationDistanceUnicycle(LinearHorizontalFormationDistanceSingleIntegrator):

    def makeFormationRobot(self):
        return RobotDistanceUnicycle.makeRandomRobot()

    @staticmethod
    def specifyFormationRobot(posX, posY):
        return RobotDistanceUnicycle(posX, posY)


class FreeFormationDistance(FormationDistance, ABC):
    def __init__(self, ptsFormation, lines):
        num_robots = len(ptsFormation)
        super().__init__(num_robots)
        self.hash_role = {}
        self.ptsFormation = ptsFormation  # salva i punti per la conversione tra tipi di free formation
        self.lines = lines  # salva le connessioni
        for i in range(num_robots):
            self.hash_role[i] = {}

    def calcolate_formation_baricenter(self):
        xG = 0
        yG = 0
        for i in self.ptsFormation:
            xG += i[1][0]
            yG += i[1][1]
        xG = xG / self.numRobots
        yG = yG / self.numRobots
        return [xG, yG]

    def calc_matrixs_cost(self, robotVector):
        robotBaricenter = self.calcolate_baricenter(robotVector)
        formationBaricenter = self.calcolate_formation_baricenter()
        difference = numpy.subtract(formationBaricenter, robotBaricenter)

        points = []
        for i in range(len(self.ptsFormation)):
            p_i = self.ptsFormation[i][1] - difference
            points.append(p_i)

        return get_matrices_from_points(robotVector, points, robotBaricenter)


class FreeFormationDistanceSingleIntegrator(FreeFormationDistance):

    def __init__(self, ptsFormation, lines):
        super().__init__(ptsFormation, lines)

    def setFormationCostraint(self, robot_vector):
        robot_vector = sorted(robot_vector, key=lambda item: item.role)

        for robot in robot_vector:
            robot.disconnect()

        for robot in robot_vector:
            for linea in self.lines:
                if linea[0] == robot.role:
                    punto1 = None
                    punto2 = None
                    for punto in self.ptsFormation:
                        if punto[0] == linea[0]:
                            punto1 = punto
                        elif punto[0] == linea[1]:
                            punto2 = punto
                    distance = distance_between_point(punto1[1], punto2[1])
                    robot.hashRole[linea[1]] = distance
                    robot2 = find_robot_by_role(robot_vector, linea[1])
                    robot2.hashRole[linea[0]] = distance  # setto i vincolo anche nel verso opposto
                    Robot.connect(robot, robot2)

    def makeFormationRobot(self):
        return RobotDistanceSingleIntegrator.makeRandomRobot()

    @staticmethod
    def specifyFormationRobot(posX, posY):
        return RobotDistanceSingleIntegrator(posX, posY)


class FreeFormationDistanceDoubleIntegrator(FreeFormationDistance):

    def __init__(self, ptsFormation, connectLineasFormations, desVelX=0, desVelY=0):
        super().__init__(ptsFormation, connectLineasFormations)
        self.desVelX = desVelX
        self.desVelY = desVelY

    def setFormationCostraint(self, robot_vector):
        robot_vector = sorted(robot_vector, key=lambda item: item.role)

        for robot in robot_vector:
            robot.disconnect()

        for robot in robot_vector:
            for linea in self.lines:
                if linea[0] == robot.role:
                    punto1 = None
                    punto2 = None
                    for punto in self.ptsFormation:
                        if punto[0] == linea[0]:
                            punto1 = punto
                        elif punto[0] == linea[1]:
                            punto2 = punto
                    distance = distance_between_point(punto1[1], punto2[1])
                    robot.hashRole[linea[1]] = [[distance], [self.desVelX, self.desVelY]]
                    robot2 = find_robot_by_role(robot_vector, linea[1])
                    robot2.hashRole[linea[0]] = [[distance], [self.desVelX, self.desVelY]]  # setto i vincolo anche nel verso opposto
                    Robot.connect(robot, robot2)

    def makeFormationRobot(self):
        return RobotDistanceDoubleIntegrator.makeRandomRobot()

    @staticmethod
    def specifyFormationRobot(posX, posY):
        return RobotDistanceDoubleIntegrator(posX, posY)


class FreeFormationDistanceUnicycle(FreeFormationDistanceSingleIntegrator):

    def makeFormationRobot(self):
        return RobotDistanceUnicycle.makeRandomRobot()

    @staticmethod
    def specifyFormationRobot( posX, posY):
        return RobotDistanceUnicycle(posX, posY)
