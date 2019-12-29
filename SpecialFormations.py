from Formation import *


class SquareFormationDisplacementSingleIntegrator6Edges(SquareFormationDisplacementSingleIntegrator):

    def setFormationCostraint(self, robotVector):
        robotVector = sorted(robotVector, key=lambda item: item.role)

        robotVector[0].disconnect()
        robotVector[1].disconnect()
        robotVector[2].disconnect()
        robotVector[3].disconnect()
        Robot.connect(robotVector[0], robotVector[1])
        Robot.connect(robotVector[1], robotVector[2])
        Robot.connect(robotVector[2], robotVector[3])
        Robot.connect(robotVector[3], robotVector[0])
        Robot.connect(robotVector[0], robotVector[2])
        Robot.connect(robotVector[1], robotVector[3])

        robotVector[0].hashRole[1] = [0, self.side]
        robotVector[1].hashRole[0] = [0, -self.side]
        robotVector[1].hashRole[2] = [self.side, 0]
        robotVector[2].hashRole[1] = [-self.side, 0]
        robotVector[2].hashRole[3] = [0, -self.side]
        robotVector[3].hashRole[2] = [0, self.side]
        robotVector[0].hashRole[3] = [self.side, 0]
        robotVector[3].hashRole[0] = [-self.side, 0]
        robotVector[0].hashRole[2] = [self.side, self.side]
        robotVector[2].hashRole[0] = [-self.side, -self.side]
        robotVector[1].hashRole[3] = [self.side, -self.side]
        robotVector[3].hashRole[1] = [-self.side, self.side]


class SquareFormationDisplacementDoubleIntegrator6Edges(SquareFormationDisplacementDoubleIntegrator):
    def setFormationCostraint(self, robotVector):
        robotVector = sorted(robotVector, key=lambda item: item.role)

        robotVector[0].disconnect()
        robotVector[1].disconnect()
        robotVector[2].disconnect()
        robotVector[3].disconnect()
        Robot.connect(robotVector[0], robotVector[1])
        Robot.connect(robotVector[1], robotVector[2])
        Robot.connect(robotVector[2], robotVector[3])
        Robot.connect(robotVector[3], robotVector[0])
        Robot.connect(robotVector[0], robotVector[2])
        Robot.connect(robotVector[1], robotVector[3])

        robotVector[0].hashRole[1] = [[0, self.side], [self.desVelX, self.desVelY]]
        robotVector[1].hashRole[0] = [[0, -self.side], [self.desVelX, self.desVelY]]
        robotVector[1].hashRole[2] = [[self.side, 0], [self.desVelX, self.desVelY]]
        robotVector[2].hashRole[1] = [[-self.side, 0], [self.desVelX, self.desVelY]]
        robotVector[2].hashRole[3] = [[0, -self.side], [self.desVelX, self.desVelY]]
        robotVector[3].hashRole[2] = [[0, self.side], [self.desVelX, self.desVelY]]
        robotVector[0].hashRole[3] = [[self.side, 0], [self.desVelX, self.desVelY]]
        robotVector[3].hashRole[0] = [[-self.side, 0], [self.desVelX, self.desVelY]]
        robotVector[0].hashRole[2] = [[self.side, self.side], [self.desVelX, self.desVelY]]
        robotVector[2].hashRole[0] = [[-self.side, -self.side], [self.desVelX, self.desVelY]]
        robotVector[1].hashRole[3] = [[self.side, -self.side], [self.desVelX, self.desVelY]]
        robotVector[3].hashRole[1] = [[-self.side, self.side], [self.desVelX, self.desVelY]]