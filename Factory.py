from Formation import *
from FormationNote import *
from Formation3D import *

class Factory:
    @staticmethod
    def factoryFormation(inputString, num_robots, side=0, velX=0, velY=0, free_formation_saves=None, last_free_formation_name=None ):
        # Displacement

        if inputString == SquareFormationDisplacementSingleIntegrator:
            return SquareFormationDisplacementSingleIntegrator(side)
        elif inputString == SquareFormationDisplacementDoubleIntegrator:
            return SquareFormationDisplacementDoubleIntegrator(side, desVelX=velX, desVelY=velY)
        elif inputString == SquareFormationDisplacementUnicycle:
            return SquareFormationDisplacementUnicycle(side)

        elif inputString == LinearHorizontalFormationDisplacementSingleIntegrator:
            return LinearHorizontalFormationDisplacementSingleIntegrator(side, num_robots)
        elif inputString == LinearHorizontalFormationDisplacementDoubleIntegrator:
            return LinearHorizontalFormationDisplacementDoubleIntegrator(side, num_robots, desVelX=velX, desVelY=velY)
        elif inputString == LinearHorizontalFormationDisplacementUnicycle:
            return LinearHorizontalFormationDisplacementUnicycle(side, num_robots)

        elif inputString == FreeFormationDisplacementSingleIntegrator:
            points = free_formation_saves[last_free_formation_name]["points"]  # free formation parameter
            lines = free_formation_saves[last_free_formation_name]["lines"]  # free formation parameter
            return FreeFormationDisplacementSingleIntegrator(points, lines)
        elif inputString == FreeFormationDisplacementDoubleIntegrator:
            points = free_formation_saves[last_free_formation_name]["points"]  # free formation parameter
            lines = free_formation_saves[last_free_formation_name]["lines"]  # free formation parameter
            return FreeFormationDisplacementDoubleIntegrator(points, lines, desVelX=velX, desVelY=velY)
        elif inputString == FreeFormationDisplacementUnicycle:
            points = free_formation_saves[last_free_formation_name]["points"]  # free formation parameter
            lines = free_formation_saves[last_free_formation_name]["lines"]  # free formation parameter
            return FreeFormationDisplacementUnicycle(points, lines)
        # Distance
        elif inputString == SquareFormationDistanceSingleIntegrator:
            return SquareFormationDistanceSingleIntegrator(side)
        elif inputString == SquareFormationDistanceDoubleIntegrator:
            return SquareFormationDistanceDoubleIntegrator(side)
        elif inputString == SquareFormationDistanceUnicycle:
            return SquareFormationDistanceUnicycle(side)

        elif inputString == LinearHorizontalFormationDistanceSingleIntegrator:
            return LinearHorizontalFormationDistanceSingleIntegrator(side, num_robots)
        elif inputString == LinearHorizontalFormationDistanceDoubleIntegrator:
            return LinearHorizontalFormationDistanceDoubleIntegrator(side, num_robots)
        elif inputString == LinearHorizontalFormationDistanceUnicycle:
            return LinearHorizontalFormationDistanceUnicycle(side, num_robots)

        elif inputString == FreeFormationDistanceSingleIntegrator:
            points = free_formation_saves[last_free_formation_name]["points"]  # free formation parameter
            lines = free_formation_saves[last_free_formation_name]["lines"]  # free formation parameter
            return FreeFormationDistanceSingleIntegrator(points, lines)
        elif inputString == FreeFormationDistanceDoubleIntegrator:
            points = free_formation_saves[last_free_formation_name]["points"]  # free formation parameter
            lines = free_formation_saves[last_free_formation_name]["lines"]  # free formation parameter
            return FreeFormationDistanceDoubleIntegrator(points, lines)
        elif inputString == FreeFormationDistanceUnicycle:
            points = free_formation_saves[last_free_formation_name]["points"]  # free formation parameter
            lines = free_formation_saves[last_free_formation_name]["lines"]  # free formation parameter
            return FreeFormationDistanceUnicycle(points, lines)

        else:
            raise NotImplementedError

    @staticmethod
    def factoryFormation3D(inputString, num_robots, side=0, velX=0, velY=0, velZ=0,free_formation_saves=None, last_free_formation_name=None ):
        # Displacement

        if inputString == SquareFormationDisplacementSingleIntegrator:
            return SquareFormationDisplacementSingleIntegrator3D(side)
        elif inputString == SquareFormationDisplacementDoubleIntegrator:
            return SquareFormationDisplacementDoubleIntegrator3D(side, desVelX=velX, desVelY=velY, desVelZ=velZ)

        elif inputString == LinearHorizontalFormationDisplacementSingleIntegrator:
            return LinearHorizontalFormationDisplacementSingleIntegrator3D(side, num_robots)
        elif inputString == LinearHorizontalFormationDisplacementDoubleIntegrator:
            return LinearHorizontalFormationDisplacementDoubleIntegrator3D(side, num_robots, desVelX=velX, desVelY=velY, desVelZ=velZ)

        elif inputString == FreeFormationDisplacementSingleIntegrator:
            points = free_formation_saves[last_free_formation_name]["points"]  # free formation parameter
            lines = free_formation_saves[last_free_formation_name]["lines"]  # free formation parameter
            return FreeFormationDisplacementSingleIntegrator3D(points, lines)
        elif inputString == FreeFormationDisplacementDoubleIntegrator:
            points = free_formation_saves[last_free_formation_name]["points"]  # free formation parameter
            lines = free_formation_saves[last_free_formation_name]["lines"]  # free formation parameter
            return FreeFormationDisplacementDoubleIntegrator3D(points, lines, desVelX=velX, desVelY=velY, desVelZ=velZ)

        elif inputString == CubeFormationDisplacementSingleIntegrator3D:
            return CubeFormationDisplacementSingleIntegrator3D(side)
        elif inputString == CubeFormationDisplacementDoubleIntegrator3D:
            return CubeFormationDisplacementDoubleIntegrator3D(side)
        # Distance
        elif inputString == SquareFormationDistanceSingleIntegrator:
            return SquareFormationDistanceSingleIntegrator3D(side)
        elif inputString == SquareFormationDistanceDoubleIntegrator:
            return SquareFormationDistanceDoubleIntegrator3D(side)

        elif inputString == LinearHorizontalFormationDistanceSingleIntegrator:
            return LinearHorizontalFormationDistanceSingleIntegrator3D(side, num_robots)
        elif inputString == LinearHorizontalFormationDistanceDoubleIntegrator:
            return LinearHorizontalFormationDistanceDoubleIntegrator3D(side, num_robots)

        elif inputString == CubeFormationDistanceSingleIntegrator3D:
            return CubeFormationDistanceSingleIntegrator3D(side)
        elif inputString == CubeFormationDistanceDoubleIntegrator3D:
            return CubeFormationDistanceDoubleIntegrator3D(side)

        elif inputString == FreeFormationDistanceSingleIntegrator:
            points = free_formation_saves[last_free_formation_name]["points"]  # free formation parameter
            lines = free_formation_saves[last_free_formation_name]["lines"]  # free formation parameter
            return FreeFormationDistanceSingleIntegrator3D(points, lines)
        elif inputString == FreeFormationDistanceDoubleIntegrator:
            points = free_formation_saves[last_free_formation_name]["points"]  # free formation parameter
            lines = free_formation_saves[last_free_formation_name]["lines"]  # free formation parameter
            return FreeFormationDistanceDoubleIntegrator3D(points, lines)


        else:
            raise NotImplementedError


def make_formation_from_robot_and_note(formation_note, robot_type, lato, num_robots, free_formation_saves=None, last_free_formation_name=None, vel_x=0, vel_y=0):

    formation_type = make_formation_type(formation_note, robot_type)
    formation = Factory.factoryFormation(inputString=formation_type, side=lato, num_robots=num_robots,
                                         free_formation_saves=free_formation_saves,
                                         last_free_formation_name=last_free_formation_name, velX=vel_x, velY=vel_y)
    return formation


def make_formation_from_robot_and_note_3D(formation_note, robot_type, lato, num_robots, free_formation_saves=None, last_free_formation_name=None, vel_x=0, vel_y=0, vel_z=0):

    formation_type = make_formation_type(formation_note, robot_type)
    formation = Factory.factoryFormation3D(inputString=formation_type, side=lato, num_robots=num_robots,
                                         free_formation_saves=free_formation_saves,
                                         last_free_formation_name=last_free_formation_name, velX=vel_x, velY=vel_y, velZ=vel_z)
    return formation


def make_formation_type(formation_note, robot_type):
    formation_type = None
    if formation_note == LinearNote:
        formation_type = make_formation_from_robot_linear(robot_type)
    elif formation_note == SquareNote:
        formation_type = make_formation_from_robot_square(robot_type)
    elif formation_note == FreeNote:
        formation_type = make_formation_from_robot_free(robot_type)
    elif formation_note == CubeNote:
        formation_type = make_formation_from_robot_cube(robot_type)
    return formation_type


def make_formation_from_robot_linear(robot_type):
    if robot_type == RobotDisplacementSingleIntegrator:
        return LinearHorizontalFormationDisplacementSingleIntegrator
    elif robot_type == RobotDisplacementDoubleIntegrator:
        return LinearHorizontalFormationDisplacementDoubleIntegrator
    elif robot_type == RobotDisplacementUnicycle:
        return LinearHorizontalFormationDisplacementUnicycle
    elif robot_type == RobotDistanceSingleIntegrator:
        return LinearHorizontalFormationDistanceSingleIntegrator
    elif robot_type == RobotDistanceDoubleIntegrator:
        return LinearHorizontalFormationDistanceDoubleIntegrator
    elif robot_type == RobotDistanceUnicycle:
        return LinearHorizontalFormationDistanceUnicycle


def make_formation_from_robot_square(robot_type):
    if robot_type == RobotDisplacementSingleIntegrator:
        a = SquareFormationDisplacementSingleIntegrator(4)
        return SquareFormationDisplacementSingleIntegrator
    elif robot_type == RobotDisplacementDoubleIntegrator:
        return SquareFormationDisplacementDoubleIntegrator
    elif robot_type == RobotDisplacementUnicycle:
        return SquareFormationDisplacementUnicycle
    elif robot_type == RobotDistanceSingleIntegrator:
        return SquareFormationDistanceSingleIntegrator
    elif robot_type == RobotDistanceDoubleIntegrator:
        return SquareFormationDistanceDoubleIntegrator
    elif robot_type == RobotDistanceUnicycle:
        return SquareFormationDistanceUnicycle


def make_formation_from_robot_free(robot_type):
    if robot_type == RobotDisplacementSingleIntegrator:
        return FreeFormationDisplacementSingleIntegrator
    elif robot_type == RobotDisplacementDoubleIntegrator:
        return FreeFormationDisplacementDoubleIntegrator
    elif robot_type == RobotDisplacementUnicycle:
        return FreeFormationDisplacementUnicycle
    elif robot_type == RobotDistanceSingleIntegrator:
        return FreeFormationDistanceSingleIntegrator
    elif robot_type == RobotDistanceDoubleIntegrator:
        return FreeFormationDistanceDoubleIntegrator
    elif robot_type == RobotDistanceUnicycle:
        return FreeFormationDistanceUnicycle

def make_formation_from_robot_cube(robot_type):
    if robot_type == RobotDisplacementSingleIntegrator:
        return CubeFormationDisplacementSingleIntegrator3D
    elif robot_type == RobotDisplacementDoubleIntegrator:
        return CubeFormationDisplacementDoubleIntegrator3D
    elif robot_type == RobotDistanceSingleIntegrator:
        return CubeFormationDistanceSingleIntegrator3D
    elif robot_type == RobotDistanceDoubleIntegrator:
        return CubeFormationDistanceDoubleIntegrator3D