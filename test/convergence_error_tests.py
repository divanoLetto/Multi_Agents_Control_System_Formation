from myGlobalEnviroment3D import *
from myGlobalEnviroment import *
from Formation import *
from Formation3D import *
from SpecialFormations import *


def make_robots_from_points_test(num_robots, formation):
    if isinstance(formation, Formation3D):
        bool_3d = True
    else:
        bool_3d = False
    pts = []
    if not bool_3d:
        for i in range(num_robots):
            randx = random.randint(0, 50)
            randy = random.randint(0, 50)
            pts.append([randx, randy])
    else:
        for i in range(num_robots):
            randx = random.randint(0, 50)
            randy = random.randint(0, 50)
            randz = random.randint(0, 50)
            pts.append([randx, randy, randz])

    rand = random.randint(0, num_robots - 1)  # scegli il commander
    robots_vector = []
    i=0
    if not bool_3d:
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


def make_robots_for_stuck_in_equilibrium(robots_vector):
    robots_vector[0].setRole(0)
    robots_vector[1].setRole(1)
    robots_vector[2].setRole(2)
    robots_vector[3].setRole(3)
    robots_vector[0].disconnect()
    robots_vector[1].disconnect()
    robots_vector[2].disconnect()
    robots_vector[3].disconnect()
    Robot.connect(robots_vector[0], robots_vector[1])
    Robot.connect(robots_vector[0], robots_vector[2])
    Robot.connect(robots_vector[0], robots_vector[3])
    Robot.connect(robots_vector[1], robots_vector[2])
    Robot.connect(robots_vector[1], robots_vector[3])
    Robot.connect(robots_vector[2], robots_vector[3])
    robots_vector[0].hashRole[1] = [[12], [0, 0, 0]]
    robots_vector[0].hashRole[2] = [[math.sqrt(65) + math.sqrt(145)], [0, 0, 0]]
    robots_vector[0].hashRole[3] = [[12], [0, 0, 0]]
    robots_vector[1].hashRole[0] = [[12], [0, 0, 0]]
    robots_vector[1].hashRole[2] = [[8], [0, 0, 0]]
    robots_vector[1].hashRole[3] = [[1], [0, 0, 0]]
    robots_vector[2].hashRole[0] = [[math.sqrt(65) + math.sqrt(145)], [0, 0, 0]]
    robots_vector[2].hashRole[1] = [[8], [0, 0, 0]]
    robots_vector[2].hashRole[3] = [[8], [0, 0, 0]]
    robots_vector[3].hashRole[0] = [[12], [0, 0, 0]]
    robots_vector[3].hashRole[1] = [[1], [0, 0, 0]]
    robots_vector[3].hashRole[2] = [[8], [0, 0, 0]]
    robots_vector[1].startX = 20
    robots_vector[1].startY = 20.5
    robots_vector[2].startX = 20 - math.sqrt(65)
    robots_vector[2].startY = 20
    robots_vector[3].startX = 20
    robots_vector[3].startY = 19.5
    robots_vector[0].startX = -10
    robots_vector[0].startY = 20

    return robots_vector


def main_test():
    print("start test function")
    num_exp = 100
    total_error_list = []
    error_convergence_list = []

    for i in range(num_exp):

        error_list = []
        current_time = 0
        max_time = 5.0
        step_time = 0.01
        num_robots = 4
        formation = SquareFormationDisplacementSingleIntegrator6Edges(4)

        robots_vector = make_robots_from_points_test(num_robots=num_robots, formation=formation)

        myGlobalEnviroment(robots_vector)  # crea un istanza di spazio globale
        myGlobalEnviroment3D(robots_vector)  # crea un istanza di spazio globale 3D
        commander = findCommander(robots_vector)
        commander.makeSpanningTree()
        commander.makeCostraint()

        # robots_vector = make_robots_for_stuck_in_equilibrium(robots_vector)
        robots_vector = sorted(robots_vector, key=lambda item: item.role)

        print("calcolating...")
        while current_time < max_time:
            current_time += step_time
            updateRobots(robots_vector, step_time)
            error = 0
            for robot in robots_vector:
                error += robot.get_error()**2
            error = math.sqrt(error)
            error_list.append(error)

        total_error_list.append(error_list)

    print("finished test simulation")
    time_list = np.arange(0.0, max_time, step_time)
    for i in range(len(time_list)):
        convergence_error = 0
        for j in range(num_exp):
            convergence_error += (total_error_list[j][i])**2
        convergence_error = math.sqrt(convergence_error / num_exp)
        error_convergence_list.append(convergence_error)
    plt.plot(time_list, error_convergence_list, label="displacement single integrator")
    plt.xlabel('error(e)')
    plt.ylabel('time(s)')

    num_exp = 100
    total_error_list = []
    error_convergence_list = []

    for i in range(num_exp):

        error_list = []
        current_time = 0
        max_time = 5.0
        step_time = 0.01
        num_robots = 4
        formation = SquareFormationDistanceSingleIntegrator(4)

        robots_vector = make_robots_from_points_test(num_robots=num_robots, formation=formation)

        myGlobalEnviroment(robots_vector)  # crea un istanza di spazio globale
        myGlobalEnviroment3D(robots_vector)  # crea un istanza di spazio globale 3D
        commander = findCommander(robots_vector)
        commander.makeSpanningTree()
        commander.makeCostraint()

        # robots_vector = make_robots_for_stuck_in_equilibrium(robots_vector)
        robots_vector = sorted(robots_vector, key=lambda item: item.role)

        print("calcolating...")
        while current_time < max_time:
            current_time += step_time
            updateRobots(robots_vector, step_time)
            error = 0
            for robot in robots_vector:
                error += robot.get_error() ** 2
            error = math.sqrt(error)
            error_list.append(error)

        total_error_list.append(error_list)

    print("finished test simulation")
    time_list = np.arange(0.0, max_time, step_time)
    for i in range(len(time_list)):
        convergence_error = 0
        for j in range(num_exp):
            convergence_error += (total_error_list[j][i]) ** 2
        convergence_error = math.sqrt(convergence_error / num_exp)
        error_convergence_list.append(convergence_error)
    plt.plot(time_list, error_convergence_list, label="distance single integrator")
    plt.xlabel('error(e)')
    plt.ylabel('time(s)')

    plt.legend()
    plt.show()
    print("finished test")


main_test()