import numpy as np

def automate_inputs(pose: tuple[tuple[int,int],str],measurement_position: tuple[int,int], grid_dim: tuple[int,int], final_theta = None):
    """
    Function to calculate movements to make to go from current position ((x1,y1),
    theta1) to next measurement position ((x2,y2)) using Manhattan Distance
    (move min x disp, then move min y disp).

    Movement Types:
        F - Move one grid forward
        B - Move one grid backward
        R - Turn Right 90 degrees
        L - Turn Left 90 degrees

    @args:
        pose ((x,y),theta) : typle[tuple[int,int],str]
        measurement_position (x,y) : tuple[int,int]
        grid_dim (n,m)
        final_theta : str

    @returns:
        list of movements{f/b/r/l}
        current theta
    """

    indexes,theta = pose
    x1,y1 = indexes
    x2,y2 = measurement_position

    x_disp = x2 - x1
    y_disp = y2 - y1

    movements = []

    current_theta = theta

    if theta == 'l':
        if x_disp > 0:
            for _ in range(x_disp):
                movements.append('b'*x_disp)
        elif x_disp < 0:
            for _ in range(abs(x_disp)):
                movements.append('f')

        if y_disp != 0:
            movements.append('r')
            current_theta = 'u'
            if y_disp > 0:
                for _ in range(y_disp):
                    movements.append('b')
            elif y_disp < 0:
                for _ in range(abs(y_disp)):
                    movements.append('f')

    elif theta == 'r':
        if x_disp > 0:
            for _ in range(x_disp):
                movements.append('f')
        elif x_disp < 0:
            for _ in range(abs(x_disp)):
                movements.append('b')

        if y_disp != 0:
            movements.append('r')
            current_theta = 'd'
            if y_disp > 0:
                for _ in range(y_disp):
                    movements.append('f')
            elif y_disp < 0:
                for _ in range(abs(y_disp)):
                    movements.append('b')

    elif theta == 'u':
        if y_disp > 0:
            for _ in range(y_disp):
                movements.append('b')
        elif y_disp < 0:
            for _ in range(abs(y_disp)):
                movements.append('f')

        if x_disp != 0:
            movements.append('r')
            current_theta = 'r'
            if x_disp > 0:
                for _ in range(x_disp):
                    movements.append('f')
            elif x_disp < 0:
                for _ in range(abs(x_disp)):
                    movements.append('b')

    elif theta == 'd':
        if y_disp > 0:
            for _ in range(y_disp):
                movements.append('f')
        elif y_disp < 0:
            for _ in range(abs(y_disp)):
                movements.append('b')

        if x_disp != 0:
            movements.append('r')
            current_theta = 'l'
            if x_disp > 0:
                for _ in range(x_disp):
                    movements.append('b')
            elif x_disp < 0:
                for _ in range(abs(x_disp)):
                    movements.append('f')

    if final_theta != None:
        if final_theta == current_theta:
            pass
        elif final_theta == 'L':
            if current_theta == 'U':
                movements.append('R')
            elif current_theta == 'R':
                for _ in range(2):
                    movements.append('R')
            elif current_theta == 'D':
                for _ in range(3):
                    movements.append('R')

        elif final_theta == 'R':
            if current_theta == 'D':
                movements.append('R')
            elif current_theta == 'L':
                for _ in range(2):
                    movements.append('R')
            elif current_theta == 'U':
                for _ in range(3):
                    movements.append('R')

        elif final_theta == 'U':
            if current_theta == 'R':
                movements.append('R')
            elif current_theta == 'D':
                for _ in range(2):
                    movements.append('R')
            elif current_theta == 'L':
                for _ in range(3):
                    movements.append('R')

        elif final_theta == 'D':
            if current_theta == 'L':
                movements.append('R')
            elif current_theta == 'U':
                for _ in range(2):
                    movements.append('R')
            elif current_theta == 'R':
                for _ in range(3):
                    movements.append('R')
        
    return movements,current_theta

def test_automation():
    # Test 1
    pose = ((0, 0), "u")
    measurement_position = (2, 3)
    grid_dim = (5, 5)
    movements,curr_theta = automate_inputs(pose, measurement_position, grid_dim)
    print(f"Test 1:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n{curr_theta=}\n\n")

    # Test 2
    pose = ((3, 4), "l")
    measurement_position = (1, 1)
    grid_dim = (5, 5)
    movements,curr_theta = automate_inputs(pose, measurement_position, grid_dim)
    print(f"Test 2:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n{curr_theta=}\n\n")

    # Test 3
    pose = ((2, 2), "r")
    measurement_position = (0, 0)
    grid_dim = (5, 5)
    movements,curr_theta = automate_inputs(pose, measurement_position, grid_dim)
    print(f"Test 3:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n{curr_theta=}\n\n")

    # Test Amog
    pose = ((0, 0), "u")
    measurement_position = (3, 3)
    grid_dim = (4, 4)
    movements,curr_theta = automate_inputs(pose, measurement_position, grid_dim)
    print(f"Test Amog:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n{curr_theta=}\n\n")

    # Test Simulation #
    pose = ((0, 0), "u")
    measurement_position = (2, 0)
    grid_dim = (3, 3)
    movements,curr_theta = automate_inputs(pose, measurement_position, grid_dim)
    print(f"Movement 1:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n{curr_theta=}\n\n")

    pose = ((2, 0), "u")
    measurement_position = (1, 0)
    movements,curr_theta = automate_inputs(pose, measurement_position, grid_dim)
    print(f"Movement 2:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n{curr_theta=}\n\n")

    pose = ((1, 0), "u")
    measurement_position = (0, 0)
    movements,curr_theta = automate_inputs(pose, measurement_position, grid_dim)
    print(f"Movement 3:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n{curr_theta=}\n\n")

    pose = ((0, 0), "r")
    measurement_position = (0, 1)
    movements,curr_theta = automate_inputs(pose, measurement_position, grid_dim)
    print(f"Movement 4:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n{curr_theta=}\n\n")

    pose = ((0, 1), "d")
    measurement_position = (1, 1)
    movements,curr_theta = automate_inputs(pose, measurement_position, grid_dim)
    print(f"Movement 5:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n{curr_theta=}\n\n")

    pose = ((1, 1), "d")
    measurement_position = (1, 1)
    movements,curr_theta = automate_inputs(pose, measurement_position, grid_dim)
    print(f"Movement 6:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n{curr_theta=}\n\n")

    pose = ((1, 1), "r")
    measurement_position = (1, 2)
    movements,curr_theta = automate_inputs(pose, measurement_position, grid_dim)
    print(f"Movement 7:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n{curr_theta=}\n\n")

    pose = ((1, 2), "d")
    measurement_position = (1, 2)
    movements,curr_theta = automate_inputs(pose, measurement_position, grid_dim)
    print(f"Movement 8:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n{curr_theta=}\n\n")

    pose = ((1, 2), "u")
    measurement_position = (1, 2)
    movements,curr_theta = automate_inputs(pose, measurement_position, grid_dim)
    print(f"Movement 9:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n{curr_theta=}\n\n")

    pose = ((1, 2), "l")
    measurement_position = (1, 1)
    movements,curr_theta = automate_inputs(pose, measurement_position, grid_dim)
    print(f"Movement 10:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n{curr_theta=}\n\n")

if __name__ == "__main__":
    test_automation()
