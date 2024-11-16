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

    if theta == 'L':
        if x_disp > 0:
            movements.append('B'*x_disp)
        elif x_disp < 0:
            movements.append('F'*abs(x_disp))

        if y_disp != 0:
            movements.append('R')
            current_theta = 'U'
            if y_disp > 0:
                movements.append('F'*y_disp)
            elif y_disp < 0:
                movements.append('B'*abs(y_disp))

    elif theta == 'R':
        if x_disp > 0:
            movements.append('F'*x_disp)
        elif x_disp < 0:
            movements.append('B'*abs(x_disp))

        if y_disp != 0:
            movements.append('R')
            current_theta = 'D'
            if y_disp > 0:
                movements.append('B'*y_disp)
            elif y_disp < 0:
                movements.append('F'*abs(y_disp))

    elif theta == 'U':
        if y_disp > 0:
            movements.append('F'*x_disp)
        elif y_disp < 0:
            movements.append('B'*abs(x_disp))

        if x_disp != 0:
            movements.append('R')
            current_theta = 'R'
            if x_disp > 0:
                movements.append('F'*y_disp)
            elif x_disp < 0:
                movements.append('B'*abs(y_disp))

    elif theta == 'D':
        if y_disp > 0:
            movements.append('B'*x_disp)
        elif y_disp < 0:
            movements.append('F'*abs(x_disp))

        if x_disp != 0:
            movements.append('R')
            current_theta = 'L'
            if x_disp > 0:
                movements.append('B'*y_disp)
            elif x_disp < 0:
                movements.append('F'*abs(y_disp))

    if final_theta != None:
        if final_theta == curr_theta:
            pass
        elif final_theta == 'L':
            if theta == 'U':
                movements.append('R')
            elif theta == 'R':
                movements.append('R'*2)
            elif theta == 'D':
                movements.append('R'*3)

        elif final_theta == 'R':
            if theta == 'D':
                movements.append('R')
            elif theta == 'L':
                movements.append('R'*2)
            elif theta == 'U':
                movements.append('R'*3)

        elif final_theta == 'U':
            if theta == 'R':
                movements.append('R')
            elif theta == 'D':
                movements.append('R'*2)
            elif theta == 'L':
                movements.append('R'*3)

        elif final_theta == 'D':
            if theta == 'L':
                movements.append('R')
            elif theta == 'U':
                movements.append('R'*2)
            elif theta == 'R':
                movements.append('R'*3)
        
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
    movements = automate_inputs(pose, measurement_position, grid_dim)
    print(f"Test 2:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n{curr_theta=}\n\n")

    # Test 3
    pose = ((2, 2), "r")
    measurement_position = (0, 0)
    grid_dim = (5, 5)
    movements = automate_inputs(pose, measurement_position, grid_dim)
    print(f"Test 3:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n{curr_theta=}\n\n")

    # Test Amog
    pose = ((0, 0), "u")
    measurement_position = (3, 3)
    grid_dim = (4, 4)
    movements = automate_inputs(pose, measurement_position, grid_dim)
    print(f"Test Amog:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n\n")

    # Test Simulation #
    pose = ((0, 0), "u")
    measurement_position = (2, 0)
    grid_dim = (3, 3)
    movements = automate_inputs(pose, measurement_position, grid_dim)
    print(f"Movement 1:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n\n")

    pose = ((2, 0), "u")
    measurement_position = (1, 0)
    movements = automate_inputs(pose, measurement_position, grid_dim)
    print(f"Movement 2:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n\n")
    print(f"Test 2:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n\n")

    pose = ((1, 0), "u")
    measurement_position = (0, 0)
    movements = automate_inputs(pose, measurement_position, grid_dim)
    print(f"Movement 3:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n\n")
    print(f"Test 2:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n\n")

    pose = ((0, 0), "r")
    measurement_position = (0, 1)
    movements = automate_inputs(pose, measurement_position, grid_dim)
    print(f"Movement 4:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n\n")
    print(f"Test 2:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n\n")

    pose = ((0, 1), "d")
    measurement_position = (1, 1)
    movements = automate_inputs(pose, measurement_position, grid_dim)
    print(f"Movement 5:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n\n")
    print(f"Test 2:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n\n")

    pose = ((1, 1), "d")
    measurement_position = (1, 1)
    movements = automate_inputs(pose, measurement_position, grid_dim)
    print(f"Movement 6:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n\n")
    print(f"Test 2:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n\n")

    pose = ((1, 1), "r")
    measurement_position = (1, 2)
    movements = automate_inputs(pose, measurement_position, grid_dim)
    print(f"Movement 7:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n\n")
    print(f"Test 2:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n\n")

    pose = ((1, 2), "d")
    measurement_position = (1, 2)
    movements = automate_inputs(pose, measurement_position, grid_dim)
    print(f"Movement 8:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n\n")
    print(f"Test 2:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n\n")

    pose = ((1, 2), "u")
    measurement_position = (1, 2)
    movements = automate_inputs(pose, measurement_position, grid_dim)
    print(f"Movement 9:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n\n")
    print(f"Test 2:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n\n")

    pose = ((1, 2), "l")
    measurement_position = (1, 1)
    movements = automate_inputs(pose, measurement_position, grid_dim)
    print(f"Movement 10:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n\n")
    print(f"Test 2:\n{pose=}\t{measurement_position=}\t{grid_dim=}\n{movements=}\n\n")

if __name__ == "__main__":
    test_automation()
