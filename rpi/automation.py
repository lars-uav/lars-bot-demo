import numpy as np

# a change

def cartesian_to_matrix_index(coord, grid_dim):
    """
    Function to convert Cartesian coordinates to matrix indexes.
    """
    x, y = coord
    n, m = grid_dim
    return y - (m-1), x

def matrix_index_to_cartesian(coord, grid_dim):
    '''
    Function to covert Matrix Coordinates to Cartesian Coordinates
    '''
    y, x = coord
    n, m = grid_dim
    return x,(m-1) - y


def automate_inputs(pose: tuple[tuple[int, int],str], measurement_position: tuple[int, int], grid_dim: tuple[int, int]):
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

    @returns:
        list of movements{f/b/r/l}
        current theta
    """

    # Convert Cartesian coordinates to matrix indexes
    indexes, theta1 = pose
    x1, y1 = matrix_index_to_cartesian(indexes, grid_dim)
    indexes = measurement_position 
    x2, y2 = matrix_index_to_cartesian(indexes, grid_dim)
    theta1 = theta1.lower()
    # Calculate x and y displacements
    x_disp = x2 - x1
    y_disp = y2 - y1

    movements = []

    curr_theta = theta1
    # Face Up, have to move, up, right, left, down #
    if theta1 == "u":
        # First Making Y-Movements #
        if y_disp > 0:
            movements.extend("f" * y_disp)
        elif y_disp < 0:
            movements.extend("b" * abs(y_disp))

        # Making X-Movements #
        if x_disp > 0:
            movements.extend("r")
            movements.extend("f" * x_disp)
            curr_theta = "r"
        elif x_disp < 0:
            movements.extend("r")
            movements.extend("r")
            movements.extend("r")
            movements.extend("f" * abs(x_disp))
            curr_theta = "l"

    elif theta1 == "d":
        # First Making Y-Movements #
        if y_disp < 0:
            movements.extend("f" * abs(y_disp))
        elif y_disp > 0:
            movements.extend("b" * y_disp)

        # Making X-Movements #
        if x_disp < 0:
            movements.extend("r")
            movements.extend("f" * abs(x_disp))
            curr_theta = "l"
        elif x_disp > 0:
            movements.extend("r")
            movements.extend("r")
            movements.extend("r")
            movements.extend("f" * x_disp)
            curr_theta = "r"

    elif theta1 == "r":
        # First Making X-Movements #
        if x_disp > 0:
            movements.extend("f" * x_disp)
        elif x_disp < 0:
            movements.extend("b" * abs(x_disp))

        # Making Y-Movements #
        if y_disp < 0:
            movements.extend("r")
            movements.extend("f" * abs(y_disp))
            curr_theta = "d"
        elif y_disp > 0:
            movements.extend("r")
            movements.extend("r")
            movements.extend("r")
            movements.extend("f" * y_disp)
            curr_theta = "u"

    elif theta1 == "l":
        # First Making X-Movements #
        if x_disp < 0:
            movements.extend("f" * abs(x_disp))
        elif x_disp > 0:
            movements.extend("b" * x_disp)

        # Making Y-Movements #
        if y_disp > 0:
            movements.extend("r")
            movements.extend("f" * y_disp)
            curr_theta = "u"
        elif y_disp < 0:
            movements.extend("r")
            movements.extend("r")
            movements.extend("r")
            movements.extend("f" * abs(y_disp))
            curr_theta = "d"

    return movements,curr_theta


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
