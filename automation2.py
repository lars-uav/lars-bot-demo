def automate_inputs(pose: tuple[tuple[int, int], str], measurement_position: tuple[int, int], grid_dim: tuple[int, int]):
    """
    Function to calculate movements to go from current position ((x1, y1), theta1)
    to the next measurement position ((x2, y2)) using Manhattan Distance.
    The movements only use 'f', 'b', and 'r', with no use of 'l'.

    Movement Types:
        f - Move one grid forward
        b - Move one grid backward
        r - Turn Right 90 degrees (used to replace 'l' by using three 'r' turns)

    @args:
        pose ((x, y), theta): tuple[tuple[int, int], str]
        measurement_position (x, y): tuple[int, int]
        grid_dim (n, m): Tuple representing the grid dimensions

    @returns:
        list of movements: List of characters ('f', 'b', 'r') indicating movements
        current theta: Updated orientation after movements
    """

    def matrix_index_to_cartesian(indexes, grid_dim):
        """Convert matrix indexes to Cartesian coordinates (row, col) to (x, y)."""
        return indexes[1], grid_dim[0] - 1 - indexes[0]

    indexes, theta1 = pose
    x1, y1 = matrix_index_to_cartesian(indexes, grid_dim)
    x2, y2 = matrix_index_to_cartesian(measurement_position, grid_dim)
    theta1 = theta1.lower()

    # Calculate x and y displacements
    x_disp = x2 - x1
    y_disp = y2 - y1

    movements = []
    curr_theta = theta1

    def turn_right(times):
        """Helper function to add 'r' movements."""
        for _ in range(times):
            movements.append("r")

    # Make y-movements first, then x-movements based on the current orientation
    if curr_theta == "u":
        if y_disp > 0:
            movements.extend("f" * y_disp)
        elif y_disp < 0:
            movements.extend("b" * abs(y_disp))

        if x_disp > 0:
            turn_right(1)
            movements.extend("f" * x_disp)
            curr_theta = "r"
        elif x_disp < 0:
            turn_right(3)
            movements.extend("f" * abs(x_disp))
            curr_theta = "l"

    elif curr_theta == "d":
        if y_disp < 0:
            movements.extend("f" * abs(y_disp))
        elif y_disp > 0:
            movements.extend("b" * y_disp)

        if x_disp < 0:
            turn_right(1)
            movements.extend("f" * abs(x_disp))
            curr_theta = "l"
        elif x_disp > 0:
            turn_right(3)
            movements.extend("f" * x_disp)
            curr_theta = "r"

    elif curr_theta == "r":
        if x_disp > 0:
            movements.extend("f" * x_disp)
        elif x_disp < 0:
            movements.extend("b" * abs(x_disp))

        if y_disp < 0:
            turn_right(1)
            movements.extend("f" * abs(y_disp))
            curr_theta = "d"
        elif y_disp > 0:
            turn_right(3)
            movements.extend("f" * y_disp)
            curr_theta = "u"

    elif curr_theta == "l":
        if x_disp < 0:
            movements.extend("f" * abs(x_disp))
        elif x_disp > 0:
            movements.extend("b" * x_disp)

        if y_disp > 0:
            turn_right(1)
            movements.extend("f" * y_disp)
            curr_theta = "u"
        elif y_disp < 0:
            turn_right(3)
            movements.extend("f" * abs(y_disp))
            curr_theta = "d"

    return movements, curr_theta

