class SlidingPuzzleSolver:
    def __init__(self, initial_puzzle):
        self.goal = [[1, 2, 3], [4, 5, 6], [7, 8, 0]]
        self.initial_puzzle = initial_puzzle
        self.visited_states = set()
        self.search_path = []
    def blank_space(self, puzzle):
        for i, row in enumerate(puzzle):
            for j, val in enumerate(row):
                if val == 0:
                    return i, j
    def isGoal(self, current_state):
        return current_state == self.goal

    def swap(self, puzzle, x1, y1, x2, y2):
        temp = puzzle[x1][y1]
        puzzle[x1][y1] = puzzle[x2][y2]
        puzzle[x2][y2] = temp

    # RECURSIVE BACKTRACKING
    def solvePuzzle(self, current_state, path=[]):
        # Defining the directions
        u = (-1, 0)
        d = (1, 0)
        l = (0, -1)
        r = (0, 1)
        directions = [u, d, l, r]

        # Base case
        if self.isGoal(current_state):
            self.search_path = path
            return True

        # Recursive Case
        # We need to convert the current state into a tuple of tuples so that it can be added to the visited_state sets.
        state_key = tuple(tuple(row) for row in current_state)
        if state_key in self.visited_states:
            return False
        # Adds the state_key into the visited states
        self.visited_states.add(state_key)

        # Get the coordinates of the blank space
        x, y = self.blank_space(current_state)

        # Manipulating Puzzle
        for dx, dy in directions:
            # move the blank space
            nx, ny = x + dx, y + dy
            # Checking if the new position is within the bounds
            if 0 <= nx < 3 and 0 <= ny < 3:
                # Swapping the blank space with the corresponding adjacent tile
                self.swap(current_state, x, y, nx, ny)
                if self.solvePuzzle(current_state, path + [(nx, ny)]):
                    return True
                self.swap(current_state, x, y, nx, ny) # Else revert back to the original state
        return False

initial_puzzle = [[1, 2, 3], [4, 5, 0], [7, 8, 6]]
solver = SlidingPuzzleSolver(initial_puzzle)
if solver.solvePuzzle(initial_puzzle):
    print("Solution found!")
    print(solver.search_path)
else:
    print("No solution exists.")
