import collections
import time
import resource
import sys
import math


class PuzzleState(object):
    """docstring for PuzzleState"""

    def __init__(self, config, n, parent=None, action="Initial", cost=0, depth=0):
        if n * n != len(config) or n < 2:
            raise Exception("the length of config is not correct!")
        self.n = n
        self.cost = cost
        self.depth = depth
        self.parent = parent
        self.action = action
        self.dimension = n
        self.config = config
        self.children = []
        for i, item in enumerate(self.config):
            if item == 0:
                self.blank_row = i // self.n
                self.blank_col = i % self.n
                break

    def display(self):
        for i in range(self.n):
            line = []
            offset = i * self.n
            for j in range(self.n):
                line.append(self.config[offset + j])
            print(line)

    def move_left(self):
        if self.blank_col == 0:
            return None
        else:
            blank_index = self.blank_row * self.n + self.blank_col
            target = blank_index - 1
            new_config = list(self.config)
            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]
            return PuzzleState(tuple(new_config),
                               self.n,
                               parent=self,
                               action="Left",
                               cost=self.cost + 1,
                               depth=self.depth + 1)

    def move_right(self):
        if self.blank_col == self.n - 1:
            return None
        else:
            blank_index = self.blank_row * self.n + self.blank_col
            target = blank_index + 1
            new_config = list(self.config)
            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]
            return PuzzleState(tuple(new_config),
                               self.n,
                               parent=self,
                               action="Right",
                               cost=self.cost + 1,
                               depth=self.depth + 1)

    def move_up(self):
        if self.blank_row == 0:
            return None
        else:
            blank_index = self.blank_row * self.n + self.blank_col
            target = blank_index - self.n
            new_config = list(self.config)
            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]
            return PuzzleState(tuple(new_config),
                               self.n,
                               parent=self,
                               action="Up",
                               cost=self.cost + 1,
                               depth=self.depth + 1)

    def move_down(self):
        if self.blank_row == self.n - 1:
            return None
        else:
            blank_index = self.blank_row * self.n + self.blank_col
            target = blank_index + self.n
            new_config = list(self.config)
            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]
            return PuzzleState(tuple(new_config),
                               self.n,
                               parent=self,
                               action="Down",
                               cost=self.cost + 1,
                               depth=self.depth + 1)

    def expand(self):
        """expand the node"""
        # add child nodes in order of UDLR
        if len(self.children) == 0:
            up_child = self.move_up()
            if up_child is not None:
                self.children.append(up_child)
            down_child = self.move_down()
            if down_child is not None:
                self.children.append(down_child)
            left_child = self.move_left()
            if left_child is not None:
                self.children.append(left_child)
            right_child = self.move_right()
            if right_child is not None:
                self.children.append(right_child)
        return self.children


class Result:
    def __init__(self, result_state, nodes_expanded, max_search_depth):
        self.result_state = result_state
        self.nodes_expanded = nodes_expanded
        self.max_search_depth = max_search_depth


# Function that Writes to output.txt
# Students need to change the method to have the corresponding parameters
def write_output(result, duration):
    path = []
    state = result.result_state
    memory_usage = format(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1000.0, '.8f')

    while state.action != "Initial":
        path.insert(0, state.action)
        state = state.parent

    print("path_to_goal:", path)
    print("cost_of_path:", result.result_state.cost)
    print("nodes_expanded:", result.nodes_expanded)
    print("search_depth:", result.result_state.depth)
    print("max_search_depth:", result.max_search_depth)
    print("running_time:", format(duration, '.8f'))
    print("max_ram_usage:", memory_usage, '.8f')

    file = open('output.txt', 'w')
    file.write("path_to_goal: " + str(path) + "\n")
    file.write("cost_of_path: " + str(result.result_state.cost) + "\n")
    file.write("nodes_expanded: " + str(result.nodes_expanded) + "\n")
    file.write("search_depth: " + str(result.result_state.depth) + "\n")
    file.write("max_search_depth: " + str(result.max_search_depth) + "\n")
    file.write("running_time: " + format(duration, '.8f') + "\n")
    file.write("max_ram_usage: " + str(memory_usage) + "\n")
    file.close()


def bfs_search(initial_state):
    explored = set()
    frontier = collections.deque([initial_state])
    nodes_expanded = 0
    max_search_depth = 0

    while frontier:
        state: PuzzleState = frontier.popleft()
        explored.add(state.config)
        if test_goal(state):
            return Result(state, nodes_expanded, max_search_depth)

        nodes_expanded += 1
        for child in state.expand():
            if child.config not in explored:
                frontier.append(child)
                explored.add(child.config)
                if child.depth > max_search_depth:
                    max_search_depth += 1


def dfs_search(initial_state):
    explored = set()
    frontier = list([initial_state])
    nodes_expanded = 0
    max_search_depth = 0

    while frontier:
        state: PuzzleState = frontier.pop()
        explored.add(state.config)
        if test_goal(state):
            return Result(state, nodes_expanded, max_search_depth)

        nodes_expanded += 1
        for child in reversed(state.expand()):
            if child.config not in explored:
                frontier.append(child)
                explored.add(child.config)
                if child.depth > max_search_depth:
                    max_search_depth += 1


class AStarState:
    def __init__(self, key, puzzle_state):
        self.key = key
        self.puzzle_state = puzzle_state


def a_star_search(initial_state):
    explored = set()
    frontier = list([AStarState(calculate_manhattan_dist(initial_state), initial_state)])
    nodes_expanded = 0
    max_search_depth = 0

    while frontier:
        frontier.sort(key=lambda o: o.key, reverse=True)
        state: AStarState = frontier.pop()
        explored.add(state.puzzle_state.config)
        if test_goal(state.puzzle_state):
            return Result(state.puzzle_state, nodes_expanded, max_search_depth)

        nodes_expanded += 1
        for child in state.puzzle_state.expand():
            if child.config not in explored:
                frontier.append(AStarState(calculate_manhattan_dist(child) + child.depth, child))
                explored.add(child.config)
                if child.depth > max_search_depth:
                    max_search_depth += 1


def calculate_manhattan_dist(initial_state: PuzzleState):
    distance = 0
    for i, item in enumerate(initial_state.config):
        row, col = int(i / 3), i % 3
        goal_row, goal_col = int(item / 3), item % 3
        distance += abs(row - goal_row) + abs(col - goal_col)
    return distance


def test_goal(puzzle_state):
    return puzzle_state.config == (0, 1, 2, 3, 4, 5, 6, 7, 8)


# Main Function that reads in Input and Runs corresponding Algorithm
def main():
    sm = sys.argv[1].lower()
    begin_state = sys.argv[2].split(",")
    begin_state = tuple(map(int, begin_state))
    size = int(math.sqrt(len(begin_state)))
    hard_state = PuzzleState(begin_state, size)
    result = None

    start_time = time.time()

    if sm == "bfs":
        result = bfs_search(hard_state)
    elif sm == "dfs":
        result = dfs_search(hard_state)
    elif sm == "ast":
        result = a_star_search(hard_state)
    else:
        print("Enter valid command arguments !")

    write_output(result, time.time() - start_time)


if __name__ == '__main__':
    main()
