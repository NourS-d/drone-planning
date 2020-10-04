import numpy as np
import matplotlib.pyplot as plt
from actions import Actions
from queue import PriorityQueue

class MotionPlanner:

    POSSIBLE_PLANNERS = [
        "a_star",
    ]

    def __init__(self, grid=None, planner="a_star", motion_model=Actions):
        self.grid = grid
        self._planner = planner
        self.motion_model = motion_model
        

    def run_planner(self, *args):
        """Start motion planner.
        
        Returns a tuple with the first element as a boolean indicating
        wether planning was successful or not.
        """
        
        if self.grid is None:
            raise ValueError("No grid specified!")

        return self.__getattribute__(self.planner)(*args)
        

    def a_star(self, start, goal, heuristic=None):
        
        if not isinstance(start, tuple):
            start = tuple(start)
        if not isinstance(goal, tuple):
            goal = tuple(goal)

        if heuristic is None:
            heuristic = self.euclidean_distance

        found = False
        path = []
        path_cost = 0

        open_set = PriorityQueue()
        open_set.put((0, start))
        
        came_from = {start: (0, None)}
        visited = [start]

        while not open_set.empty():
            cost_current, current = open_set.get()
            
            if current == goal:
                found = True
                break
            else:
                for action in self.motion_model:
                    new = tuple([current[i] + a for i, a 
                                                in enumerate(action.delta)])
                    
                    if not self.is_valid_node(new):
                        continue
                    cost_new = cost_current + action.cost + heuristic(new, goal)
                    if new not in visited:
                        visited.append(new)
                        open_set.put((cost_new, new))
                        came_from[new] = (cost_new, current)
        if found:
            # reconstruct path
            n = goal
            path_cost = came_from[n][0]
            while n in tuple(came_from):
                path.append(list(n))
                n = came_from[n][1]
            print(f"Success! Path found with cost {path_cost}.")
        else:
            print("No path found.")

        return found, path[::-1], path_cost

    def is_valid_node(self, node):
        """Check if a node is safe"""

        for i, v in enumerate(node):
            if not 0 <= v < self.grid.shape[i]:
                return False
        
        if self.grid.item(*node):
            return False
        
        return True
    
    def euclidean_distance(self, p1, p2, w=1.0):
        return w*np.linalg.norm(np.array(p1)-np.array(p2))
    @property
    def planner(self):
        return self._planner
    
    @planner.setter
    def planner(self, planner):
        if planner not in MotionPlanner.POSSIBLE_PLANNERS:
            print(f"Specified {planner} not implemented here. "
                  f"Using {MotionPlanner.POSSIBLE_PLANNERS[0]}.")
            self._planner = MotionPlanner.POSSIBLE_PLANNERS[0]
        else:
            self._planner = planner


if __name__ == "__main__":

    # Generate random grid
    GRID_SIDE = 100
    OBSTACLES = int(0.2 * GRID_SIDE * GRID_SIDE)
    grid = np.zeros((GRID_SIDE**2))
    grid[:OBSTACLES] = 1
    np.random.shuffle(grid)
    grid = np.reshape(grid,(GRID_SIDE,GRID_SIDE))
    
    # Random start and goal points
    start, goal =  np.random.randint(0, high=GRID_SIDE, size=(2, 2))
    
    mp = MotionPlanner(grid)

    success, path, cost = mp.run_planner(start, goal)
    if success:
        path = np.array(path)
        print(path)
        plt.matshow(grid)
        plt.plot(path[:, 1], path[:, 0], 'r')
        plt.show()
