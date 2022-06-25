## EX NO:04
## DATE:17.5.22
# <p align="center">A* Path Finding Algorithm for 2D Grid World
## AIM

To develop a code to find the route from the source to the destination point using A* algorithm for 2D grid world.

## THEORY
A* assigns a weight to each open node equal to the weight of the edge to that node plus the approximate distance between that node and the finish. This approximate distance is found by the heuristic, and represents a minimum possible distance between that node and the end.

## DESIGN STEPS

### STEP 1:
Build a 2D grid world with initial state , goal state and obstacles

### STEP 2:
Create a class with the appropriate functions.

### STEP 3:
Build a 2D grid world with the initial state, goal state, and obstacles.

### STEP 4:
Mention its evaluation function f(n) = g(n) + h(n)

### STEP 5:
The algorithm finds the best optimal solution.


## 2D Grid
![Screenshot (336)](https://user-images.githubusercontent.com/75235090/168829300-8f53b0ba-77ec-42fa-8082-822cce343641.png)

## PROGRAM
```python
    Developed by: Prasannakumar M
    Register No:212220230035 
%matplotlib inline
import matplotlib.pyplot as plt
import random
import math
import sys
from collections import defaultdict, deque, Counter
from itertools import combinations
import heapq

Problems
This is the abstract class. Specific problem domains will subclass this.
class Problem(object):
    """The abstract class for a formal problem. A new domain subclasses this,
    overriding `actions` and `results`, and perhaps other methods.
    The default heuristic is 0 and the default action cost is 1 for all states.
    When yiou create an instance of a subclass, specify `initial`, and `goal` states 
    (or give an `is_goal` method) and perhaps other keyword args for the subclass."""

    def __init__(self, initial=None, goal=None, **kwds): 
        self.__dict__.update(initial=initial, goal=goal, **kwds) 
        
    def actions(self, state):        
        raise NotImplementedError
    def result(self, state, action): 
        raise NotImplementedError
    def is_goal(self, state):        
        return state == self.goal
    def action_cost(self, s, a, s1): 
        return 1
    
    def __str__(self):
        return '{0}({1}, {2})'.format(
            type(self).__name__, self.initial, self.goal)

Nodes
This is the Node in the search tree. Helper functions (expand, path_actions, path_states) use this Node class
class Node:
    "A Node in a search tree."
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.__dict__.update(state=state, parent=parent, action=action, path_cost=path_cost)

    def __str__(self): 
        return '<{0}>'.format(self.state)
    def __len__(self): 
        return 0 if self.parent is None else (1 + len(self.parent))
    def __lt__(self, other): 
        return self.path_cost < other.path_cost
failure = Node('failure', path_cost=math.inf) # Indicates an algorithm couldn't find a solution.
cutoff  = Node('cutoff',  path_cost=math.inf) # Indicates iterative deepening search was cut off.

Helper functions
def expand(problem, node):
    "Expand a node, generating the children nodes."
    s = node.state
    for action in problem.actions(s):
        s1 = problem.result(s, action)
        cost = node.path_cost + problem.action_cost(s, action, s1)
        yield Node(s1, node, action, cost)
        

def path_actions(node):
    "The sequence of actions to get to this node."
    if node.parent is None:
        return []  
    return path_actions(node.parent) + [node.action]


def path_states(node):
    "The sequence of states to get to this node."
    if node in (cutoff, failure, None): 
        return []
    return path_states(node.parent) + [node.state]

Search Algorithm : Best First Search
class PriorityQueue:
    """A queue in which the item with minimum f(item) is always popped first."""

    def __init__(self, items=(), key=lambda x: x): 
        self.key = key
        self.items = [] # a heap of (score, item) pairs
        for item in items:
            self.add(item)
         
    def add(self, item):
        """Add item to the queuez."""
        pair = (self.key(item), item)
        heapq.heappush(self.items, pair)

    def pop(self):
        """Pop and return the item with min f(item) value."""
        return heapq.heappop(self.items)[1]
    
    def top(self): return self.items[0][1]

    def __len__(self): return len(self.items)
def best_first_search(problem, f):
    "Search nodes with minimum f(node) value first."
    node = Node(problem.initial)
    frontier = PriorityQueue([node], key=f)
    reached = {problem.initial: node}
    while frontier:
        node = frontier.pop()
        if problem.is_goal(node.state):
            return node
        for child in expand(problem, node):
            s = child.state
            if s not in reached or child.path_cost < reached[s].path_cost:
                reached[s] = child
                frontier.add(child)
    return failure

def g(n): 
    return n.path_cost

2D Grid Pathfinding Problem
class GridProblem(Problem):
    """Finding a path on a 2D grid with obstacles. Obstacles are (x, y) cells."""

    def __init__(self, initial=(5, 10), goal=(5, 3), obstacles=(), **kwds):
        Problem.__init__(self, initial=initial, goal=goal, 
                         obstacles=set(obstacles) - {initial, goal}, **kwds)

    directions = [(-1, -1), (0, -1), (1, -1),
                  (-1, 0),           (1,  0),
                  (-1, +1), (0, +1), (1, +1)]
    
    def action_cost(self, s, action, s1): 
        return straight_line_distance(s, s1)
    
    def h(self, node): 
        return straight_line_distance(node.state, self.goal)
                  
    def result(self, state, action): 
        "Both states and actions are represented by (x, y) pairs."
        return action if action not in self.obstacles else state
    
    def actions(self, state):
        """You can move one cell in any of `directions` to a non-obstacle cell."""
        
        x, y = state
        return {(x + dx, y +dy) for (dx, dy) in self.directions} - self.obstacles
def straight_line_distance(A, B):
    "Straight-line distance between two points."
    return sum(abs(a - b)**2 for (a, b) in zip(A, B)) ** 0.5
def g(n): 
    return n.path_cost
def astar_search(problem, h=None):
    """Search nodes with minimum f(n) = g(n) + h(n)."""
    h = h or problem.h
    return best_first_search(problem, f=lambda n: g(n) + h(n))
obstacles=({(1,1),(1,5),(1,7),(2,3),(2,8),(3,1),(3,3),(3,5),(4,4)})
grid1 = GridProblem(initial=(1,2), goal =(3,10) ,obstacles=obstacles)
solution1 = astar_search(grid1)
path_states(solution1)
```

## OUTPUT:

![Screenshot (340)](https://user-images.githubusercontent.com/75235090/168831702-4dcb7e5b-be9d-4224-82c7-8b97b81cbecf.png)


Explain the Completeness and complexity of the algorithm:
This algorithm is complete if it guarantees to return a correct answer for any arbitrary input (or, if no answer exists, it guarantees to return failure).
The time complexity of A* depends on the heuristic.

## RESULT:
Thus, the algorithm finds the best path to reach the final goal using Informed (Heuristic) Search Strategies.
