"""
Implementation of the Theta* algorithm (Daniel, Nash, "Theta star, Any-angle path planning on grids").
This is simply to compare both Phi* and Theta* algorithms.
"""

import numpy as np, math, sys, time, matplotlib.pyplot as plt
import plot
from noise import pnoise2
from functools import reduce
from utils import dist, Node, lineOfSight, phi, lineOfSightNeighbors, corners
from collections import deque
from config import *


# Return all the children (neighbors) of a specific node
def children(node, grid, obs, crossbar=True, checkLOS=True):
    pos, c = np.array(node.pos), []
    if crossbar:
        directions = np.array([[1,0],[0,1],[0,-1],[-1,0]])
    else:
        directions = np.array([[1,0],[0,1],[1,1],[-1,-1],[1,-1],[-1,1],[0,-1],[-1,0]])
    for d in pos + directions:
        if 0 <= d[0] < grid.shape[0] and 0 <= d[1] < grid.shape[1] and (not checkLOS or lineOfSightNeighbors(node.pos, grid[d[0], d[1]].pos, obs)):
            c.append(grid[d[0], d[1]])
    return c
    

def updateVertex(current, node, grid, obs):
    if current.parent and lineOfSight(current.parent, node, grid, obs):
        # Path 2
        # If in line of sight, we connect to the parent, it avoid unecessary grid turns
        new_g = current.parent.G + dist(current.parent, node)
        if new_g < node.G:
            node.G = new_g
            node.parent = current.parent
            node.local = current
    else:
        # Path 1
        new_g = current.G + dist(current, node)
        if new_g < node.G:
            node.G = new_g
            node.parent = current
            node.local = current

# Return the path computed by the A* optimized algorithm from the start and goal points
def theta_star(start, goal, grid, obs, openset=set(), closedset=set()):
    if len(openset) == 0:
        openset.add(start)

    i = 0
    while min(map(lambda o: o.G + 1.5 * o.H, openset)) < goal.G + 1.5 * goal.H and openset:
        i = i + 1
        current = min(openset, key=lambda o: o.G + 1.5 * o.H)

        openset.remove(current)
        closedset.add(current)

        # Loop through the node's children/siblings
        for node in children(current, grid, obs, crossbar=False):
            # If it is already in the closed set, skip it
            if node in closedset:
                continue

            if node not in openset:
                node.G = float('inf')
                node.H = dist(node, goal)
                node.parent = None
                openset.add(node)

            updateVertex(current, node, grid, obs)
            
        if i % 1 == 0 and DISPLAY:
            plot.display(start, goal, grid, obs, nodes=openset.union(closedset), point=current, point2=node, showPath2=False)

    if not goal.parent:
        raise ValueError('No Path Found')
    
    path = []
    current = goal
    while current.parent:
        path.append(current)
        current = current.parent
    path.append(current)
    return path[::-1]


def main():
    start = (0, 0)
    goal = (WIDTH-1, HEIGHT-1)

    x, y = np.mgrid[0:WIDTH, 0:HEIGHT]
    x_obs, y_obs = np.mgrid[0:WIDTH-1, 0:HEIGHT-1]
    grid_obs = np.vectorize(pnoise2)((x_obs - OBSTACLE_X_OFFSET) / OBSTACLE_X_SIZE, (y_obs - OBSTACLE_Y_OFFSET) / OBSTACLE_Y_SIZE)
    grid_obs[grid_obs > OBSTACLE_THRESHOLD] = Node.OBSTACLE
    grid_obs[grid_obs <= OBSTACLE_THRESHOLD] = Node.FREE
    grid_obs[start], grid_obs[goal[0]-1, goal[1]-1] = Node.FREE, Node.FREE
    grid = np.vectorize(Node)(x, y)
    start, goal = grid[start], grid[goal]
    goal.H, start.G, start.H = 0, 0, dist(start, goal)

    openset = set()
    closedset = set()

    while True:
        t1 = time.time()
        path = theta_star(start, goal, grid, grid_obs, openset, closedset)
        duration = abs(time.time() - t1)
        print('Computation time:', duration)
        plot.display(start, goal, grid, grid_obs, nodes=openset.union(closedset), path=path)
        plot.waitForInput(grid_obs, lambda: plot.display(start, goal, grid, grid_obs))
        openset = set()
        closedset = set()
        goal.reset()


if __name__ == '__main__':
    main()
