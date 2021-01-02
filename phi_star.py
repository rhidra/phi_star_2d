import numpy as np, math, sys, time, matplotlib.pyplot as plt
import plot
from noise import pnoise2
from functools import reduce
from utils import dist, Node, lineOfSight, phi, lineOfSightNeighbors, corners, pathLength, updateGridBlockedCells, NoPathFound
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


def pathTie(node, current):
    return False
    angle = abs(phi(current.parent, current, node))
    return math.isclose(angle, 180, abs_tol=1e-6)
    

def updateVertex(current, node, grid, obs):
    if current.parent and lineOfSight(current.parent, node, obs) \
                    and current.lb <= phi(current, current.parent, node) <= current.ub \
                    and not pathTie(node, current):
        # Path 2
        # If in line of sight, we connect to the parent, it avoid unecessary grid turns
        new_g = current.parent.G + dist(current.parent, node)
        showPath2 = True
        if new_g < node.G:
            node.G = new_g
            node.parent = current.parent
            node.local = current
            neighbors = list(map(lambda nb: phi(node, current.parent, nb), children(node, grid, obs, crossbar=True)))
            l = min(neighbors)
            h = max(neighbors)
            delta = phi(current, current.parent, node)
            node.lb = max(l, current.lb - delta)
            node.ub = min(h, current.ub - delta)
    else:
        # Path 1
        showPath2 = False
        new_g = current.G + dist(current, node)
        if new_g < node.G:
            node.G = new_g
            node.parent = current
            node.local = current
            node.lb = -45
            node.ub = 45

    return showPath2

# Return the path computed by the A* optimized algorithm from the start and goal points
def find_path(start, goal, grid, obs, openset=set(), closedset=set()):
    startTime = time.time()
    if len(openset) == 0:
        openset.add(start)

    i = 0
    while openset and min(map(lambda o: o.G + H_COST_WEIGHT * o.H, openset)) < goal.G + H_COST_WEIGHT * goal.H and time.time() - startTime < TIME_OUT:
        i = i + 1
        current = min(openset, key=lambda o: o.G + H_COST_WEIGHT * o.H)

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
                node.ub = float('inf')
                node.lb = - float('inf')
                openset.add(node)
            
            showPath2 = updateVertex(current, node, grid, obs)

        if DISPLAY and i % DISPLAY_FREQ == 0:
            plot.display(start, goal, grid, obs, nodes=openset.union(closedset), point=current, point2=node, showPath2=showPath2)

    if not goal.parent:
        print('  No path found !')
        raise NoPathFound
    
    path = []
    current = goal
    while current.parent:
        path.append(current)
        current = current.parent
    path.append(current)
    return path[::-1]


def clearSubtree(node, grid, obs, openset, closedset):
    under, over = deque(), deque()
    under.append(node)

    while under:
        node = under.popleft()
        over.append(node)
        node.reset()

        openset.discard(node)
        closedset.discard(node)

        for neigh in children(node, grid, [], crossbar=False, checkLOS=False):
            if neigh.local == node:
                under.append(neigh)
    
    while over:
        node = over.popleft()
        for neigh in children(node, grid, obs, crossbar=False, checkLOS=True):
            if neigh in closedset:
                g_old = node.G
                updateVertex(neigh, node, grid, obs)
                if node.G < g_old:
                    openset.add(node)


def phi_star(start, goal, grid_obs, newBlockedCells=[]):
    print('  Computing Phi* algorithm...')
    durations = []
    lengths = []
    paths = []

    x, y = np.mgrid[0:grid_obs.shape[0]+1, 0:grid_obs.shape[1]+1]
    grid = np.vectorize(Node)(x, y)
    start, goal = grid[start], grid[goal]
    goal.H, start.G, start.H = 0, 0, dist(start, goal)

    newBlockedCells = iter(newBlockedCells)
    openset = set()
    closedset = set()

    t1 = time.time()
    duration = 0

    i = 0
    while True:
        print('  Planning #{}'.format(i))
        i += 1
        path = find_path(start, goal, grid, grid_obs, openset, closedset)
        
        if not DISPLAY:
            duration = abs(time.time() - t1)
        durations.append(duration)
        lengths.append(pathLength(path))
        paths.append(list(map(lambda n: n.pos, path)))

        if DISPLAY_END:
            plot.display(start, goal, grid, grid_obs, nodes=openset.union(closedset), path=path)

        if not REPLANNING:
            break
        

        if DISPLAY_END and WAIT_INPUT:
            blockedCells = plot.waitForInput(grid_obs, lambda: plot.display(start, goal, grid, grid_obs))
        else:
            try:
                blockedCells = next(newBlockedCells)
                updateGridBlockedCells(blockedCells, grid_obs)
            except StopIteration:
                break
              
        t1 = time.time()

        for pt in corners(blockedCells):
            if (grid[pt] in openset or grid[pt] in closedset) and grid[pt] != start:
                clearSubtree(grid[pt], grid, grid_obs, openset, closedset)
    return path, openset.union(closedset), np.array(durations), np.array(lengths), paths


def main():
    start = (0, 0)
    goal = (WIDTH-1, HEIGHT-1)

    x_obs, y_obs = np.mgrid[0:WIDTH-1, 0:HEIGHT-1]
    grid_obs = np.vectorize(pnoise2)(x_obs / OBSTACLE_X_SIZE, y_obs / OBSTACLE_Y_SIZE)
    grid_obs[grid_obs > OBSTACLE_THRESHOLD] = Node.OBSTACLE
    grid_obs[grid_obs <= OBSTACLE_THRESHOLD] = Node.FREE
    grid_obs[start], grid_obs[goal[0]-1, goal[1]-1] = Node.FREE, Node.FREE
    
    phi_star(start, goal, grid_obs)


if __name__ == '__main__':
    main()
