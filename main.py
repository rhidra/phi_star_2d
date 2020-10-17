import numpy as np, math, sys, time
import plot
from noise import pnoise2
from functools import reduce
from utils import dist, Node, lineOfSight, phi, lineOfSightNeighbors


# Return all the children (neighbors) of a specific node
def children(node, grid, obs, crossbar=True):
    pos, c = np.array(node.pos), []
    if crossbar:
        directions = np.array([[1,0],[0,1],[0,-1],[-1,0]])
    else:
        directions = np.array([[1,0],[0,1],[1,1],[-1,-1],[1,-1],[-1,1],[0,-1],[-1,0]])
    for d in pos + directions:
        if 0 <= d[0] < grid.shape[0] and 0 <= d[1] < grid.shape[1] and lineOfSightNeighbors(node.pos, grid[d[0], d[1]].pos, obs):
            c.append(grid[d[0], d[1]])
    return c


def pathTie(node, current):
    angle = abs(phi(current.parent, current, node))
    return math.isclose(angle, 180, abs_tol=1e-6)


# Return the path computed by the A* optimized algorithm from the start and goal points
def phi_star(start, goal, grid, obs):
    openset = set()
    closedset = set()

    current = start
    openset.add(current)

    i = 0
    while openset:
        i = i + 1

        current = min(openset, key=lambda o: o.G + 1.5 * o.H)

        if current == goal:
            path = []
            while current.parent:
                path.append(current)
                current = current.parent
            path.append(current)
            return path[::-1]

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

            if current.parent and lineOfSight(current.parent, node, grid, obs) \
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

        if i % 10 == 0:
            plot.display(start, goal, grid, obs, nodes=openset, point=current, point2=node)

    raise ValueError('No Path Found')


def main(obs_threshold=.2):
    width, height = 100, 100
    start = (0, 0)
    goal = (width-1, height-1)

    x, y = np.mgrid[0:width, 0:height]
    x_obs, y_obs = np.mgrid[0:width-1, 0:height-1]
    grid_obs = np.vectorize(pnoise2)(x_obs / 6+100, y_obs / 6+100)
    grid_obs[grid_obs > obs_threshold] = Node.OBSTACLE
    grid_obs[grid_obs <= obs_threshold] = Node.FREE
    grid_obs[start], grid_obs[goal[0]-1, goal[1]-1] = Node.FREE, Node.FREE
    grid = np.vectorize(Node)(x, y)
    start, goal = grid[start], grid[goal]

    print('Computing shortest path with Phi* ...')
    path = phi_star(start, goal, grid, grid_obs)
    plot.display(start, goal, grid, grid_obs, path=path, hold=True)

if __name__ == '__main__':
    main()
