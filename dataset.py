"""
dataset.py

Parser and utility functions for the Occ-Traj120 trajectories dataset (https://github.com/soraxas/Occ-Traj120).
"""

import os, re, numpy as np, plot
from utils import Node

def mapPath(id):
    return './dataset/occtraj_{}_map.txt'.format(id)

def trajPath(id):
    return './dataset/occtraj_{}_trajs.txt'.format(id)

def getIdList():
    l = os.listdir('./dataset')
    l = filter(lambda s: re.match('occtraj_(.*)_map.txt', s), l)
    l = map(lambda s: re.search('occtraj_(.*)_map.txt', s).group(1), l)
    return list(l)

def readMap(id):
    grid = []
    with open(mapPath(id)) as f:
        for line in f.readlines():
            grid.append([])
            for c in line:
                if c != '1' and c != '0':
                   continue
                grid[-1].append(Node.OBSTACLE if c == '1' else Node.FREE)
    grid = np.array(grid)
    return grid

def readTraj(id):
    trajs = []
    with open(trajPath(id)) as f:
        while True:
            id = f.readline()
            if id == '':
                break
            id = re.search('traj-([0-9]+):', id).group(1)
            xs = []
            ys = []
            for x in f.readline().split(' '):
                try:
                    xs.append(float(x))
                except:
                    continue
            for y in f.readline().split(' '):
                try:
                    ys.append(float(y))
                except:
                    continue
            trajs.append({'id': id, 'x': np.array(ys), 'y': np.array(xs)})
    return trajs


def mapGenerator():
    for id in getIdList():
        grid = readMap(id)
        trajs = readTraj(id)
        for traj in trajs:
            x, y = traj['x'], traj['y']
            start, goal = (int(x[0]), int(y[0])), (int(x[-1]), int(y[-1]))
            # start, goal = ((x[0]), (y[0])), ((x[-1]), (y[-1]))
            grid[start], grid[goal] = Node.FREE, Node.FREE
            yield start, goal, grid

