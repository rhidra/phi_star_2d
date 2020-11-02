import numpy as np, matplotlib.pyplot as plt, matplotlib.patches as patches, matplotlib.collections as collections
from utils import supercover, Node, lineOfSightNeighbors, lineOfSight, dist, phi
from config import DISPLAY_DELAY

fig, ax = plt.subplots()

def display(start=None, goal=None, grid=[], grid_obs=[], path=[], nodes=[], point=None, point2=None, showPath2=True, hold=False):
    print('  Plotting...')
    ax.clear()
    if len(grid) != 0:
        ax.set_xlim(-0.5, grid.shape[0])
        ax.set_ylim(-0.5, grid.shape[1])
    elif len(grid_obs) != 0:
        ax.set_xlim(-0.5, grid_obs.shape[0])
        ax.set_ylim(-0.5, grid_obs.shape[0])

    if len(grid_obs) != 0:
        # obs = []
        # x, y = np.mgrid[0:grid_obs.shape[0], 0:grid_obs.shape[1]]
        # np.vectorize(lambda node, x, y: obs.append(patches.Rectangle([x, y], 1, 1)) if node == Node.OBSTACLE else None)(grid_obs, x, y)
        obs = [patches.Rectangle([x, y], w, h) for x, y, w, h in extractRect(grid_obs)]
        ax.add_collection(collections.PatchCollection(obs))

    lines = []
    for node in nodes:
        pt_list = []
        while node.local:
            pt_list.append([node.pos[0], node.pos[1]])            
            node = node.local
        pt_list.append([node.pos[0], node.pos[1]])
        lines.append(pt_list)
    ax.add_collection(collections.LineCollection(lines, colors='green', alpha=1 if len(path) == 0 else .5))

    lines = []
    for node in nodes:
        pt_list = []
        while node.parent:
            pt_list.append([node.pos[0], node.pos[1]])            
            node = node.parent
        pt_list.append([node.pos[0], node.pos[1]])
        lines.append(pt_list)
    ax.add_collection(collections.LineCollection(lines, colors='red', alpha=1 if len(path) == 0 else .5))

    if start is not None:
        ax.add_patch(patches.Circle(start.pos if isinstance(start, Node) else start, .3, linewidth=1, facecolor='green'))
    if goal is not None:
        ax.add_patch(patches.Circle(goal.pos if isinstance(goal, Node) else goal, .3, linewidth=1, facecolor='blue'))
    if point:
        ax.add_patch(patches.Circle(point.pos, .3, linewidth=1, facecolor='red'))
    if point2:
        ax.add_patch(patches.Circle(point2.pos, .2, linewidth=1, facecolor='magenta'))
    
    if point and point2 and lineOfSightNeighbors(point.pos, point2.pos, grid_obs):
        ax.add_patch(patches.Arrow(point.pos[0], point.pos[1], point2.pos[0]-point.pos[0], point2.pos[1]-point.pos[1], .4, facecolor='red'))
    
    if point and point2 and point.parent and lineOfSight(point.parent, point2, grid_obs) and showPath2:
        ax.add_patch(patches.Arrow(point.parent.pos[0], point.parent.pos[1], point2.pos[0]-point.parent.pos[0], point2.pos[1]-point.parent.pos[1], .3, facecolor='magenta'))
    
    if point and point2 and point.parent:
        rect = []
        for pt in supercover(point.parent, point2):
            rect.append(patches.Rectangle([pt[0], pt[1]], 1, 1, facecolor='black', alpha=.1))
        ax.add_collection(collections.PatchCollection(rect, match_original=True))
    
    if point and point.parent and showPath2:
        mid_angle = phi([point.parent.pos[0]+1, point.parent.pos[1]], point.parent, point)
        ax.add_patch(patches.Wedge(point.parent.pos, 5, mid_angle + point.lb, mid_angle + point.ub, facecolor='cyan', alpha=.3))

        
    if len(path) > 0 and isinstance(path[0], Node):
        local = []
        node = path[-1]
        while node.local:
            local.append(node.pos)
            node = node.local
        local.append(node.pos)
        local = np.array(local)
        path_ = np.array([p.pos for p in path])
        plt.plot(local[:, 0], local[:, 1], color='green', linewidth=3)
        plt.plot(path_[:, 0], path_[:, 1], color='red', linewidth=4)

        pts = []
        node = path[-1]
        while node.parent:
            pts += supercover(node, node.parent)
            node = node.parent
        ax.add_collection(collections.PatchCollection([patches.Rectangle([p[0], p[1]], 1, 1, linewidth=1, facecolor='orange', alpha=.5) for p in pts], match_original=True))

    elif len(path) > 0:
        plt.plot(path[:, 0], path[:, 1], 'o-', color='red', linewidth=1)

    plt.title('Processing...')
    if hold and isinstance(hold, bool):
        plt.show()
    else:
        plt.pause(DISPLAY_DELAY if isinstance(hold, bool) else hold)
    print('End plot')


def extractRect(grid):
    rects = []
    def alreadyDone(i, j):
        for x, y, w, h in rects:
            if x <= i < x+w and y <= j < y+h:
                return True
        return False

    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            if not alreadyDone(i, j) and grid[i, j] == Node.OBSTACLE:
                for k in range(i+1, grid.shape[0]):
                    if grid[k, j] == Node.FREE:
                        break
                    k += 1
                imax = k
                for k in range(j+1, grid.shape[1]):
                    if grid[i:imax, j:k][grid[i:imax, j:k] == Node.FREE].size != 0:
                        break
                    k += 1
                jmax = k
                rects.append([i, j, imax-i, jmax-j])
    return rects




def waitForInput(obs, plotCb):
    refreshDisplay = False
    inputPending = True
    blockedCells = []

    def onclick(event):
        nonlocal refreshDisplay
        refreshDisplay = True
        x, y = int(event.xdata), int(event.ydata)
        obs[x, y] = Node.OBSTACLE if obs[x, y] == Node.FREE else Node.FREE
        blockedCells.append((x, y))

    def onkey(event):
        nonlocal refreshDisplay, inputPending
        if event.key == 'enter':
            refreshDisplay = True
            inputPending = False

    cid1 = fig.canvas.mpl_connect('button_press_event', onclick)
    cid2 = fig.canvas.mpl_connect('key_press_event', onkey)

    while inputPending:
        plt.title('Waiting for input... Press Enter to confirm')
        while not refreshDisplay:
            plt.pause(.001)
        refreshDisplay = False
        plotCb()
    
    fig.canvas.mpl_disconnect(cid1)
    fig.canvas.mpl_disconnect(cid2)
    return blockedCells
