import numpy as np, matplotlib.pyplot as plt, matplotlib.patches as patches, matplotlib.collections as collections
from utils import supercover, Node, lineOfSightNeighbors, lineOfSight, dist, phi

fig, ax = plt.subplots()

def display(start, goal, grid, grid_obs, path=[], nodes=[], point=None, point2=None, showPath2=True, hold=False):
    ax.clear()
    ax.set_xlim(-0.5, grid.shape[0])
    ax.set_ylim(-0.5, grid.shape[1])

    obs = []
    x, y = np.mgrid[0:grid_obs.shape[0], 0:grid_obs.shape[1]]
    np.vectorize(lambda node, x, y: obs.append(patches.Rectangle([x, y], 1, 1)) if node == Node.OBSTACLE else None)(grid_obs, x, y)
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

    if start:
        ax.add_patch(patches.Circle(start.pos, .3, linewidth=1, facecolor='green'))
    if goal:
        ax.add_patch(patches.Circle(goal.pos, .3, linewidth=1, facecolor='blue'))
    if point:
        ax.add_patch(patches.Circle(point.pos, .3, linewidth=1, facecolor='red'))
    if point2:
        ax.add_patch(patches.Circle(point2.pos, .2, linewidth=1, facecolor='magenta'))
    
    if point and point2 and lineOfSightNeighbors(point.pos, point2.pos, grid_obs):
        ax.add_patch(patches.Arrow(point.pos[0], point.pos[1], point2.pos[0]-point.pos[0], point2.pos[1]-point.pos[1], .4, facecolor='red'))
    
    if point and point2 and point.parent and lineOfSight(point.parent, point2, grid, grid_obs) and showPath2:
        ax.add_patch(patches.Arrow(point.parent.pos[0], point.parent.pos[1], point2.pos[0]-point.parent.pos[0], point2.pos[1]-point.parent.pos[1], .3, facecolor='magenta'))
    
    if point and point2 and point.parent:
        rect = []
        for pt in supercover(point.parent, point2):
            rect.append(patches.Rectangle([pt[0], pt[1]], 1, 1, facecolor='black', alpha=.1))
        ax.add_collection(collections.PatchCollection(rect, match_original=True))
    
    if point and point.parent and showPath2:
        mid_angle = phi([point.parent.pos[0]+1, point.parent.pos[1]], point.parent, point)
        ax.add_patch(patches.Wedge(point.parent.pos, 5, mid_angle + point.lb, mid_angle + point.ub, facecolor='cyan', alpha=.3))

        
    if len(path) > 0:
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

    plt.title('Processing...')
    if hold:
        plt.show()
    else:
        plt.pause(.01)
    

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
