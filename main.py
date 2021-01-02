import plot, pickle, numpy as np, sys
from theta_star import theta_star
from phi_star import phi_star
from dataset import mapGenerator
from utils import NoPathFound, Node
from config import *
from noise import pnoise2

def dataAlreadyExist(data, mapId, trajId):
    try:
        next(filter(lambda e: e['mapId'] == mapId and e['trajId'] == trajId, data))
    except StopIteration:
        return False
    return True


def save(data):
    print('Saving data...')
    with open('data_upsample.backup', 'wb') as f:
        pickle.dump(data, f)


def load():
    try:
        with open('data_upsample.backup', 'rb') as f:
            return pickle.load(f)
    except:
        return []


def testAlgo(algo, start, goal, grid, newObs):
    durations = np.zeros((ALGO_DURATION_AVERAGE, BLOCKED_CELLS_LOOP + 1))
    for i in range(ALGO_DURATION_AVERAGE):
        print(' Test #{}'.format(i))
        g = np.copy(grid)
        _, _, durations[i, :], lengths, paths = algo(start, goal, g, newBlockedCells=newObs)
    return durations.mean(axis=0), durations.std(axis=0), lengths, paths


def generateObstacles(start=(0,0), goal=(0,0), width=1600, height=1600):
    x, y = np.mgrid[0:width-1, 0:height-1]
    grid = np.vectorize(pnoise2)((x - np.random.randint(1000)) / 110, (y - np.random.randint(1000)) / 110)
    grid[grid <= .4] = Node.FREE
    grid[grid > .4] = Node.OBSTACLE
    grid[start], grid[goal[0]-1, goal[1]-1] = Node.FREE, Node.FREE
    x, y = np.where(grid == Node.OBSTACLE)
    return np.concatenate((x.reshape(-1, 1), y.reshape(-1, 1)), axis=1)


def main():
    data = load()
    print('Data already loaded for {} entries'.format(len(data)))

    i = 0
    for ids, start, goal, grid in mapGenerator(upsampling=40, cycleThroughMaps=True):
        i += 1
        if dataAlreadyExist(data, ids[0], ids[1]):
            continue
        
        newObs = []
        for _ in range(BLOCKED_CELLS_LOOP):
            newObs.append(generateObstacles(start, goal))
            # newObs.append(np.concatenate((np.random.randint(0, grid.shape[0], size=BLOCKED_CELLS).reshape(-1, 1), np.random.randint(0, grid.shape[1], size=BLOCKED_CELLS).reshape(-1, 1)), axis=1))

        try:
            # path, nodes, durationsPhi, lengthsPhi, pathsPhi = phi_star(start, goal, grid, newBlockedCells=newObs)
            print('Testing Phi*')
            durationsPhiMean, durationsPhiStd, lengthsPhi, pathsPhi = testAlgo(phi_star, start, goal, grid, newObs)
            print('Testing Theta*')
            durationsThetaMean, durationsThetaStd, lengthsTheta, pathsTheta = testAlgo(theta_star, start, goal, grid, newObs)
        except NoPathFound:
            continue
        
        print('*'*10)
        print('*** Map {:8}: Phi*   {:5.4f}±{:.5f}sec | {:6.2f} length'.format(str(ids), durationsPhiMean[0], durationsPhiStd[0], lengthsPhi[0]))
        print('*** Map {:8}: Theta* {:5.4f}±{:.5f}sec | {:6.2f} length'.format(str(ids), durationsThetaMean[0], durationsThetaStd[0], lengthsTheta[0]))
        print('*'*10)

        data.append(dict(mapId=ids[0], trajId=ids[1], durationsPhiMean=durationsPhiMean, durationsPhiStd=durationsPhiStd, lengthsPhi=lengthsPhi, pathsPhi=pathsPhi, 
                        durationsThetaMean=durationsThetaMean, durationsThetaStd=durationsThetaStd, lengthsTheta=lengthsTheta, pathsTheta=pathsTheta))
        # plot.display(start, goal, grid_obs=grid, nodes=nodes, path=path, hold=1)

        save(data)


def showMap(mapId):
    for ids, start, goal, grid in mapGenerator(upsampling=40, cycleThroughMaps=True):
        if ids[0] == mapId:
          break
    path, nodes, durationsPhi, lengthsPhi, pathsPhi = phi_star(start, goal, grid)


if __name__ == '__main__':
    if len(sys.argv) <= 1:
        main()
    else:
        showMap(int(sys.argv[1]))