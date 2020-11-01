import plot, pickle, numpy as np
from theta_star import theta_star
from phi_star import phi_star
from dataset import mapGenerator
from utils import NoPathFound
from config import *

def dataAlreadyExist(data, mapId, trajId):
    try:
        next(filter(lambda e: e['mapId'] == mapId and e['trajId'] == trajId, data))
    except StopIteration:
        return False
    return True


def save(data):
    print('Saving data...')
    with open('data.backup', 'wb') as f:
        pickle.dump(data, f)


def load():
    try:
        with open('data.backup', 'rb') as f:
            return pickle.load(f)
    except:
        return []


def testAlgo(algo, start, goal, grid, newObs):
    durations = np.zeros((ALGO_DURATION_AVERAGE, BLOCKED_CELLS_LOOP + 1))
    for i in range(ALGO_DURATION_AVERAGE):
        g = np.copy(grid)
        _, _, durations[i, :], lengths, paths = algo(start, goal, g, newBlockedCells=newObs)
    return durations.mean(axis=0), durations.std(axis=0), lengths, paths


def main():
    data = load()
    print('Data already loaded for {} entries'.format(len(data)))

    i = 0
    for ids, start, goal, grid in mapGenerator():
        i += 1
        if dataAlreadyExist(data, ids[0], ids[1]):
            continue
        
        newObs = []
        for _ in range(BLOCKED_CELLS_LOOP):
            newObs.append(np.concatenate((np.random.randint(0, grid.shape[0], size=BLOCKED_CELLS).reshape(-1, 1), np.random.randint(0, grid.shape[1], size=BLOCKED_CELLS).reshape(-1, 1)), axis=1))

        try:
            # path, nodes, durationsPhi, lengthsPhi, pathsPhi = phi_star(start, goal, grid, newBlockedCells=newObs)
            durationsPhiMean, durationsPhiStd, lengthsPhi, pathsPhi = testAlgo(phi_star, start, goal, grid, newObs)
            durationsThetaMean, durationsThetaStd, lengthsTheta, pathsTheta = testAlgo(phi_star, start, goal, grid, newObs)
        except NoPathFound:
            continue
        
        print('Map {:8}: Phi*   {:5.2f}sec | {:6.2f} length'.format(str(ids), durationsPhiMean[0], lengthsPhi[0]))
        print('Map {:8}: Theta* {:5.2f}sec | {:6.2f} length'.format(str(ids), durationsThetaMean[0], lengthsTheta[0]))

        data.append(dict(algo='phi_star', mapId=ids[0], trajId=ids[1], durationsMean=durationsPhiMean, durationsStd=durationsPhiStd, lengths=lengthsPhi, paths=pathsPhi))
        data.append(dict(algo='theta_star', mapId=ids[0], trajId=ids[1], durationsMean=durationsThetaMean, durationsStd=durationsThetaStd, lengths=lengthsTheta, paths=pathsTheta))
        # plot.display(start, goal, grid_obs=grid, nodes=nodes, path=path, hold=1)

        if i % 20 == 0:
            save(data)

if __name__ == '__main__':
    main()