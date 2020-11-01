from theta_star import theta_star
from phi_star import phi_star
from dataset import mapGenerator
import plot

def main():
    for start, goal, grid in mapGenerator():
        try:
            path, nodes = phi_star(start, goal, grid)
        except ValueError:
            continue
        
        plot.display(start, goal, grid_obs=grid, nodes=nodes, path=path, hold=1)

if __name__ == '__main__':
    main()