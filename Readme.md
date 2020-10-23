Phi* 2D
=======

2D Implementation of the paper by Nash and Koenig 
[_Incremental Phi star, incremental any-angle path planning on grids_](https://repository.upenn.edu/cgi/viewcontent.cgi?article=1021&context=grasp_papers)
using Python, Numpy and Matplotlib.

To run the simulation use:

```shell script
# Run the simulation with the Phi* path planning algorithm
python phi_star.py


# Run the simulation with the Theta* path planning algorithm
python theta_star.py
```

To configure the setup of the simulation, you can edit the `config.py` file.

## Performance analysis with Theta*

Theta*: [Daniel, Nash, "Theta star, Any-angle path planning on grids"](https://www.jair.org/index.php/jair/article/download/10676/25515/)

### Time comparison

| # | Size         | Obstacle size | Offset | Theta*  | Theta* - Replanning (end) | Phi*   | Phi* - Replanning (mid) | Phi* - Replanning (end) |
|---|--------------|---------------|--------|---------|---------------------------|--------|-------------------------|-------------------------|
| 1 | (100, 100)   | 6             | 0      | 0.04888 | 0.05731                   | 0.7533 | 0.03627                 | 0.03152                 |
| 2 | (500, 500)   | 6             | 0      | 0.5551  | 0.5386                    | 0.7062 | 0.5521                  | 0.3449                  |
| 3 | (1000, 1000) | 20            | 0      | 2.578   |                           | 2.797  |                         |                         |
| 4 | (5000, 5000) | 20            | 0      | 142.50  |                           | 210.18 |                         |                         |

### Length comparison

| # | Size     | Obstacle size | Offset | Theta* | Phi*  |
|---|----------|---------------|--------|--------|-------|
| 1 | (30, 30) | 6             | 0      | 42.71  | 44.01 |