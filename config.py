# Algorithm parameters
H_COST_WEIGHT = 1.7

# Parameters for map generation with Perlin noise
WIDTH = 20
HEIGHT = 20
OBSTACLE_THRESHOLD = .4
OBSTACLE_X_SIZE = 4
OBSTACLE_Y_SIZE = 4
OBSTACLE_X_OFFSET = 0
OBSTACLE_Y_OFFSET = 0

# Real time display of the algorithm
DISPLAY = True
DISPLAY_DELAY = .1
DISPLAY_FREQ = 1
DISPLAY_END = True # Should display for a few seconds at the end of the algorithm (before auto replanning)
WAIT_INPUT = DISPLAY_END and True

# Allow replanning, through user input or programmatically
REPLANNING = True

# For automatic blocked cell generation
BLOCKED_CELLS = 20 # Number of newly blocked cell in one loop 
BLOCKED_CELLS_LOOP = 5 # Number of loops of replanning 

# For the algorithm time duration, average on how many samples
ALGO_DURATION_AVERAGE = 12

# Delay before stopping the algorithm (in seconds)
TIME_OUT = 60