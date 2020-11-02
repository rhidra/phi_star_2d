# Algorithm parameters
H_COST_WEIGHT = 1.7

# Parameters for map generation with Perlin noise
WIDTH = 100
HEIGHT = 100
OBSTACLE_THRESHOLD = .2
OBSTACLE_X_SIZE = 6
OBSTACLE_Y_SIZE = 6
OBSTACLE_X_OFFSET = 0
OBSTACLE_Y_OFFSET = 0

# Real time display of the algorithm
DISPLAY = False
DISPLAY_DELAY = .001
DISPLAY_FREQ = 5
DISPLAY_END = False # Should display for a few seconds at the end of the algorithm (before auto replanning)
WAIT_INPUT = DISPLAY_END and False

# Allow replanning, through user input or programmatically
REPLANNING = True

# For automatic blocked cell generation
BLOCKED_CELLS = 20 # Number of newly blocked cell in one loop 
BLOCKED_CELLS_LOOP = 5 # Number of loops of replanning 

# For the algorithm time duration, average on how many samples
ALGO_DURATION_AVERAGE = 12