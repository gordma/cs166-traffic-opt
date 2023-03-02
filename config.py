



cell_size = 4 #m / cell
kmh_to_ms_conversion = 1/3.6

prob_slow = 0.2 #Default Slow Probability
slow_down_distance = 16 #meters
slow_down_cells = int(slow_down_distance / cell_size)
maximum_wait_time = 60
eps = 10e-5

data_collection_time = 900 #start collecting data after x timesteps

