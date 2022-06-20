import numpy as np

def solve_tsp_nearest_neighbor(distance_matrix):
    path = [0]
    cost = 0
    N = distance_matrix.shape[0]
    mask = np.ones(N, dtype=bool) 
    mask[0] = False

    for i in range(N-1):
        last = path[-1]
        next_ind = np.argmin(distance_matrix[last][mask]) # find minimum of remaining locations
        next_loc = np.arange(N)[mask][next_ind] # convert to original location
        path.append(next_loc)
        mask[next_loc] = False
        cost += distance_matrix[last, next_loc]
        if(i == N-2):
            cost += distance_matrix[next_loc, 0]

    return path, cost