import numpy as np

def system_edges(obstacles, boids, vicseks):
    """Edge types of the directed graph representing the influences between
    elements of the system.
        |    |Goal|Obst|Boid|Visc|
        |Goal| 0  | 0  | 1  | 5  |
        |Obst| 0  | 0  | 2  | 6  |
        |Boid| 0  | 0  | 3  | 7  |
        |Visc| 0  | 0  | 4  | 8  |
    """
    # If boids == 0, edges would be same as if vicseks were boids
    if boids == 0:
        boids, vicseks = vicseks, boids

    particles = 1 + obstacles + boids + vicseks
    edges = np.zeros((particles, particles), dtype=int)

    up_to_goal = 1
    up_to_obs = up_to_goal + obstacles
    up_to_boids = up_to_obs + boids

    edges[0, up_to_obs:up_to_boids] = 1  # influence from goal to boid.
    edges[up_to_goal:up_to_obs, up_to_obs:up_to_boids] = 2  # influence from obstacle to boid.
    edges[up_to_obs:up_to_boids, up_to_obs:up_to_boids] = 3  # influence from boid to boid.
    edges[up_to_boids:, up_to_obs:up_to_boids] = 4  # influence from vicsek to boid.

    edges[0, up_to_boids:] = 5  # influence from goal to vicsek.
    edges[up_to_goal:up_to_obs, up_to_boids:] = 6  # influence from obstacle to vicsek.
    edges[up_to_obs:up_to_boids, up_to_boids:] = 7  # influence from obstacle to agent.
    edges[up_to_boids:, up_to_boids:] = 8  # influence from viscek to viscek.

    np.fill_diagonal(edges, 0)
    return edges