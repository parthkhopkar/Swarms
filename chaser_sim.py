import os
import time
import argparse
import numpy as np
import particle


def create_chasers(n):
    """
    Create n particle chasers. 
    Each particle chases the previous one in the list of particles.
    """
    prev = None
    particles = []
    for _ in range(n):
        r = 20
        theta = np.random.rand() * 2 * np.pi

        x, y = r * np.cos(theta), r * np.sin(theta)

        v = np.random.uniform(-2, 2, 2)

        p = particle.ParticleChaser((x, y), v, max_v=10, max_a=10)
        p.target = prev
        particles.append(p)

        prev = p

    if particles:
        particles[0].target = prev

    return particles


def chasers_edges(n):
    """
    Adjacency matrix of the chaser swarm defined in function `create_chasers(n)`.
    A 1 at Row i, Column j means Particle i influences Particle j. No influence
    is represented by 0.
    """
    matrix = np.zeros((n, n), dtype=int)
    for i in range(n):
        matrix[i, (i+1) % n] = 1

    return matrix


def simulation(n):
    particles = create_chasers(ARGS.num_particles)

    position_data = []
    velocity_data = []

    for _ in range(ARGS.steps):
        step_position = []
        step_velocity = []
        for p in particles:
            step_position.append(p.position.copy())
            step_velocity.append(p.velocity.copy())

            p.move(ARGS.dt)

        position_data.append(step_position)
        velocity_data.append(step_velocity)

    return position_data, velocity_data


def main():
    if not os.path.exists(ARGS.save_dir):
        os.makedirs(ARGS.save_dir)

    position_data_all = []
    velocity_data_all = []

    prev_time = time.time()
    for i in range(ARGS.instances):
        if i % 1000 == 0:
            print('Simulation {}/{}... {:.1f}s'.format(i,
                                                       ARGS.instances, time.time()-prev_time))
            prev_time = time.time()

        position_data, velocity_data = simulation(ARGS.num_particles)

        position_data_all.append(position_data)
        velocity_data_all.append(velocity_data)

    print('Simulations {0}/{0} completed.'.format(ARGS.instances))

    np.save(os.path.join(ARGS.save_dir, ARGS.prefix+'_position.npy'), position_data_all)
    np.save(os.path.join(ARGS.save_dir, ARGS.prefix+'_velocity.npy'), velocity_data_all)

    if ARGS.save_edges:
        edges_all = np.tile(chasers_edges(ARGS.num_particles), (ARGS.instances, 1, 1))
        np.save(os.path.join(os.path.expanduser(ARGS.save_dir), ARGS.prefix+'_edge.npy'), edges_all)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--num-particles', '-n', type=int, default=5,
                        help='number of particles')
    parser.add_argument('--instances', type=int, default=1000,
                        help='number of instances to run')
    parser.add_argument('--steps', type=int, default=50,
                        help='number of time steps per simulation')
    parser.add_argument('--dt', type=float, default=0.3,
                        help='unit time step')
    parser.add_argument('--save-dir', type=str,
                        help='name of the save directory')
    parser.add_argument('--prefix', type=str, default='',
                        help='prefix for save files')
    parser.add_argument('--save-edges', action='store_true', default=False,
                        help='turn on to save edges')
    ARGS = parser.parse_args()

    main()
