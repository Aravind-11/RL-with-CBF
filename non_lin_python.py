import numpy as np

def non_lin(u, x0, n):
    # Non-linear constraints
    dt = 0.1
    alpha1 = 5
    alpha2 = 5
    Ds = 5
    eps = 1e-5

    x1 = x0[0]
    y1 = x0[1]
    vx1 = x0[2]
    vy1 = x0[3]

    x2 = x0[4]
    y2 = x0[5]
    vx2 = x0[6]
    vy2 = x0[7]

    c = []

    for i in range(n):
        # Extract control inputs for each agent at time step i
        u1 = [u[4 * i - 3], u[4 * i - 2]]
        u2 = [u[4 * i - 1], u[4 * i]]

        # Update velocities of each agent using control inputs
        vx1 = vx1 + u1[0] * dt
        vy1 = vy1 + u1[1] * dt
        vx2 = vx2 + u2[0] * dt
        vy2 = vy2 + u2[1] * dt

        # Update positions of each agent using their velocities
        x1 = x1 + vx1 * dt
        y1 = y1 + vy1 * dt
        x2 = x2 + vx2 * dt
        y2 = y2 + vy2 * dt

        # Compute difference in positions and velocities between agents
        delta_p = np.array([x1, y1]) - np.array([x2, y2])
        delta_v = np.array([vx1, vy1]) - np.array([vx2, vy2])
        norm_p = np.linalg.norm(delta_p)

        # Compute value of a12
        a12 = (np.sqrt(2 * (alpha1 + alpha2) * (norm_p - Ds)) + (np.dot(delta_p, delta_v) / (norm_p)))
        # a12 = (np.sqrt(2 * (alpha1 + alpha2) * (norm_p - Ds) * (norm_p - Ds)) + (np.dot(delta_p, delta_v) / (norm_p)))

        # Compute value of h12
        h12 = a12 - Ds

        # Compute value of gamma
        gamma = 1

        # Compute value of b12
        b12 = (norm_p * gamma * (h12 ** 5) + 0.5 * (2 * (alpha1 + alpha2) * np.dot(delta_p, delta_v)) / (np.sqrt(2 * (alpha1 + alpha2) * (norm_p - Ds)))
              + np.dot(delta_v, delta_v) - ((np.dot(delta_p, delta_v) ** 2) / (norm_p ** 2 + eps
