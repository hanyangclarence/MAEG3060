import numpy as np

def generate_trajectory_3d(pos_list, time_list,	frequency):
    
    pos_array = np.array(pos_list)  # shape: (N, 3), N = number of waypoints
    time_array = np.array(time_list)  # shape: (N,)

    if len(pos_array) != len(time_array):
        raise ValueError("pos_list and time_list must have the same length")
    if len(pos_array) < 2:
        raise ValueError("At least two positions are required")
    if not all(time_array[i] < time_array[i + 1] for i in range(len(time_array) - 1)):
        raise ValueError("time_list must be strictly increasing")

    total_duration = time_array[-1] - time_array[0]
    num_steps = int(total_duration * frequency) + 1
    time_steps = np.linspace(time_array[0], time_array[-1], num_steps)

    poses = np.zeros((num_steps, 3))
    velocities = np.zeros((num_steps, 3))
    accelerations = np.zeros((num_steps, 3))

    # dim as each dimension (x, y, z)
    for dim in range(3):
        positions = pos_array[:, dim]  # Shape: (N,)

        # compute cubic spline coefficients for each segment
        N = len(positions) - 1  # number of segments
        h = np.diff(time_array)  # time intervals between intermediate points

        v = np.zeros(N + 1)  # velocities at waypoints
        v[0] = 0.0  # Zero initial velocity
        v[N] = 0.0  # Zero final velocity
        # compute average velocities at waypoints
        for i in range(1, N):
            vel_prev = (positions[i] - positions[i - 1]) / (time_array[i] - time_array[i - 1])
            vel_next = (positions[i + 1] - positions[i]) / (time_array[i + 1] - time_array[i])
            v[i] = (vel_prev + vel_next) / 2

        # compute coefficients for each segment
        a = positions[:-1]  # Position offset at start of segment
        b = v[:-1]  # Velocity at start of segment
        c = np.zeros(N)
        d = np.zeros(N)

        for i in range(N):
            # compute c and d coefficients to ensure position and velocity continuity
            c[i] = (3 * (positions[i + 1] - positions[i]) / h[i] - 2 * v[i] - v[i + 1]) / h[i]
            d[i] = (2 * (positions[i] - positions[i + 1]) / h[i] + v[i] + v[i + 1]) / (h[i] ** 2)

        # for i in range(N):
        #     print(f"Segment {i} (t in [{time_array[i]}, {time_array[i+1]}]):")
        #     print(f"  Pose: p(t) = {a[i]} + {b[i]}(t - {time_array[i]}) + {c[i]}(t - {time_array[i]})^2 + {d[i]}(t - {time_array[i]})^3")
        #     print(f"  Velocity: v(t) = {b[i]} + 2 * {c[i]}(t - {time_array[i]}) + 3 * {d[i]}(t - {time_array[i]})^2")
        #     print(f"  Acceleration: a(t) = 2 * {c[i]} + 6 * {d[i]}(t - {time_array[i]})")

        # interpolate each time step
        for t_idx, t in enumerate(time_steps):
            # get corresponding segment 
            for i in range(N):
                if time_array[i] <= t <= time_array[i + 1] or (i == N - 1 and np.isclose(t, time_array[-1])):
                    s = t - time_array[i]
                    poses[t_idx, dim] = a[i] + b[i] * s + c[i] * s**2 + d[i] * s**3
                    velocities[t_idx, dim] = b[i] + 2 * c[i] * s + 3 * d[i] * s**2
                    accelerations[t_idx, dim] = 2 * c[i] + 6 * d[i] * s
                    break
                
    return time_steps, poses, velocities, accelerations 
