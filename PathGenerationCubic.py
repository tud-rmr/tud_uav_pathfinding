__author__ = 'Raul Acuna'
from pylab import zeros, array, sqrt, vstack, dot, arange, linalg, hstack


def path_3d(pos, loop):
    # INPUTS
    # pos: A 3xn python array with the required points of the route
    # loop: A boolean, true means that the path is cyclic i.e a loop.
    # OUPUTS
    # X, Y, Z, Tt
    # X, Y, Z: Each one a vector representing the coordinates of every point
    # in the calculated path
    # Tt: A vector with the time information associated with each point
    n_points = pos.shape[0]
    path_x = 0
    path_y = 0
    path_z = 0
    path_t = 0
    # Vector calculation for the path
    if loop:
        # If the path is a loop
        # Start and end is the same
        vectors = zeros((n_points + 1, 3))
        for i in range(n_points):
            if i < n_points - 1:
                # All the internal points
                vector1 = pos[i, :] - pos[i - 1, :]
                vector2 = pos[i + 1, :] - pos[i, :]
                v_result = vector1 + vector2
                vectors[i, :] = v_result / sqrt(dot(v_result, v_result))
            else:
                # Careful with the last one
                vector1 = pos[i, :] - pos[i - 1, :]
                vector2 = pos[0, :] - pos[i, :]
                v_result = vector1 + vector2
                vectors[i, :] = v_result / sqrt(dot(v_result, v_result))
        # We add an additional point to cloe the loop
        vectors[n_points, :] = vectors[0, :]
        pos = vstack((pos, pos[0]))
        n_points += 1
    else:
        # If points don't form a loop
        vectors = zeros((n_points, 3))
        for i in range(n_points):
            if (i > 0) and (i < n_points - 1):
                # All the internal points
                vector1 = pos[i, :] - pos[i - 1, :]
                vector2 = pos[i + 1, :] - pos[i, :]
                v_result = vector1 + vector2
                vectors[i, :] = v_result / sqrt(dot(v_result, v_result))
            else:
                vectors[0, :] = zeros(3)
                vectors[n_points - 1, :] = zeros(3)

    to = 1
    t1 = 10
    time_step = 0.05

    for i in range(n_points - 1):
        # Initial position and velocities
        xi = pos[i, 0]
        dxi = vectors[i, 0]
        yi = pos[i, 1]
        dyi = vectors[i, 1]
        zi = pos[i, 2]
        dzi = vectors[i, 2]
        # Final position and velocities
        xf = pos[i + 1, 0]
        dxf = vectors[i + 1, 0]
        yf = pos[i + 1, 1]
        dyf = vectors[i + 1, 1]
        zf = pos[i + 1, 2]
        dzf = vectors[i + 1, 2]
        # time vector
        t = arange(to, t1, time_step)
        # Matrix for the cubic polynomial calculation
        m = array([[1, to, to ** 2, to ** 3],
                   [0, 1, 2 * to, 3 * to ** 2],
                   [1, t1, t1 ** 2, t1 ** 3],
                   [0, 1, 2 * t1, 3 * t1 ** 2]])

        m_a = dot(linalg.inv(m), array([[xi], [dxi], [xf], [dxf]]))
        m_b = dot(linalg.inv(m), array([[yi], [dyi], [yf], [dyf]]))
        m_c = dot(linalg.inv(m), array([[zi], [dzi], [zf], [dzf]]))

        x_temp = m_a[0] + m_a[1] * t + m_a[2] * t ** 2 + m_a[3] * t ** 3
        y_temp = m_b[0] + m_b[1] * t + m_b[2] * t ** 2 + m_b[3] * t ** 3
        z_temp = m_c[0] + m_c[1] * t + m_c[2] * t ** 2 + m_c[3] * t ** 3

        if i == 0:
            path_x = x_temp
            path_y = y_temp
            path_z = z_temp
            path_t = t
        else:
            path_x = hstack((path_x, x_temp))
            path_y = hstack((path_y, y_temp))
            path_z = hstack((path_z, z_temp))
            path_t = hstack((path_t, t))
        to += 10
        t1 += 10
    return path_x, path_y, path_z, path_t
