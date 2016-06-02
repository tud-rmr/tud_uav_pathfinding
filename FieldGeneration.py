__author__ = 'Raul Acuna'
from pylab import zeros, mgrid, array, dot, sqrt, empty_like

# This function calculates for every point inside the CUBE
# a vector that represent a direction of approach to the path
# in the proper direction of the path and a tangent.
# Since it uses for-loops it is SLOWWWWWW......


def calc_vec_field(path_x, path_y, path_z, cube_size, cube_step):
    min_mod_vector = 100
    min_vector = 0
    vector_field = zeros((3, cube_size/cube_step, cube_size/cube_step, cube_size/cube_step))
    xc, yc, zc = mgrid[0:cube_size:cube_step, 0:cube_size:cube_step, 0:cube_size:cube_step]
    for i in range(0, cube_size, cube_step):
        for j in range(0, cube_size, cube_step):
            for k in range(0, cube_size, cube_step):
                for m in range(path_x.shape[0]):
                    vector = array([path_x[m]-i-path_x[m-1], path_y[m]-j-path_y[m-1], path_z[m]-k-path_z[m-1]])
                    modvector = sqrt(dot(vector, vector))
                    if modvector < min_mod_vector:
                        min_mod_vector = modvector
                        min_vector = vector
                ii = i/cube_step
                jj = j/cube_step
                kk = k/cube_step
                vector_field[:, ii, jj, kk] = min_vector
                min_mod_vector = 100
    return vector_field, xc, yc, zc


# This function makes the same calculations as the previous one but
# uses matrix operation optimizations for speed


def calc_vec_field_fast(path_x, path_y, path_z, cube_size, cube_step):
    vector_field = 0
    # Grid for the coordinates of the field (Read about mgrid, similar to matlab)
    xc, yc, zc = mgrid[0:cube_size:cube_step, 0:cube_size:cube_step, 0:cube_size:cube_step]

    for m in range(path_x.shape[0]):

        # First we subtract a point of the path to the entire field
        vector_field_matrix = array([path_x[m]-xc, path_y[m]-yc, path_z[m]-zc])
        
        # Now we obtain the components to calculate the vector module
        v_x = vector_field_matrix[0, :, :, :]
        v_y = vector_field_matrix[1, :, :, :]
        v_z = vector_field_matrix[2, :, :, :]
        v_x2 = v_x*v_x
        v_y2 = v_y*v_y
        v_z2 = v_z*v_z

        # Now we calculate a module vector field
        mod_vec = sqrt(v_x2+v_y2+v_z2)
        
        # Now we obtain the tangent to that point on the route
        tangent = array([(path_x[m]-path_x[m-1]), (path_y[m]-path_y[m-1]), (path_z[m]-path_z[m-1])])

        # And define the tangent vector field
        tan_vec_field = empty_like(vector_field_matrix)

        # We apply a scaling factor to this matrix
        scaling_factor = 10
        tan_vec_field[0, :, :, :] = (tangent[0])*scaling_factor
        tan_vec_field[1, :, :, :] = (tangent[1])*scaling_factor
        tan_vec_field[2, :, :, :] = (tangent[2])*scaling_factor
        vector_field_matrix = vector_field_matrix + tan_vec_field
        
        # Now we compare

        if m == 0:
            # First iteration
            min_mod_vec = mod_vec.copy()
            vector_field = vector_field_matrix.copy()
        else:
            # We check if the module of the vector in any point of the
            # matrix is less than the minimum module.
            # if it is we replace the value.
            # That means that for this particular point in the path there exist a shorter way
            # so we replace that part of the field.
            condition = (mod_vec < min_mod_vec)
            min_mod_vec[condition] = mod_vec[condition]
            vector_field[:, condition] = vector_field_matrix[:, condition]
    return vector_field, xc, yc, zc
