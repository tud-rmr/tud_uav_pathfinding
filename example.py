__author__ = 'Raul Acuna'
from PathGenerationCubic import path_3d
from FieldGeneration import calc_vec_field_fast
from mayavi import mlab
from pylab import array

# We first define the main 3D points of the route
# For closed loop trajectories: loop=True

Pos0 = array([4, 8, 3])
Pos1 = array([14, 12, 17])
Pos2 = array([14, 4, 17])
Loop = True
Pos = array([Pos0, Pos1, Pos2])

# We obtain a path from a set of discrete points using
# Cubic splines

X, Y, Z, Tt = path_3d(Pos, Loop)

# Now we calculate the vector field using the provided functions

vector_field_3D, Xc, Yc, Zc = calc_vec_field_fast(X, Y, Z, 20, 1)


# We plot
mlab.figure(size=(800, 600))
mlab.flow(Xc, Yc, Zc, vector_field_3D[0], vector_field_3D[1], vector_field_3D[2], linetype='tube', seedtype='plane')
mlab.pipeline.vector_field(Xc, Yc, Zc, vector_field_3D[0], vector_field_3D[1], vector_field_3D[2], name='vector field')
mlab.points3d(X, Y, Z, Tt)
mlab.quiver3d(Xc, Yc, Zc, vector_field_3D[0], vector_field_3D[1], vector_field_3D[2])
mlab.show()
