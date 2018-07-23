"""
Basic usage example on a ply mesh. Note that this require a closed, manifold
input mesh.
"""
##
import os
import numpy as np
import mayavi.mlab as mlab
import itertools
import utils
import ply
import numpy
import meshcut
from matplotlib import pyplot
##

colors = [
    (0, 1, 1),
    (1, 0, 1),
    (0, 0, 1)
]

#Load and plot the pretty mesh
with open(r"C:\Users\aaron\sfm\tranquilitatis\cross-sections\MTP_V2.ply") as f:
    display_mesh = ply.load_ply(f)
    # Draw the mesh with Mayavi
    utils.trimesh3d(verts=display_mesh[0], faces=display_mesh[1])

#Load the mesh for calculations
with open(r"C:\Users\aaron\sfm\tranquilitatis\cross-sections\MTP_V2_print.ply") as f:
    display_mesh = ply.load_ply(f)
    mesh = meshcut.TriangleMesh(verts=display_mesh[0], tris=display_mesh[1])

plane_orig = (0, 0, 0)
plane_angles = numpy.arange(0, 2*numpy.pi, 0.2*numpy.pi)
plane_normals = [(numpy.cos(angle), numpy.sin(angle), 0) for angle in plane_angles]
slices = []
#create all the cut planes
for plane_normal in plane_normals:
    plane = meshcut.Plane(plane_orig, plane_normal)
    sliced = meshcut.cross_section_mesh(mesh, plane)
    for p, color in zip(sliced, itertools.cycle(colors)):
        p = np.array(p)
        #utils.points3d(np.array(p), point_size=3, color=(1,1,1))
        mlab.plot3d(p[:, 0], p[:, 1], p[:, 2], tube_radius=None,
                    line_width=3.0, color=color)
    # sliced = slice_and_display(plane, expected_n_contours=3)
mlab.show()

pyplot.figure()
#TODO needs rotation before plotting in 2d
[pyplot.plot(seg[:, 0], seg[:, 1], '.') for seg in sliced]
pyplot.show()
slices.append(sliced)


