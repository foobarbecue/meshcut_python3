"""
Basic usage example on a ply mesh. Note that this require a closed, manifold
input mesh.
"""
##
import os
import numpy as np
import math
import mayavi.mlab as mlab
import itertools
import utils
import ply
import meshcut
from matplotlib import pyplot
##

colors = [
    (0, 1, 1),
    (1, 0, 1),
    (0, 0, 1)
]

def rotate(x_section, axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    axis = axis/math.sqrt(np.dot(axis, axis))
    a = math.cos(theta/2.0)
    b, c, d = -axis*math.sin(theta/2.0)
    aa, bb, cc, dd = a*a, b*b, c*c, d*d
    bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
    rot_mat = np.array([[aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac)],
                     [2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab)],
                     [2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc]])
    rotated = np.array([np.dot(rot_mat, point) for point in x_section])
    return rotated

class cutplane:
    def __init__(self, angle, mesh, origin = [0, 0, 0], color=(0,0,0)):
        self.angle = angle
        self.mesh = mesh
        self.origin = origin
        self.color = color

        self.plane = meshcut.Plane(origin, self.get_normal())
        self.sliced = meshcut.cross_section_mesh(mesh, self.plane)

    def get_normal(self):
        return (np.cos(self.angle), np.sin(self.angle), 0)

    def plot_3D(self, unrotated = False):
        for seg in self.sliced:
            if unrotated:
                seg = rotate(seg, [0, 0, 1], 2*np.pi - self.angle)
            mlab.plot3d(seg[:, 0], seg[:, 1], seg[:, 2], tube_radius=None,
                    line_width=3.0, color=self.color)

    def plot_2D(self):
        for seg in self.sliced:
            unrotated_seg = rotate(seg, [0, 0, 1], 2*np.pi - self.angle)
            pyplot.plot(unrotated_seg[:, 1], unrotated_seg[:, 2])

#Load and plot the pretty mesh
with open(r"C:\Users\aaron\sfm\tranquilitatis\cross-sections\MTP_V2.ply") as f:
    display_mesh = ply.load_ply(f)
    # Draw the mesh with Mayavi
    utils.trimesh3d(verts=display_mesh[0], faces=display_mesh[1])

#Load the mesh for calculations
with open(r"C:\Users\aaron\sfm\tranquilitatis\cross-sections\MTP_V2_print.ply") as f:
    display_mesh = ply.load_ply(f)
    mesh = meshcut.TriangleMesh(verts=display_mesh[0], tris=display_mesh[1])

#create all the cut planes
xs_angles = np.arange(0, 2*np.pi, (2.0*np.pi)/10)
slices = []
initial_subplot = pyplot.subplot(2, 5, 1)
for n, xs_angle in enumerate(xs_angles):
    newcut = cutplane(angle=xs_angle, mesh=mesh)
    slices.append(newcut)
    pyplot.subplot(2, 5, n+1, sharex=initial_subplot, sharey=initial_subplot)
    newcut.plot_2D()
pyplot.show()

print('end')
#mlab.show()



