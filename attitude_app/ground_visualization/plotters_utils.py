from math import sin

import numpy as np
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d.axes3d import Axes3D
from mpl_toolkits.mplot3d.proj3d import proj_transform

"""
A set of utilities to easily plot in 2D/3D with matplotlib.
"""


class Arrow3D(FancyArrowPatch):
    """
    New artist to use with the Axes3D class to easily draw 3D arrows.
    """

    def __init__(self, x, y, z, dx, dy, dz, *args, **kwargs):
        """ Creates a new Arrow3D artist instance.

        Parameters
        ----------
        x : float
            the arrow x origin
        y : float
            the arrow y origin
        z: float
            the arrow z origin
        dx : float
            the arrow displacement along the x axis
        dy : float
            the arrow displacement along the y axis
        dz : float
            the arrow displacement along the j axis
        """
        super().__init__((0, 0), (0, 0), *args, **kwargs)
        self.set_data(x, y, z, dx, dy, dz)

    def draw(self, renderer):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)

        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), renderer.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        super().draw(renderer)

    def set_data(self, x, y, z, dx, dy, dz):
        self._xyz = (x, y, z)
        self._dxdydz = (dx, dy, dz)


def _arrow3D(ax, x, y, z, dx, dy, dz, *args, **kwargs):
    """ Add an Arrow3D artist to an Axes3D instance.

    Parameters
	----------
	ax : the matplotlib Axes3D
	x : float
	    the arrow x origin
	y : float
	    the arrow y origin
	z: float
	    the arrow z origin
	dx : float
	    the arrow displacement along the x axis
	dy : float
	    the arrow displacement along the y axis
	dz : float
	    the arrow displacement along the j axis
    """
    arrow = Arrow3D(x, y, z, dx, dy, dz, *args, **kwargs)
    ax.add_artist(arrow)

    return arrow


# enhance Axes3D class with arrow3D function
setattr(Axes3D, 'arrow3D', _arrow3D)


def arc_points_between_vectors(x, y, z, v1, v2, angle, nb_points):
    """ Returns points constituting the arc between 2 vectors that both starts at x,y,z

    Parameters
	----------
	x : float
	    x coordinate of the origin of the 2 vectors
	y : float
	    y coordinate of the origin of the 2 vectors
	z : float
	    z coordinate of the origin of the 2 vectors
	v1 : np.array
	    3D vector from which the arc starts
	v2 : np.array
	    3D vector at which the arc stops
	angle : float
	    size of the arc in radians
	nb_points : int
	    number of points to generate in the arc

	Returns
    -------
    np.array: the points
    """
    arc_origin = np.array([x, y, z])
    arc_points = []
    for t in np.linspace(0, 1, nb_points):
        # slerp formula (https://en.wikipedia.org/wiki/Slerp) between v1 vector and v2 vector
        arc_points.append(
            sin((1 - t) * angle) / sin(angle) * v1 + sin(t * angle) / sin(angle) * v2 + arc_origin)

    return np.array(arc_points)


def los_to_earth(position, pointing):
    """ Finds the intersection of a pointing vector u and starting point s with the WGS-84 geoid
    (thanks to https://stephenhartzell.medium.com/satellite-line-of-sight-intersection-with-earth-d786b4a6a9b6)

    Parameters
	----------
        position (np.array): length 3 array defining the starting point location(s) in meters
        pointing (np.array): length 3 array defining the pointing vector(s) (must be a unit vector)
    Returns
    -------
        np.array: the point(s) of intersection with the surface of the Earth in meters or None if the vector doesn't
        point to earth
    """

    a = 6371008.7714
    b = 6371008.7714
    c = 6356752.314245
    x = position[0]
    y = position[1]
    z = position[2]
    u = pointing[0]
    v = pointing[1]
    w = pointing[2]

    value = -a ** 2 * b ** 2 * w * z - a ** 2 * c ** 2 * v * y - b ** 2 * c ** 2 * u * x
    radical = a ** 2 * b ** 2 * w ** 2 + a ** 2 * c ** 2 * v ** 2 - a ** 2 * v ** 2 * z ** 2 + 2 * a ** 2 * v * w * y * z - a ** 2 * w ** 2 * y ** 2 + b ** 2 * c ** 2 * u ** 2 - b ** 2 * u ** 2 * z ** 2 + 2 * b ** 2 * u * w * x * z - b ** 2 * w ** 2 * x ** 2 - c ** 2 * u ** 2 * y ** 2 + 2 * c ** 2 * u * v * x * y - c ** 2 * v ** 2 * x ** 2
    magnitude = a ** 2 * b ** 2 * w ** 2 + a ** 2 * c ** 2 * v ** 2 + b ** 2 * c ** 2 * u ** 2

    if radical < 0:
        return None
    d = (value - a * b * c * np.sqrt(radical)) / magnitude

    if d < 0:
        return None

    return np.array([
        x + d * u,
        y + d * v,
        z + d * w,
    ])
