#!/usr/bin/env python3

from math import radians, degrees, acos

# import cartopy.crs as ccrs
import matplotlib.lines as mlines
import matplotlib.pyplot as plt
import numpy as np
from ground_visualization.plotters_utils import arc_points_between_vectors
from scipy.spatial.transform import Rotation as ro

"""
Contains the visualizer class to display in 2D and 3D plots the spacecraft attitude.
"""


class AttitudeVisualizer:
    def __init__(self):
        self._init_3d_view()
        # self._init_map_view()

    def _init_3d_view(self):
        """
        Initializes the 3D view of the visualizer.
        """
        self._fig3d = plt.figure()
        self._ax3d = self._fig3d.add_subplot(111, projection='3d')

        self._lims3d = [-1.25, 1.25]  # 3d axes limites
        self._arrow_style3d = "->"
        self._arrow_scale3d = 15
        self._arrow_length3d = 0.85
        self._arrow_weight3d = 1.5
        self._eci_frame_colors = ["red", "green", "blue"]
        self._sc_frame_colors = ["lightcoral", "yellowgreen", "skyblue"]
        self._nadir_vector_color = "turquoise"
        self._angle_to_nadir_color = "orange"
        self._nadir_line_style = "--"
        self._camera_frustum_color = "hotpink"
        self._camera_frustum_scale = 0.5
        self._camera_frustum_linestyle = "--"

    def _init_map_view(self):
        earth_radius = 6371000.
        position = [520000., 75., 0.]  # altitude (m), lat, long
        radius = degrees(acos(earth_radius / (earth_radius + position[0])))
        print(radius)  # in subtended degrees??

        self._fig_map = plt.figure()

        ortho = ccrs.Orthographic(central_longitude=0.0, central_latitude=60.0)
        ortho.set
        self._orth_ax = self._map_fig.add_subplot(2, 1, 1, projection=ccrs.Orthographic(central_longitude=0.0,
                                                                                        central_latitude=60.0))

        # let's take 1 subtended degree = 112 km on earth surface (*** you set the value as needed ***)
        ax.tissot(rad_km=radius * 112, lons=position[2], lats=position[1], n_samples=64, \
                  facecolor='red', edgecolor='black', linewidth=0.15, alpha=0.3)

        ax.coastlines(linewidth=0.15)
        ax.gridlines(draw_labels=False, linewidth=1, color='blue', alpha=0.3, linestyle='--')

        ax2 = self._map_fig.add_subplot(2, 1, 2, projection=ccrs.PlateCarree())
        ax2.set_extent([-180, 180, -90, 90], ccrs.PlateCarree())
        ax2.coastlines(resolution='110m')
        ax2.plot(0.21653933823108673, -2.6539976596832275, markersize=8, marker='o', color='red')

    def _draw_3d_eci_frame(self):
        """
        Draws the ECI coordinates frame in the 3D view.
        """
        # set axis label
        self._ax3d.set_xlabel("x")
        self._ax3d.set_ylabel("y")
        self._ax3d.set_zlabel("z")

        # Remove ticks and keep grid
        self._ax3d.grid(True)
        self._ax3d.set_xticklabels([])
        self._ax3d.set_yticklabels([])
        self._ax3d.set_zticklabels([])

        # set size of plot
        self._ax3d.set_xlim(self._lims3d)
        self._ax3d.set_ylim(self._lims3d)
        self._ax3d.set_zlim(self._lims3d)

        # draw x, y, z axes
        x_earth, y_earth, z_earth = np.array([0, 0, 0])
        self._ax3d.arrow3D(x_earth, y_earth, z_earth, self._arrow_length3d, 0, 0, color=self._eci_frame_colors[0],
                           mutation_scale=self._arrow_scale3d,
                           arrowstyle=self._arrow_style3d,
                           lw=self._arrow_weight3d)
        self._ax3d.arrow3D(x_earth, y_earth, z_earth, 0, self._arrow_length3d, 0, color=self._eci_frame_colors[1],
                           mutation_scale=self._arrow_scale3d,
                           arrowstyle=self._arrow_style3d,
                           lw=self._arrow_weight3d)
        self._ax3d.arrow3D(x_earth, y_earth, z_earth, 0, 0, self._arrow_length3d, color=self._eci_frame_colors[2],
                           mutation_scale=self._arrow_scale3d,
                           arrowstyle=self._arrow_style3d,
                           lw=self._arrow_weight3d)

        # draw point for earth
        self._ax3d.plot(x_earth, y_earth, z_earth, marker="o", markersize=20, color="dodgerblue")

    def _draw_3d_spacecraft(self, sat_state):
        # compute S/C body frame in ECI coordinates
        sc_body = np.array([[self._arrow_length3d, 0, 0], [0, self._arrow_length3d, 0], [0, 0, -self._arrow_length3d]])
        q0, q1, q2, q3 = sat_state.quat_attitude
        rotation = ro.from_quat([q1, q2, q3, q0])
        sc_body_eci = rotation.apply(sc_body)  # # SciPy uses the convention ([vector], scalar).

        self._draw_3d_sc_body(sat_state, sc_body_eci)
        self._draw_3d_camera_frustum(sat_state, rotation, sc_body)
        self._draw_3d_nadir(sat_state, sc_body_eci)

        # draw square for satellite
        self._ax3d.plot(sat_state.position.x, sat_state.position.y, sat_state.position.z, marker="s", markersize=4,
                        color="grey")

    def _draw_3d_sc_body(self, sat_state, sc_body_eci):
        # origin of drawings
        x, y, z = sat_state.position.x, sat_state.position.y, sat_state.position.z

        # draw S/C body frame
        colors = self._sc_frame_colors
        for axis in range(3):
            self._ax3d.arrow3D(x, y, z, sc_body_eci[axis][0],
                               sc_body_eci[axis][1], sc_body_eci[axis][2], color=colors[axis],
                               mutation_scale=self._arrow_scale3d,
                               arrowstyle=self._arrow_style3d,
                               lw=self._arrow_weight3d)

    def _draw_3d_camera_frustum(self, sat_state, body_to_eci_ro, sc_body):
        # fov in degrees (square resolution, h_fov = v_fov)
        fov = 10

        # origin of drawings
        x, y, z = sat_state.position.x, sat_state.position.y, sat_state.position.z

        # draw frustum vectors
        half_fov = radians(fov / 2.0)
        frustum_rotations = [ro.from_euler('yx', [half_fov, half_fov]), ro.from_euler('yx', [-half_fov, half_fov]),
                             ro.from_euler('yx', [-half_fov, -half_fov]),
                             ro.from_euler('yx', [half_fov, -half_fov])]
        frustum_vectors = [(body_to_eci_ro * r).apply(sc_body[2]) * self._camera_frustum_scale for r in
                           frustum_rotations]

        for i, vector in enumerate(frustum_vectors):
            # oriented sides
            self._ax3d.arrow3D(x, y, z, vector[0],
                               vector[1], vector[2], color=self._camera_frustum_color,
                               mutation_scale=self._arrow_scale3d,
                               arrowstyle="-",
                               linestyle=self._camera_frustum_linestyle,
                               lw=self._arrow_weight3d * self._camera_frustum_scale)
            # flat sides
            if i > 0:
                self._ax3d.plot([frustum_vectors[i - 1][0] + x, vector[0] + x],
                                [frustum_vectors[i - 1][1] + y, vector[1] + y],
                                [frustum_vectors[i - 1][2] + z, vector[2] + z],
                                color=self._camera_frustum_color,
                                linestyle=self._camera_frustum_linestyle,
                                lw=self._arrow_weight3d * self._camera_frustum_scale)
            if i == 3:
                self._ax3d.plot([frustum_vectors[0][0] + x, vector[0] + x],
                                [frustum_vectors[0][1] + y, vector[1] + y],
                                [frustum_vectors[0][2] + z, vector[2] + z],
                                color=self._camera_frustum_color,
                                linestyle=self._camera_frustum_linestyle,
                                lw=self._arrow_weight3d * self._camera_frustum_scale)

    def _draw_3d_nadir(self, sat_state, sc_body_eci):
        # origin of drawings
        x, y, z = sat_state.position.x, sat_state.position.y, sat_state.position.z

        # draw nadir vector
        nadir = np.array(sat_state.nadir)
        nadir *= self._arrow_length3d
        self._ax3d.arrow3D(x, y, z, nadir[0], nadir[1], nadir[2],
                           color=self._nadir_vector_color,
                           mutation_scale=self._arrow_scale3d,
                           arrowstyle=self._arrow_style3d,
                           linestyle=self._nadir_line_style,
                           lw=self._arrow_weight3d)

        # draw arc to show angle between nadir and -z of S/C body frame
        arc_scale = 0.65
        angle = sat_state.angle_to_nadir
        arc_points = arc_points_between_vectors(x, y, z,
                                                nadir * arc_scale, sc_body_eci[2] * arc_scale,
                                                radians(angle),
                                                10)
        self._ax3d.plot(arc_points[:, 0], arc_points[:, 1], arc_points[:, 2], lw=self._arrow_weight3d * 0.75,
                        linestyle=self._nadir_line_style,
                        color=self._angle_to_nadir_color)

        # add text to show angle value
        arc_middle = arc_points[len(arc_points) // 2]
        self._ax3d.text(arc_middle[0], arc_middle[1], arc_middle[2],
                        "{:.1f}°".format(angle),
                        color=self._angle_to_nadir_color, horizontalalignment='center')

    def _update_3d_view(self, sat_state):
        """
        TODO
        """
        self._ax3d.clear()

        self._draw_3d_eci_frame()
        self._draw_3d_spacecraft(sat_state)

        self._ax3d.set_title("Satellite attitude at {}".format(sat_state.timestamp))

    def update(self, sat_state):
        """
        TODO
        """
        self._update_3d_view(sat_state)

    def show(self):
        """
        Shows the different views of the visualizer.
        """
        self._add_legend()
        plt.show()

    def _add_legend(self):
        """
        TODO
        """
        x_eci_legend = mlines.Line2D([], [], color=self._eci_frame_colors[0],
                                     markersize=15, label='ECI x')
        y_eci_legend = mlines.Line2D([], [], color=self._eci_frame_colors[1],
                                     markersize=15, label='ECI y')
        z_eci_legend = mlines.Line2D([], [], color=self._eci_frame_colors[2],
                                     markersize=15, label='ECI z')
        x_sc_legend = mlines.Line2D([], [], color=self._sc_frame_colors[0],
                                    markersize=15, label='SC x')
        y_sc_legend = mlines.Line2D([], [], color=self._sc_frame_colors[1],
                                    markersize=15, label='SC y')
        z_sc_legend = mlines.Line2D([], [], color=self._sc_frame_colors[2],
                                    markersize=15, label='SC -z')
        nadir_legend = mlines.Line2D([], [], color=self._nadir_vector_color,
                                     markersize=15, label='Nadir vector', linestyle=self._nadir_line_style)
        angle_to_nadir_legend = mlines.Line2D([], [], color=self._angle_to_nadir_color,
                                              markersize=15, label='Angle to nadir', linestyle=self._nadir_line_style)
        camera_frustum_legend = mlines.Line2D([], [], color=self._camera_frustum_color,
                                              markersize=15, label='Camera frustum',
                                              linestyle=self._camera_frustum_linestyle)
        self._fig3d.legend(
            handles=[x_eci_legend, y_eci_legend, z_eci_legend, x_sc_legend, y_sc_legend, z_sc_legend, nadir_legend,
                     angle_to_nadir_legend, camera_frustum_legend])
