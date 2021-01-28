#!/usr/bin/env python3

import time
from math import radians, tan

import cartopy.crs as ccrs
import matplotlib.animation as animation
import matplotlib.lines as mlines
import matplotlib.pyplot as plt
import numpy as np
import pymap3d as pm
from ground_visualization.plotters_utils import arc_points_between_vectors, los_to_earth
from numpy.linalg import norm
from scipy.spatial.transform import Rotation as ro

"""
Contains the visualizer class to display in 2D and 3D plots the spacecraft attitude.
"""


class AttitudeVisualizer:
    def __init__(self):
        self._fig = plt.figure()

        self._init_3d_view()
        self._init_map_view()

        self._camera_fov = 14.8  # camera fov in degrees (camera has square resolution so h_fov = v_fov)

    def _init_3d_view(self):
        """
        Initializes the 3D view of the visualizer.
        """
        self._ax3d = self._fig.add_subplot(1, 2, 1, projection='3d')

        self._sc_body_eci = None  # set by _draw_3d_spacecraft()

        self._lims3d = [-6000, 6000]  # 3d axes limites
        self._arrow_style3d = "->"
        self._arrow_scale3d = 15
        self._arrow_length3d = 4000
        self._arrow_weight3d = 1.5
        self._eci_frame_colors = ["red", "green", "blue"]
        self._sc_frame_colors = ["lightcoral", "yellowgreen", "skyblue"]
        self._nadir_vector_color = "hotpink"
        self._angle_to_nadir_color = "orange"
        self._nadir_line_style = "--"
        self._camera_frustum_color = "turquoise"
        self._camera_frustum_scale = 0.5
        self._camera_frustum_linestyle = "--"

    def _init_map_view(self):
        """
        Initializes the map view of the visualizer
        """
        self._map_grid_lines_color = "blue"
        self._map_grid_lines_style = "--"
        self._map_grid_lines_width = 1
        self._map_grid_labels_enabled = True

        # flat map view
        self._nadir_target_marker_style = "+"
        self._camera_pointing_marker_style = "+"

        self._flat_map_proj = ccrs.PlateCarree()
        self._ax_flat_map = self._fig.add_subplot(2, 2, 2, projection=self._flat_map_proj)
        self._ax_flat_map.set_global()
        self._ax_flat_map.coastlines(resolution='110m')
        self._ax_flat_map.gridlines(draw_labels=self._map_grid_labels_enabled, linewidth=self._map_grid_lines_width,
                                    color=self._map_grid_lines_color, alpha=0.3, linestyle=self._map_grid_lines_style)
        self._nadir_target, = self._ax_flat_map.plot(0, 0, markersize=20,
                                                     marker=self._nadir_target_marker_style,
                                                     color=self._nadir_vector_color)
        self._camera_pointing, = self._ax_flat_map.plot(0, 0, markersize=20,
                                                        marker=self._camera_pointing_marker_style,
                                                        color=self._camera_frustum_color)

        # zoomed flat map view
        self._nadir_coverage_marker_style = "o"
        self._camera_coverage_marker_style = "o"
        self._zoom_margin = 5

        self._ax_flat_map_zoom = self._fig.add_subplot(2, 2, 4, projection=self._flat_map_proj)
        self._ax_flat_map_zoom.coastlines(resolution='50m')
        self._nadir_coverage = self._ax_flat_map_zoom.tissot(rad_km=50,
                                                        lats=0, lons=0, n_samples=15,
                                                        facecolor=self._nadir_vector_color,
                                                        edgecolor=self._nadir_vector_color,
                                                        linewidth=0.15, alpha=0.75)
        self._camera_coverage = None

    def _draw_3d_eci_frame(self):
        """
        Draws the ECI coordinates frame in the 3D view.
        """
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
        self._ax3d.plot(x_earth, y_earth, z_earth, marker="o", markersize=10, color="dodgerblue")

    def _draw_3d_spacecraft(self, sat_state):
        """ Draws all the features related to the spacecraft: body frame (x,y,-z), nadir vector, camera frustum.

        Parameters
	    ----------
	    sat_state : SatState
	        The satellite data to use
        """
        # compute S/C body frame in ECI coordinates
        sc_body = np.array([[self._arrow_length3d, 0, 0], [0, self._arrow_length3d, 0], [0, 0, -self._arrow_length3d]])
        q0, q1, q2, q3 = sat_state.quat_attitude
        rotation = ro.from_quat([q1, q2, q3, q0])
        sc_body_eci = rotation.apply(sc_body)  # # SciPy uses the convention ([vector], scalar).
        self._sc_body_eci = sc_body_eci

        self._draw_3d_sc_body(sat_state)
        self._draw_3d_camera_frustum(sat_state, rotation, sc_body)
        self._draw_3d_nadir(sat_state)

        # draw square for satellite
        self._ax3d.plot(sat_state.position.x, sat_state.position.y, sat_state.position.z, marker="s", markersize=4,
                        color="grey")

    def _draw_3d_sc_body(self, sat_state):
        """ Draws the spacecraft body frame (x, y, -z).

        Parameters
        ----------
        sat_state : SatState
            The satellite data to use
        """
        # origin of drawings
        x, y, z = sat_state.position.x, sat_state.position.y, sat_state.position.z

        # draw S/C body frame
        for axis in range(3):
            self._ax3d.arrow3D(x, y, z, self._sc_body_eci[axis][0],
                               self._sc_body_eci[axis][1], self._sc_body_eci[axis][2],
                               color=self._sc_frame_colors[axis],
                               mutation_scale=self._arrow_scale3d,
                               arrowstyle=self._arrow_style3d,
                               lw=self._arrow_weight3d)

    def _draw_3d_camera_frustum(self, sat_state, body_to_eci_ro, sc_body):
        """ Draws the satellite's camera frustum.

        Parameters
        ----------
        sat_state : SatState
            The satellite data to use
        sc_body_eci: np.array(float)
            The satellite body frame axis (x, y, -z) in ECI coordinates frame
        sc_body: np.array(float)
            The satellite body frame axis (x, y, -z)
        """
        # fov in degrees
        fov = self._camera_fov

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

    def _draw_3d_nadir(self, sat_state):
        """ Draws the nadir vector along with an arc indicating the angle between the nadir vector and
        the -z axis of the satellite.

        Parameters
        ----------
        sat_state : SatState
            The satellite data to use
        """
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
                                                nadir * arc_scale, self._sc_body_eci[2] * arc_scale,
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
        """ Updates the 3D view of the visualizer with the given satellite data.

        Parameters
        ----------
        sat_state : SatState
            The satellite data to use
        """
        self._ax3d.clear()

        # set axis label
        self._ax3d.set_xlabel("X (km)")
        self._ax3d.set_ylabel("Y (km)")
        self._ax3d.set_zlabel("Z (km)")

        # set size of plot
        self._ax3d.set_xlim(self._lims3d)
        self._ax3d.set_ylim(self._lims3d)
        self._ax3d.set_zlim(self._lims3d)

        self._draw_3d_eci_frame()
        self._draw_3d_spacecraft(sat_state)

        self._ax3d.set_title("Satellite attitude at {}".format(sat_state.timestamp))

    def _update_flat_map_view(self, sat_state, nadir_ll, minus_z_ll):
        """ Updates the flat map view of the visualizer with the given satellite data.

        Parameters
        ----------
        sat_state : SatState
            The satellite data to use
        nadir_ll : np.array(float)
            latitude and longitude at which the nadir vector points
        minus_z_ll : np.array(float) or None if not pointing at earth
            latitude and longitude at which the satellite body minus_z axis points
        """

        margin = self._zoom_margin
        map_center = nadir_ll
        if minus_z_ll is not None:
            map_center = minus_z_ll

        self._ax_flat_map_zoom.set_extent(
            [map_center[1] - margin, map_center[1] + margin, map_center[0] - margin, map_center[0] + margin],
            crs=self._flat_map_proj)

        # plot nadir target
        self._nadir_target.set_data(nadir_ll[1], nadir_ll[0])

        # plot camera pointing
        if minus_z_ll is not None:
            self._camera_pointing.set_data(minus_z_ll[1], minus_z_ll[0])
        else:
            self._camera_pointing.set_data(200, 100)  # plot outside the map

        camera_coverage_radius = tan(radians(self._camera_fov / 2.0)) * (sat_state.lla[2] / 1000.0)

        # plot nadir coverage
        self._nadir_coverage.remove()
        self._nadir_coverage = self._ax_flat_map_zoom.tissot(rad_km=camera_coverage_radius,
                                                        lats=nadir_ll[0], lons=nadir_ll[1], n_samples=15,
                                                        facecolor=self._nadir_vector_color,
                                                        edgecolor=self._nadir_vector_color,
                                                        linewidth=0.15, alpha=0.75)
        # plot camera coverage
        if self._camera_coverage is not None:
            self._camera_coverage.remove()

        if minus_z_ll is not None:
            self._camera_coverage = self._ax_flat_map_zoom.tissot(rad_km=camera_coverage_radius,
                                                             lats=minus_z_ll[0], lons=minus_z_ll[1], n_samples=15,
                                                             facecolor=self._camera_frustum_color,
                                                             edgecolor=self._camera_frustum_color,
                                                             linewidth=0.15, alpha=0.75)
        else:
            self._camera_coverage = None

    def _update_map_views(self, sat_state):
        """ Updates the map views of the visualizer with the given satellite data.

        Parameters
        ----------
        sat_state : SatState
            The satellite data to use
        """

        sat_pos = np.array([sat_state.position.x, sat_state.position.y, sat_state.position.z]) * 1000

        # calculate where (latitude and longitude) nadir vector points
        nadir_ll = None
        nadir_los = los_to_earth(sat_pos, sat_state.nadir)
        if nadir_los is not None:
            nadir_ll = np.hstack(pm.eci2geodetic(nadir_los[0], nadir_los[1], nadir_los[2], sat_state.timestamp)[:2])

        # calculate where (latitude and longitude) satellite body minus_z axis points
        minus_z_ll = None
        minus_z_normalized = self._sc_body_eci[2] / norm(self._sc_body_eci[2])
        minus_z_los = los_to_earth(sat_pos, minus_z_normalized)
        if minus_z_los is not None:
            minus_z_ll = np.hstack(
                pm.eci2geodetic(minus_z_los[0], minus_z_los[1], minus_z_los[2], sat_state.timestamp)[:2])

        self._update_flat_map_view(sat_state, nadir_ll, minus_z_ll)

    def _update(self, sat_state):
        """ Updates the all the views of the visualizer with the given satellite data.

        Parameters
        ----------
        sat_state : SatState
            The satellite data to use
        """
        self._update_3d_view(sat_state)
        self._update_map_views(sat_state)

    def show(self, sat_state):
        """
        Visualize the provided satellite state.
        """
        self._update(sat_state)
        self._add_legend()

        # show
        plt.show()

    def animate(self, sat_state_generator, interval=50):
        """
        Visualize multiple satellite state. The visualizer updates itself with new satellite state provided by a satellite state generator.
        """

        def on_new_frame(frame_number):
            start = time.time()

            sat_state = next(sat_state_generator)
            if sat_state is not None:
                self._update(sat_state)

            end = time.time()
            frame_time = end - start
            print(frame_time)

        self._animation = animation.FuncAnimation(self._fig, on_new_frame, interval=interval)
        self._add_legend()
        plt.show()

    def _add_3d_view_legend(self):
        """
        Adds the legend for the 3D view of the visualizer.
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
        camera_frustum_legend = mlines.Line2D([], [], color=self._camera_frustum_color,
                                              markersize=15, label='Camera frustum',
                                              linestyle=self._camera_frustum_linestyle)
        nadir_legend = mlines.Line2D([], [], color=self._nadir_vector_color,
                                     markersize=15, label='Nadir vector', linestyle=self._nadir_line_style)
        angle_to_nadir_legend = mlines.Line2D([], [], color=self._angle_to_nadir_color,
                                              markersize=15, label='Angle to nadir', linestyle=self._nadir_line_style)
        self._fig.legend(
            handles=[x_eci_legend, y_eci_legend, z_eci_legend, x_sc_legend, y_sc_legend, z_sc_legend,
                     camera_frustum_legend, nadir_legend, angle_to_nadir_legend], loc="upper left")

    def _add_map_views_legend(self):
        """
        Adds the legend for the map views of the visualizer.
        """
        # flat map view
        nadir_target_legend = mlines.Line2D([], [], color=self._nadir_vector_color,
                                              marker=self._nadir_target_marker_style, linestyle='None',
                                              markersize=8, label='Expected nadir pointing')
        camera_pointing_legend = mlines.Line2D([], [], color=self._camera_frustum_color,
                                               marker=self._camera_pointing_marker_style, linestyle='None',
                                               markersize=8, label='Acutal camera pointing')

        self._fig.legend(handles=[nadir_target_legend, camera_pointing_legend], loc="upper right")

        # zoomed
        nadir_coverage_legend = mlines.Line2D([], [], color=self._nadir_vector_color,
                                              marker=self._nadir_coverage_marker_style, linestyle='None',
                                              markersize=8, label='Expected nadir coverage')
        camera_coverage_legend = mlines.Line2D([], [], color=self._camera_frustum_color,
                                               marker=self._camera_coverage_marker_style, linestyle='None',
                                               markersize=8, label='Actual camera coverage')
        self._fig.legend(handles=[nadir_coverage_legend, camera_coverage_legend], loc="lower right")

    def _add_legend(self):
        """
        Adds the legend for all the views of the visualizer.
        """
        self._add_3d_view_legend()
        self._add_map_views_legend()
