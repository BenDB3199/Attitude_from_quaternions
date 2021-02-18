import sys
from datetime import datetime
from math import radians, tan, degrees, acos

import cartopy.crs as ccrs
import ephem
import matplotlib.animation as animation
import matplotlib.lines as mlines
import matplotlib.pyplot as plt
import numpy as np
import pymap3d as pm
from ground_visualization.plotters_utils import arc_points_between_vectors, los_to_earth
from numpy import dot
from numpy.linalg import norm
from scipy.spatial.transform import Rotation as ro

"""
Contains the visualizer class to display in 2D and 3D plots the spacecraft attitude.
"""


class AttitudeVisualizer:
    def __init__(self, adcs="IADCS"):
        self._fig = plt.figure(figsize=(14, 7))
        self._fig.canvas.set_window_title("OPS-SAT attitude visualizer")

        self._init_3d_view()
        self._init_map_view()
        self._init_modes_legend()

        self._camera_fov = 14.8  # camera fov in degrees (camera has square resolution so h_fov = v_fov)
        self._adcs = adcs
        self._adcs_color = "magenta" if self._adcs == "CADCS" else "chocolate"

    def _init_3d_view(self):
        """
        Initializes the 3D view of the visualizer.
        """
        self._sc_body_eci = None  # set by _draw_3d_spacecraft()

        # drawings properties
        self._lims3d = [-6500, 6500]
        self._arrow_length3d = 4000
        self._arrow_style3d = "->"
        self._arrow_scale3d = 15
        self._arrow_weight3d = 1.5
        self._earth_color = "dodgerblue"
        self._earth_center_marker = "o"
        self._earth_surface_weight3d = 0.5
        self._eci_frame_colors = ["red", "green", "blue"]
        self._sc_frame_colors = ["lightcoral", "yellowgreen", "skyblue"]
        self._nadir_vector_color = "hotpink"
        self._angle_to_nadir_color = "orange"
        self._nadir_line_style = "--"
        self._timestamp_color = "black"
        self._camera_frustum_color = "turquoise"
        self._camera_frustum_scale = 0.5
        self._camera_frustum_linestyle = "--"

        self._ax3d = self._fig.add_subplot(1, 2, 1, projection='3d')

        # set axis label
        self._ax3d.set_xlabel("X (km)")
        self._ax3d.set_ylabel("Y (km)")
        self._ax3d.set_zlabel("Z (km)")

        # set size of plot
        x_y_scale = 0.9
        self._ax3d.set_xlim([self._lims3d[0] * x_y_scale, self._lims3d[1] * x_y_scale])
        self._ax3d.set_ylim([self._lims3d[0] * x_y_scale, self._lims3d[1] * x_y_scale])
        self._ax3d.set_zlim(self._lims3d)

        self._draw_3d_eci_frame()

        # s/c body
        zero = np.array([0])
        self._sc_body_3d, = self._ax3d.plot(zero, zero, zero, marker="s", markersize=4, color="grey")

        # draw S/C body frame
        self._sc_body_arrow3d = []
        for axis in range(3):
            self._sc_body_arrow3d.append(self._ax3d.arrow3D(0, 0, 0, 0, 0, 0,
                                                            color=self._sc_frame_colors[axis],
                                                            mutation_scale=self._arrow_scale3d,
                                                            arrowstyle=self._arrow_style3d,
                                                            lw=self._arrow_weight3d))

        # nadir vector
        self._nadir_arrow = self._ax3d.arrow3D(0, 0, 0, 1, 1, 1,
                                               color=self._nadir_vector_color,
                                               mutation_scale=self._arrow_scale3d,
                                               arrowstyle=self._arrow_style3d,
                                               linestyle=self._nadir_line_style,
                                               lw=self._arrow_weight3d)
        # angle to nadir arc
        self._arc3d, = self._ax3d.plot(zero, zero, zero,
                                       lw=self._arrow_weight3d * 0.75,
                                       linestyle=self._nadir_line_style,
                                       color=self._angle_to_nadir_color)
        # angle to nadir text
        self._nadir_angle_text = self._ax3d.text2D(0.95, 0.95,
                                                   "{:.1f}°".format(0), transform=self._ax3d.transAxes,
                                                   color=self._angle_to_nadir_color, horizontalalignment='right')

        # timestamp text
        self._timestamp_text = self._ax3d.text2D(0.1, 0.95, "", transform=self._ax3d.transAxes,
                                                 color=self._timestamp_color, horizontalalignment='left')

        # camera frustum
        self._camera_frustum_arrow3d = []
        self._camera_frustum_plot = []
        for i in range(4):
            # oriented sides
            self._camera_frustum_arrow3d.append(self._ax3d.arrow3D(0, 0, 0, 0, 0, 0,
                                                                   color=self._camera_frustum_color,
                                                                   mutation_scale=self._arrow_scale3d,
                                                                   arrowstyle="-",
                                                                   linestyle=self._camera_frustum_linestyle,
                                                                   lw=self._arrow_weight3d * self._camera_frustum_scale))
            # flat sides
            if i > 0:
                p, = self._ax3d.plot([0, 0], [0, 0], [0, 0],
                                     color=self._camera_frustum_color,
                                     linestyle=self._camera_frustum_linestyle,
                                     lw=self._arrow_weight3d * self._camera_frustum_scale)
                self._camera_frustum_plot.append(p)
            if i == 3:
                p, = self._ax3d.plot([0, 0], [0, 0], [0, 0],
                                     color=self._camera_frustum_color,
                                     linestyle=self._camera_frustum_linestyle,
                                     lw=self._arrow_weight3d * self._camera_frustum_scale)
                self._camera_frustum_plot.append(p)

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
        self._ax_flat_map.coastlines(resolution='50m')
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
        self._zoom_margin = 7  # in coordinates degrees

        self._ax_flat_map_zoom = self._fig.add_subplot(2, 2, 4, projection=self._flat_map_proj)
        self._ax_flat_map_zoom.coastlines(resolution='10m')
        self._nadir_coverage = self._ax_flat_map_zoom.tissot(rad_km=50,
                                                             lats=0, lons=0, n_samples=15,
                                                             facecolor=self._nadir_vector_color,
                                                             edgecolor=self._nadir_vector_color,
                                                             linewidth=0.15, alpha=0.75)
        self._camera_coverage = None

    def _init_modes_legend(self):
        """
        Initializes data related to the modes (LIVE/PLAYBACK) legend.
        """
        # run modes data
        self._live_mode_color = "chartreuse"
        self._live_mode_marker_style = ">"
        self._live_mode_label = "LIVE"

        self._playback_mode_color = "gold"
        self._playback_mode_marker_style = "<"
        self._playback_mode_label = "PLAYBACK"

        self._paused_color = "red"
        self._paused_marker_style = "s"
        self._paused_label = "PAUSED"

        # default mode is LIVE
        self._current_mode_color = self._live_mode_color
        self._current_mode_marker_style = self._live_mode_marker_style
        self._current_mode_label = self._live_mode_label

        self._mode_legend_handle = None

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

        # draw earth center
        self._ax3d.plot(np.array([x_earth]), np.array([y_earth]), np.array([z_earth]),
                        marker=self._earth_center_marker, markersize=5, color=self._earth_color)

        # draw earth sphere
        radius = ephem.earth_radius / 1000
        u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:10j]
        x = radius * np.cos(u) * np.sin(v)
        y = radius * np.sin(u) * np.sin(v)
        z = radius * np.cos(v)
        self._ax3d.plot_wireframe(x, y, z, color=self._earth_color, lw=self._earth_surface_weight3d, alpha=0.25)

    def _draw_3d_spacecraft(self, sat_state):
        """ Draws all the features related to the spacecraft: body frame (x,y,-z), nadir vector, camera frustum.

        Parameters
	    ----------
	    sat_state : SatState
	        The satellite data to use
        """
        # default mode is IADCS
        q0, q1, q2, q3 = sat_state.quat_attitude
        eci_to_body_ro = ro.from_quat([q1, q2, q3, q0])  # SciPy uses the convention ([vector], scalar)
        rotation = eci_to_body_ro

        # if CADCS, inverse the rotation
        if self._adcs == "CADCS":
            body_to_eci_ro = eci_to_body_ro.inv()
            rotation = body_to_eci_ro

        # compute S/C body frame in ECI coordinates
        sc_body = np.array([[1, 0, 0], [0, 1, 0], [0, 0, -1]]) * self._arrow_length3d
        sc_body_eci = rotation.apply(sc_body)
        self._sc_body_eci = sc_body_eci

        # draw S/C related artefacts
        self._draw_3d_sc_body(sat_state)
        self._draw_3d_camera_frustum(sat_state, rotation, sc_body)
        self._draw_3d_nadir(sat_state)

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
            self._sc_body_arrow3d[axis].set_data(x, y, z, self._sc_body_eci[axis][0],
                                                 self._sc_body_eci[axis][1],
                                                 self._sc_body_eci[axis][2])

        # draw square for satellite
        self._sc_body_3d.set_data(np.array([x]), np.array([y]))
        self._sc_body_3d.set_3d_properties(np.array(z))

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
            self._camera_frustum_arrow3d[i].set_data(x, y, z, vector[0], vector[1], vector[2])

            # flat sides
            if i > 0:
                self._camera_frustum_plot[i - 1].set_data(np.array([frustum_vectors[i - 1][0] + x, vector[0] + x]),
                                                          np.array([frustum_vectors[i - 1][1] + y, vector[1] + y]))
                self._camera_frustum_plot[i - 1].set_3d_properties(
                    np.array([frustum_vectors[i - 1][2] + z, vector[2] + z]))

            if i == 3:
                self._camera_frustum_plot[i].set_data(np.array([frustum_vectors[0][0] + x, vector[0] + x]),
                                                      np.array([frustum_vectors[0][1] + y, vector[1] + y]))
                self._camera_frustum_plot[i].set_3d_properties(np.array([frustum_vectors[0][2] + z, vector[2] + z]))

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

        nadir = np.array(sat_state.nadir)
        eci_z = self._sc_body_eci[2] / norm(self._sc_body_eci[2])
        nadir_angle = degrees(acos(dot(eci_z, nadir)))
        # print(sat_state.angle_to_nadir)

        # draw nadir vector
        nadir *= self._arrow_length3d
        self._nadir_arrow.set_data(x, y, z, nadir[0], nadir[1], nadir[2])

        # draw arc to show angle between nadir and -z of S/C body frame
        arc_scale = 0.65
        arc_points = arc_points_between_vectors(x, y, z,
                                                nadir * arc_scale, self._sc_body_eci[2] * arc_scale,
                                                radians(nadir_angle),
                                                10)
        self._arc3d.set_data(arc_points[:, 0], arc_points[:, 1])
        self._arc3d.set_3d_properties(arc_points[:, 2])

        # add text to show nadir_angle value
        self._nadir_angle_text.set_text("{:.1f}°".format(nadir_angle))

    def _update_3d_view(self, sat_state):
        """ Updates the 3D view of the visualizer with the given satellite data.

        Parameters
        ----------
        sat_state : SatState
            The satellite data to use
        """
        self._draw_3d_spacecraft(sat_state)

        self._timestamp_text.set_text("Attitude at {}".format(sat_state.timestamp))

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

        # adjust flat map zoom location
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
        """ Updates all the views of the visualizer with the given satellite data.

        Parameters
        ----------
        sat_state : SatState
            The satellite data to use
        """
        # when provided quaternion was invalid, the computed SatState == -1
        # let's protect from updating in that case
        if sat_state == -1:
            print("AttitudeVisualizer._update(): invalid SatState provided, not updating")
            return

        self._update_3d_view(sat_state)
        self._update_map_views(sat_state)

    def update(self, sat_state):
        """ Updates all the views of the visualizer with the given satellite data and re-draws them.

        Parameters
        ----------
        sat_state : SatState
            The satellite data to use
        """

        self._update(sat_state)
        plt.draw()

    def show(self):
        """
        Visualize the current satellite state provided either by update() or animate().
        """
        self._add_legend()

        # show
        plt.show()

    def animate(self, sat_state_generator, interval=50):
        """ Visualize multiple satellite states frame by frame. The visualizer updates itself with new satellite state
        provided by the satellite state generator, there is no need to call the update() method.

        Parameters
        ----------
        sat_state_generator : generator of SatState
            generator returning the next satellite state to use for the next frame
        interval : int
            interval between frames in milliseconds
        """
        self._animation_interval = interval

        # update legend to PLAYBACK mode
        self._update_modes_legend(self._playback_mode_color,
                                  self._playback_mode_marker_style,
                                  self._playback_mode_label)

        # catch keyboard key press event
        def key_press_event(event):
            if event.key == 'p':
                self._pause_animation()

        self._fig.canvas.mpl_connect('key_press_event', key_press_event)

        # define new frame function
        def on_new_frame(sat_state):
            # if end of generator is reached, we exit
            if sat_state is None:
                exit(0)
            # otherwise update frame with latest data
            self._update(sat_state)

        self._is_animation_running = True
        self._animation = animation.FuncAnimation(self._fig, on_new_frame, sat_state_generator,
                                                  save_count=sys.maxsize, interval=self._animation_interval)

    def export_mp4(self):
        """
        Exports the animation frames in an mp4 file instead of showing them.
        If this is called, calling show() won't do anything.
        """
        self._add_legend()

        # prepare file name
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        video_file_name = 'attitude-{}.mp4'.format(timestamp)
        print("Exporting {}".format(video_file_name))

        # save the animation
        try:
            Writer = animation.writers['ffmpeg']
            writer = Writer(fps=int(1000 / self._animation_interval), metadata=dict(artist='Me'), bitrate=1800)
            self._animation.save(video_file_name, writer=writer)
        except Exception as e:
            print(e)

    def _pause_animation(self):
        """
        Pauses or start the visualizer animation.
        """
        if self._is_animation_running:
            self._update_modes_legend(self._paused_color, self._paused_marker_style, self._paused_label)
            self._animation.event_source.stop()
            self._is_animation_running = False
        else:
            self._update_modes_legend(self._playback_mode_color,
                                      self._playback_mode_marker_style,
                                      self._playback_mode_label)
            self._animation.event_source.start()
            self._is_animation_running = True

    def _add_3d_view_legend(self):
        """
        Adds the legend for the 3D view of the visualizer.
        """
        earth_center_legend = mlines.Line2D([], [], color=self._earth_color, marker=self._earth_center_marker,
                                            linestyle='None', label='Earth center')
        earth_surface_legend = mlines.Line2D([], [], color=self._earth_color,
                                             linewidth=self._earth_surface_weight3d, label='Earth surface')
        x_eci_legend = mlines.Line2D([], [], color=self._eci_frame_colors[0],
                                     markersize=15, label='ECI X')
        y_eci_legend = mlines.Line2D([], [], color=self._eci_frame_colors[1],
                                     markersize=15, label='ECI Y')
        z_eci_legend = mlines.Line2D([], [], color=self._eci_frame_colors[2],
                                     markersize=15, label='ECI Z')
        x_sc_legend = mlines.Line2D([], [], color=self._sc_frame_colors[0],
                                    markersize=15, label='SC body X')
        y_sc_legend = mlines.Line2D([], [], color=self._sc_frame_colors[1],
                                    markersize=15, label='SC body Y')
        z_sc_legend = mlines.Line2D([], [], color=self._sc_frame_colors[2],
                                    markersize=15, label='SC body -Z')
        camera_frustum_legend = mlines.Line2D([], [], color=self._camera_frustum_color,
                                              markersize=15, label='Camera frustum',
                                              linestyle=self._camera_frustum_linestyle)
        nadir_legend = mlines.Line2D([], [], color=self._nadir_vector_color,
                                     markersize=15, label='Nadir vector', linestyle=self._nadir_line_style)
        angle_to_nadir_legend = mlines.Line2D([], [], color=self._angle_to_nadir_color,
                                              markersize=15, label='Angle to nadir', linestyle=self._nadir_line_style)
        self._fig.legend(
            handles=[earth_center_legend, earth_surface_legend, x_eci_legend, y_eci_legend, z_eci_legend, x_sc_legend,
                     y_sc_legend, z_sc_legend,
                     camera_frustum_legend, nadir_legend, angle_to_nadir_legend], loc="upper left")

    def _add_map_views_legend(self):
        """
        Adds the legend for the map views of the visualizer.
        """
        # flat map view
        nadir_target_legend = mlines.Line2D([], [], color=self._nadir_vector_color,
                                            marker=self._nadir_target_marker_style, linestyle='None',
                                            markersize=8, label='Nadir pointing')
        camera_pointing_legend = mlines.Line2D([], [], color=self._camera_frustum_color,
                                               marker=self._camera_pointing_marker_style, linestyle='None',
                                               markersize=8, label='Camera pointing')

        self._fig.legend(handles=[nadir_target_legend, camera_pointing_legend], loc="upper right")

        # zoomed
        nadir_coverage_legend = mlines.Line2D([], [], color=self._nadir_vector_color,
                                              marker=self._nadir_coverage_marker_style, linestyle='None',
                                              markersize=8, label='Nadir coverage')
        camera_coverage_legend = mlines.Line2D([], [], color=self._camera_frustum_color,
                                               marker=self._camera_coverage_marker_style, linestyle='None',
                                               markersize=8, label='Camera coverage')
        self._fig.legend(handles=[nadir_coverage_legend, camera_coverage_legend], loc="lower right")

    def _add_adcs_legend(self):
        """
        Adds the legend showing from which ADCS the quaternion messages should come from.
        """
        adcs_legend = mlines.Line2D([], [], color=self._adcs_color,
                                    marker='s', linestyle='None',
                                    markersize=8, label=self._adcs + " MODE")
        self._fig.legend(handles=[adcs_legend], loc="lower center")

    def _add_modes_legend(self):
        """
        Adds legend for the run modes (LIVE/PLAYBACK).
        """
        if self._mode_legend_handle is not None:
            self._mode_legend_handle.remove()

        self._mode_legend = mlines.Line2D([], [], color=self._current_mode_color,
                                          marker=self._current_mode_marker_style, linestyle='None',
                                          markersize=8, label=self._current_mode_label)

        self._mode_legend_handle = self._fig.legend(handles=[self._mode_legend], loc="upper center")

    def _add_legend(self):
        """
        Adds the legend for all the views of the visualizer.
        """
        self._add_3d_view_legend()
        self._add_map_views_legend()
        self._add_adcs_legend()
        self._add_modes_legend()

    def _update_modes_legend(self, color, marker_style, label):
        """
        Updates the modes legend.
        """
        self._current_mode_color = color
        self._current_mode_marker_style = marker_style
        self._current_mode_label = label

        self._add_modes_legend()
        plt.draw()
