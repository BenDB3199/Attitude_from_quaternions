#!/usr/bin/env python3

import matplotlib.lines as mlines
import matplotlib.pyplot as plt
import numpy as np

"""
Contains the visualizer class to display in 2D and 3D plots the spacecraft attitude.
"""


class AttitudeVisualizer:
    def __init__(self):
        self._init_3d_view()

    def _init_3d_view(self):
        """
        Initializes the 3D view of the visualizer.
        """
        self._fig3d = plt.figure()
        self._ax3d = self._fig3d.add_subplot(111, projection='3d')

        self._lims3d = [-1.25, 1.25]  # 3d axes limites
        self._arrow_style3d = "->"
        self._arrow_scale3d = 12
        self._arrow_length3d = 0.65
        self._eci_frame_colors = ["red", "green", "blue"]
        self._sc_frame_colors = ["lightcoral", "yellowgreen", "skyblue"]
        self._nadir_vector_color = "turquoise"

    def _draw_3d_eci_frame(self):
        """
        Draws the ECI coordinates frame in the 3D view.
        """
        # set size of plot
        self._ax3d.set_xlim(self._lims3d)
        self._ax3d.set_ylim(self._lims3d)
        self._ax3d.set_zlim(self._lims3d)

        # draw x, y, z axes
        origin = np.array([0, 0, 0])
        self._ax3d.arrow3D(origin[0], origin[1], origin[2], self._arrow_length3d, 0, 0, color=self._eci_frame_colors[0],
                           mutation_scale=self._arrow_scale3d,
                           arrowstyle=self._arrow_style3d)
        self._ax3d.arrow3D(origin[0], origin[1], origin[2], 0, self._arrow_length3d, 0, color=self._eci_frame_colors[1],
                           mutation_scale=self._arrow_scale3d,
                           arrowstyle=self._arrow_style3d)
        self._ax3d.arrow3D(origin[0], origin[1], origin[2], 0, 0, self._arrow_length3d, color=self._eci_frame_colors[2],
                           mutation_scale=self._arrow_scale3d,
                           arrowstyle=self._arrow_style3d)

    def _update_3d_view(self, sat_state):
        """
        TODO
        """
        self._ax3d.clear()

        self._draw_3d_eci_frame()

        # compute S/C body frame in ECI coordinates
        sc_body = np.array([[self._arrow_length3d, 0, 0], [0, self._arrow_length3d, 0], [0, 0, -self._arrow_length3d]])
        sc_body_eci = sat_state.attitude.apply(sc_body)

        # draw S/C body frame
        scale = 15
        colors = self._sc_frame_colors
        for axis in range(3):
            self._ax3d.arrow3D(sat_state.position.x, sat_state.position.y, sat_state.position.z, sc_body_eci[axis][0],
                               sc_body_eci[axis][1], sc_body_eci[axis][2], color=colors[axis],
                               mutation_scale=self._arrow_scale3d,
                               arrowstyle=self._arrow_style3d)

        # draw expected nadir
        nadir = np.array(sat_state.nadir)
        nadir *= self._arrow_length3d
        self._ax3d.arrow3D(sat_state.position.x, sat_state.position.y, sat_state.position.z, nadir[0],
                           nadir[1], nadir[2], color=self._nadir_vector_color, mutation_scale=self._arrow_scale3d,
                           arrowstyle=self._arrow_style3d, linestyle="--")

        self._ax3d.set_title(sat_state.timestamp)

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
                                     markersize=15, label='Nadir')
        plt.legend(
            handles=[x_eci_legend, y_eci_legend, z_eci_legend, x_sc_legend, y_sc_legend, z_sc_legend, nadir_legend])
