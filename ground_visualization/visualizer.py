#!/usr/bin/env python3

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from pyquaternion import Quaternion

import plotters_utils


def draw_reference_frame(plot):
    # set size of plot
    axes_lim = [-1, 1]
    plot.set_xlim(axes_lim)
    plot.set_ylim(axes_lim)
    plot.set_zlim(axes_lim)

    # draw x, y, z axes
    arrow_style = "-|>"
    scale = 15
    plot.arrow3D(0, 0, 0, 1, 0, 0, color="red", mutation_scale=scale, arrowstyle=arrow_style)
    plot.arrow3D(0, 0, 0, 0, 1, 0, color="green", mutation_scale=scale, arrowstyle=arrow_style)
    plot.arrow3D(0, 0, 0, 0, 0, 1, color="blue", mutation_scale=scale, arrowstyle=arrow_style)


if __name__ == '__main__':
    # prepare
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    plotters_utils.set_size(10, 10, ax)

    draw_reference_frame(ax)

    # read attitude
    df = pd.read_csv("../data/webmust_labeled/labeled_O_Q_FB_FI_EST.csv", index_col=0)
    df.index = pd.to_datetime(df.index, format="%Y-%m-%d %H:%M:%S.%f")


    def animate(i):
        ax.clear()

        draw_reference_frame(ax)

        # compute spacecraft attitude
        sc = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=float)
        rotation = Quaternion([df.iloc[i, 0], df.iloc[i, 1], df.iloc[i, 2], df.iloc[i, 3]])

        for i, row in enumerate(sc):
            sc[i] = rotation.rotate(row)

        # draw spacecraft attitude
        arrow_style = "-|>"
        scale = 15
        # draw x
        axis = 0
        ax.arrow3D(0, 0, 0, sc[axis][0], sc[axis][1], sc[axis][2], color="lightcoral", mutation_scale=scale,
                   arrowstyle=arrow_style)
        # draw y
        axis = 1
        ax.arrow3D(0, 0, 0, sc[axis][0], sc[axis][1], sc[axis][2], color="yellowgreen", mutation_scale=scale,
                   arrowstyle=arrow_style)
        # draw z
        axis = 2
        ax.arrow3D(0, 0, 0, sc[axis][0], sc[axis][1], sc[axis][2], color="skyblue", mutation_scale=scale,
                   arrowstyle=arrow_style)


    # create animation
    ani = animation.FuncAnimation(fig, animate, interval=50)

    # show
    plt.show()
