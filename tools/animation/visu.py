#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  L-Cube visualization main module
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Main program of the visualization that makes use of the other module in the same folder
"""

from matplotlib.animation import FuncAnimation
import numpy as np
from matplotlib import pyplot as plt
import csv
from threading import Thread
from matplotlib.widgets import Button
import matplotlib.patches as patches

# my modules
import bs_print
import receive_data
import var

# if actual connections should be established
establish_connection = 1
# if cube should operate via mqtt or serial
use_mqtt = 1
# if recorded serial data of cube1 should be logged to file after closing
write_logged_data = 0
# animation period [ms]
animation_period = 200
# size of markers for cube and tile
marker_size = 50

# function that gets called every animation period
def animate(i):
    # =====================================================================================
    # draw 3d part
    # draw data =====================================================
    # for clear new picture
    ax_3d.clear()

    # formation of axes has to be done every frame
    ax_3d.axes.set_xlim3d(left=2, right=-2)
    ax_3d.axes.set_ylim3d(bottom=2, top=-2)
    ax_3d.axes.set_zlim3d(bottom=0, top=4)

    ax_3d.set_aspect('equal', adjustable='box')
    ax_3d.set_xlabel('x')
    ax_3d.set_ylabel('y')
    ax_3d.set_zlabel('z')

    # draw base stations
    bs_print.draw_bs(ax_3d, bs_cube_coords[0])
    bs_print.draw_bs(ax_3d, bs_cube_coords[1])

    # draw cube 1 - red --------------------------------------------------------------------------------------------
    ax_3d.scatter3D(var.position[0], var.position[1], var.position[2], c='r', marker='s', s=marker_size)
    # draw heading only with crossing
    if var.is_crossing == 1:
        # front vector
        arrow_f = bs_print.Arrow3D(var.position[0], var.position[1], var.position[2],
                                   var.front_vector[0] * arrow_scale, var.front_vector[1] * arrow_scale,
                                   var.front_vector[2] * arrow_scale,
                                   mutation_scale=5, ec='red')
        ax_3d.add_artist(arrow_f)
        # side vector
        side_vector = np.cross(var.up_vector, var.front_vector)
        side_vector = side_vector / np.linalg.norm(side_vector)
        arrow_s = bs_print.Arrow3D(var.position[0], var.position[1], var.position[2],
                                   side_vector[0] * arrow_scale, side_vector[1] * arrow_scale, side_vector[2] * arrow_scale,
                                   mutation_scale=5, ec='green')
        ax_3d.add_artist(arrow_s)
        # up vector
        arrow_u = bs_print.Arrow3D(var.position[0], var.position[1], var.position[2],
                                   var.up_vector[0] * arrow_scale, var.up_vector[1] * arrow_scale, var.up_vector[2] * arrow_scale,
                                   mutation_scale=5, ec='blue')
        ax_3d.add_artist(arrow_u)

    # draw cube 2 - green --------------------------------------------------------------------------------------------
    ax_3d.scatter3D(var.position2[0], var.position2[1], var.position2[2], c='g', marker='s', s=marker_size)
    # draw heading only with crossing
    if var.is_crossing2 == 1:
        # front vector
        arrow_f2 = bs_print.Arrow3D(var.position2[0], var.position2[1], var.position2[2],
                                   var.front_vector2[0] * arrow_scale, var.front_vector2[1] * arrow_scale,
                                   var.front_vector2[2] * arrow_scale,
                                   mutation_scale=5, ec='red')
        ax_3d.add_artist(arrow_f2)
        # side vector
        side_vector2 = np.cross(var.up_vector2, var.front_vector2)
        side_vector2 = side_vector2 / np.linalg.norm(side_vector2)
        arrow_s2 = bs_print.Arrow3D(var.position2[0], var.position2[1], var.position2[2],
                                   side_vector2[0] * arrow_scale, side_vector2[1] * arrow_scale, side_vector2[2] * arrow_scale,
                                   mutation_scale=5, ec='green')
        ax_3d.add_artist(arrow_s2)
        # up vector
        arrow_u2 = bs_print.Arrow3D(var.position2[0], var.position2[1], var.position2[2],
                                   var.up_vector2[0] * arrow_scale, var.up_vector2[1] * arrow_scale, var.up_vector2[2] * arrow_scale,
                                   mutation_scale=5, ec='blue')
        ax_3d.add_artist(arrow_u2)

    # draw tile - blue ---------------------------------------------------------------------------------------------
    ax_3d.scatter3D(var.position_tile[0], var.position_tile[1], var.position_tile[2], c='b', marker='o', s=marker_size)
    # draw heading only with crossing
    if var.is_crossing_tile == 1:
        # front vector
        arrow_f_tile = bs_print.Arrow3D(var.position_tile[0], var.position_tile[1], var.position_tile[2],
                                        var.front_vector_tile[0] * arrow_scale, var.front_vector_tile[1] * arrow_scale,
                                        var.front_vector_tile[2] * arrow_scale,
                                        mutation_scale=5, ec='red')
        ax_3d.add_artist(arrow_f_tile)
        # side vector
        side_vector_tile = np.cross(var.up_vector_tile, var.front_vector_tile)
        side_vector_tile = side_vector_tile / np.linalg.norm(side_vector_tile)
        arrow_s_tile = bs_print.Arrow3D(var.position_tile[0], var.position_tile[1], var.position_tile[2],
                                        side_vector_tile[0] * arrow_scale, side_vector_tile[1] * arrow_scale,
                                        side_vector_tile[2] * arrow_scale,
                                        mutation_scale=5, ec='green')
        ax_3d.add_artist(arrow_s_tile)
        # up vector
        arrow_u_tile = bs_print.Arrow3D(var.position_tile[0], var.position_tile[1], var.position_tile[2],
                                        var.up_vector_tile[0] * arrow_scale, var.up_vector_tile[1] * arrow_scale,
                                        var.up_vector_tile[2] * arrow_scale,
                                        mutation_scale=5, ec='blue')
        ax_3d.add_artist(arrow_u_tile)

    # could be used for backtracking position if addapted to data
    # x = [0, 1]
    # y = [0, 1]
    # z = [0, 1]
    # ax.plot3D(x, y, z, 'gray', linewidth=0.1)

    # =====================================================================================
    # display positions on left side
    ax_params.clear()
    ax_params.set_ylim([0, 3])
    ax_params.set_xlim([0, 3])
    ax_params.get_xaxis().set_visible(False)
    ax_params.get_yaxis().set_visible(False)

    # separator lines
    ax_params.plot([1, 1], [0, 3], color="black")
    ax_params.plot([2, 2], [0, 3], color="black")
    ax_params.plot([0, 3], [1, 1], color="black")
    ax_params.plot([0, 3], [2, 2], color="black")

    # identification text
    ax_params.text(0.5, 2.5, "Cube 1", horizontalalignment='center', verticalalignment='center', color='r')
    ax_params.text(1.5, 2.5, "Cube 2", horizontalalignment='center', verticalalignment='center', color='g')
    ax_params.text(2.5, 2.5, "Tile 1", horizontalalignment='center', verticalalignment='center', color='b')

    # positions
    ax_params.text(0.5, 1.5,
                   "[" + "{:.2f}".format(var.position[0]) + ", " + "{:.2f}".format(var.position[1]) + ", "
                   + "{:.2f}".format(var.position[2]) + "]", horizontalalignment='center', verticalalignment='center')
    ax_params.text(1.5, 1.5,
                   "[" + "{:.2f}".format(var.position2[0]) + ", " + "{:.2f}".format(var.position2[1]) + ", "
                   + "{:.2f}".format(var.position2[2]) + "]", horizontalalignment='center', verticalalignment='center')
    ax_params.text(2.5, 1.5,
                   "[" + "{:.2f}".format(var.position_tile[0]) + ", " + "{:.2f}".format(var.position_tile[1]) + ", "
                   + "{:.2f}".format(var.position_tile[2]) + "]", horizontalalignment='center', verticalalignment='center')

    # rectangle if sweeping or crossing
    rect1 = None
    rect2 = None
    rect3 = None
    # cube 1
    if var.is_crossing == 1:
        ax_params.text(0.5, 0.5, "Crossing", horizontalalignment='center', verticalalignment='center')
        rect1 = patches.Rectangle((0, 0), 1, 1, linewidth=0, facecolor='green')
    else:
        ax_params.text(0.5, 0.5, "Sweeping", horizontalalignment='center', verticalalignment='center')
        rect1 = patches.Rectangle((0, 0), 1, 1, linewidth=0, facecolor='red')
    # cube 2
    if var.is_crossing2 == 1:
        ax_params.text(1.5, 0.5, "Crossing", horizontalalignment='center', verticalalignment='center')
        rect2 = patches.Rectangle((1, 0), 1, 1, linewidth=0, facecolor='green')
    else:
        ax_params.text(1.5, 0.5, "Sweeping", horizontalalignment='center', verticalalignment='center')
        rect2 = patches.Rectangle((1, 0), 1, 1, linewidth=0, facecolor='red')
    # tile
    if var.is_crossing_tile == 1:
        ax_params.text(2.5, 0.5, "Crossing", horizontalalignment='center', verticalalignment='center')
        rect3 = patches.Rectangle((2, 0), 1, 1, linewidth=0, facecolor='green')
    else:
        ax_params.text(2.5, 0.5, "Sweeping", horizontalalignment='center', verticalalignment='center')
        rect3 = patches.Rectangle((2, 0), 1, 1, linewidth=0, facecolor='red')

    # Add the patch to the Axes
    ax_params.add_patch(rect1)
    ax_params.add_patch(rect2)
    ax_params.add_patch(rect3)

    # =====================================================================================
    # draw violin plot
    ax1.clear()
    ax1.get_xaxis().set_visible(False)

    # depending on the button state, violin plot gets drawn or not
    if var.button_state is True:
        ax1.title.set_text('Cube 1 Error to (0, 0, 1)')
        ax1.violinplot(var.violin_dataset)


# callback for button
def cb_button(event):
    # update button state - true is recording, false is not recording
    var.button_state = not var.button_state
    if var.button_state is True:
        my_button.label.set_text('Stop Violin Plot')
    else:
        my_button.label.set_text('Start Violin Plot')
        var.violin_dataset = [0]
    fig.canvas.draw()


# read corner coordinates for cube which represents the base stations
bs_cube_coords = bs_print.read_configuration()

# create overlay of figure
fig, (ax1, ax2) = plt.subplots(1, 2, width_ratios=[1, 2])
fig.suptitle('LOST-CUBE Visualization')
# remove the right subplot to be set as 3d projections
ax2.remove()
# add the subplot on the right side back as 3d projections
ax2 = fig.add_subplot(1, 2, 2, projection='3d')

# adjust subplot on left side to smaller size
ax1.set_position([0.1, 0.5, 0.3, 0.23])

# add axis for parameters
ax_params = fig.add_axes([0.1, 0.2, 0.3, 0.2])

# add button
ax_one_button = fig.add_axes([0.1, 0.1, 0.3, 0.05])
my_button = Button(ax_one_button, 'Start Violin Plot')
my_button.on_clicked(cb_button)

# set 3d axis
ax_3d = ax2

# scale of heading arrows
arrow_scale = 0.4

# get receivers working - comment the start() functions to omit the specific receiver
thread_mqtt = 0
thread_cube = 0
thread_cube2 = 0
thread_tile = 0
if establish_connection == 1:
    if use_mqtt == 1:
        thread_mqtt = Thread(target=receive_data.thread_receiver_cubes_mqtt, args=())
        thread_mqtt.start()

    else:
        thread_cube = Thread(target=receive_data.thread_receiver_cube, args=())
        thread_cube.start()

        thread_cube2 = Thread(target=receive_data.thread_receiver_cube2, args=())
        thread_cube2.start()

    thread_tile = Thread(target=receive_data.thread_receiver_tile, args=())
    thread_tile.start()

# animated 3d graph with update interval in milliseconds
ani = FuncAnimation(plt.gcf(), animate, interval=animation_period, cache_frame_data=False)

plt.show()

# this is not reached until plot is closed
# stop recording
var.stop = 1

# write data to file if required
if write_logged_data == 1:
    header = ['tile', 'pos_x [m]', 'pos_y [m]', 'pos_z [m]', 'front_vec_x', 'front_vec_y', 'front_vec_z', 'up_vec_x', 'up_vec_y', 'up_vec_z', 'time']
    with open('animation_record.csv', 'w', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)

        # write the header
        writer.writerow(header)
        for mes_point in range(len(var.data)):
            writer.writerow(var.data[mes_point])

if establish_connection == 1:
    # wait other threads
    if use_mqtt == 1:
        thread_mqtt.join()
    else:
        thread_cube.join()
        thread_cube2.join()
    thread_tile.join()

