import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

def piTurn(r_min, w_row, rows):
    # First check if a pi turn is possible
    if rows * np.abs(w_row) < 2 * r_min:
        path = np.zeros((0, 0))  # Turn is not possible. Path is empty
        distance = np.nan  # Distance cannot be calculated
        return (path, distance)

    d = rows * w_row  # distance from start path to end path
    if d > 0:  # Turn to the right
        # Create the starting arc for leaving
        # the initial path (60 points+endpoint)
        a = np.linspace(-np.pi/2, 0, 61)
        x_start = 0 + r_min * np.cos(a)
        y_start = -1 * r_min - r_min * np.sin(a)
        # Create the final arc for entering
        # the target path (60 points+endpoint)
        a = np.linspace(0, np.pi/2, 61)
        x_end = 0 + r_min * np.cos(a)
        y_end = -1 * rows * w_row + r_min - r_min * np.sin(a)
        # Create straight section if necessary
        if rows * w_row == 2 * r_min:  # no straight section. Connect arcs
            # The first point in x_end repeats x_start.
            # Same for y_end and y_start
            x = np.hstack((x_start, x_end[1:]))
            y = np.hstack((y_start, y_end[1:]))
            path = np.array((x, y))
        else:
            # Create straight section
            x_straight = np.linspace(x_start[-1], x_end[0], 61)
            y_straight = np.linspace(y_start[-1], y_end[0], 61)
            # Connect segments. Once again each segment repeats the start
            # and end.
            x = np.hstack((x_start, x_straight[1:], x_end[1:]))
            y = np.hstack((y_start, y_straight[1:], y_end[1:]))
    else:
        # Create the starting arc for leaving
        # the initial path (60 points+endpoint)
        a = np.linspace(np.pi/2, 0, 61)
        x_start = 0 + r_min * np.cos(a)
        y_start = r_min - r_min * np.sin(a)
        # Create the final arc for entering
        # the target path (60 points+endpoint)
        a = np.linspace(0, -1 * np.pi/2, 61)
        x_end = 0 + r_min * np.cos(a)
        y_end = -rows * w_row - r_min - r_min * np.sin(a)
        # Create straight section if necessary
        if rows * w_row == 2 * r_min:  # no straight section. Connect arcs
            # The first point in x_end repeats x_start.
            # Same for y_end and y_start
            x = np.hstack((x_start, x_end[1:]))
            y = np.hstack((y_start, y_end[1:]))
            path = np.array((x, y))
        else:
            # Create straight section
            x_straight = np.linspace(x_start[-1], x_end[0], 61)
            y_straight = np.linspace(y_start[-1], y_end[0], 61)
            # Connect segments. Once again each segment repeats the start
            # and end.
            x = np.hstack((x_start, x_straight[1:], x_end[1:]))
            y = np.hstack((y_start, y_straight[1:], y_end[1:]))
    path = np.array((x, y))
    distance = rows * w_row + (np.pi - 2) * r_min
    return path, distance



def omegaTurn(r_min, w_row, rows):
    # First check if a omega turn is possible
    d = rows * w_row  # distance from start path to end path
    if rows * w_row > 2 * r_min:
        path = np.zeros((0, 0))  # Turn is not possible. Path is empty
        distance = np.nan  # Distance cannot be calculated
        return (path, distance)

    if d > 0:  # Turn to the right
        # Create the starting arc for leaving the  path (60 points+endpoint)
        # Arc starts at pi/2 and rotates up/back toward 0, angle will be alpha
        alpha = np.arccos((r_min + d / 2) / (2 * r_min))
        a = np.linspace(np.pi / 2, np.pi / 2 - alpha, 61)
        x_start = 0 + r_min * np.cos(a)
        y_start = r_min - r_min * np.sin(a)
        # Create the final arc for entering the  path (60 points+endpoint)
        a = np.linspace(-1 * np.pi / 2 + alpha, -1 * np.pi/2, 61)
        x_end = 0 + r_min * np.cos(a)
        y_end = -1 * d - r_min - r_min * np.sin(a)
        # Create bulb section
        bulb_center_x = 2 * r_min * np.sqrt(1 -
                                            np.float_power((r_min + d / 2) /
                                                           (2 * r_min), 2))
        bulb_center_y = -1 * d / 2
        a = np.linspace(-1 * np.pi/2 - alpha, np.pi / 2 + alpha, 61)
        x_bulb = bulb_center_x + r_min * np.cos(a)
        y_bulb = bulb_center_y - r_min * np.sin(a)
    else:
        # Create the starting arc for leaving the path (60 points+endpoint)
        d = d * -1
        # Arc starts at pi/2 and rotates up/back toward 0, angle will be alpha
        alpha = np.arccos((r_min + d / 2) / (2 * r_min))
        a = np.linspace(-1 * np.pi/2, -1 * np.pi / 2 + alpha, 61)
        x_start = 0 + r_min * np.cos(a)
        y_start = -1 * r_min - r_min * np.sin(a)
        # Create the final arc for entering the path (60 points+endpoint)
        a = np.linspace(np.pi / 2 - alpha, np.pi / 2, 61)
        x_end = 0 + r_min * np.cos(a)
        y_end = d + r_min - r_min * np.sin(a)
        # Create bulb section
        bulb_center_x = 2 * r_min * np.sqrt(1 -
                                            np.float_power((r_min + d / 2) /
                                                           (2 * r_min), 2))
        bulb_center_y = d / 2
        a = np.linspace(np.pi / 2 + alpha, -1 * np.pi/2 - alpha, 61)
        x_bulb = bulb_center_x + r_min * np.cos(a)
        y_bulb = bulb_center_y - r_min * np.sin(a)
    # Connect segments. Each segment repeats the start and end.
    x = np.hstack((x_start, x_bulb[1:], x_end[1:]))
    y = np.hstack((y_start, y_bulb[1:], y_end[1:]))
    path = np.array((x, y))
    distance = (4 * alpha + np.pi) * r_min
    return path, distance
