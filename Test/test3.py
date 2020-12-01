import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

from Functions import DifferentialSteering, FieldCoverage, CollisionCheck
from Functions import KinematicCarModel, Maneuvers, MoveToPoint
from Functions import PositionTransformation, PurePursuitController

# Define variables for testing
# Initial State
init_x = 0
init_y = 0
init_angle = np.pi/2
init_V = 0
init_steer = 0
# The Goal
desired_x = 20
desired_y = 20
desired_angle = 0
# Vehicle parameters
maxSteer = np.pi / 4  # 45 deg steering angle
wheelBase = 2
desired_V = 15
# Controller/Modelling parameters
dt = 0.01
DT = dt
iterations = 500
lookAheadD = 10
pathToFollow = np.array((np.linspace(0, 100, 101),
                         np.linspace(0, 100, 101)))


veh_state = np.array([[init_x], [init_y], [init_angle],
                      [init_V], [init_steer]])
dStr = np.zeros((iterations))
xte = np.zeros((iterations))
dStr[0], xte[0] = purePursuitController(q=veh_state[:, -1][:, np.newaxis],
                                             L=wheelBase, ld=lookAheadD,
                                             path=pathToFollow)

for i in range(1, iterations):
    dStr[i], xte[i] = purePursuitController(q=veh_state[:, -1][:, np.newaxis],
                                                 L=wheelBase, ld=lookAheadD,
                                                 path=pathToFollow)
    new_veh_state = bicycleMS4(u=np.array([[dStr[i]], [desired_V]]),
                               q=veh_state[:, -1][:, np.newaxis],
                               dt=dt, DT=dt, L=wheelBase, maxSteer=maxSteer)
    veh_state = np.hstack((veh_state, new_veh_state[:, 1:]))