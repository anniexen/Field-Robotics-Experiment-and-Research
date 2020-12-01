import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

from Functions import DifferentialSteering, FieldCoverage, CollisionCheck
from Functions import KinematicCarModel, Maneuvers, MoveToPoint
from Functions import PositionTransformation, PurePursuitController

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.v += a * dt
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)
    
class States:

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)



class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf
#Define path -  step by step
#Step 1
PathA = np.array((np.ones(181)*10,np.linspace(0,301,181)))
pathToFollow = PathA

#Step 2
turningRadius = 15
width = 5
rows = 6
PathB, DistanceB = piTurn(turningRadius, width, rows)

PathB = np.vstack((PathB, np.ones((1,PathB.shape[1]))))
PathB = trot2(90,"deg")@PathB
PathB = transl2(10,300)@PathB

#Step 3
PathC = np.array((np.ones(181)*40,np.linspace(0,301,181)))

#Step 4
turningRadius = 15
width = 5
rowsD = -2
PathD, DistanceD = omegaTurn(turningRadius, width, rowsD)

PathD = np.vstack((PathD, np.ones((1,PathD.shape[1]))))
PathD = trot2(-90,"deg")@PathD
PathD = transl2(40,0)@PathD

#Step 5
PathE = np.array((np.ones(181)*50,np.linspace(0,301,181)))

#Step 6
turningRadius = 15
width = 5
rowsF = 8
PathF, DistanceF = piTurn(turningRadius, width, rowsF)

PathF = np.vstack((PathF, np.ones((1,PathF.shape[1]))))
PathF= trot2(90,"deg")@PathF
PathF = transl2(50,300)@PathF

#Step 7
PathG = np.array((np.ones(181)*90,np.linspace(0,301,181)))

#Step 8
turningRadius = 15
width = 5
rowsH = 0
PathH, DistanceH = omegaTurn(turningRadius, width, rowsH)

PathH = np.vstack((PathH, np.ones((1,PathH.shape[1]))))
PathH = trot2(-90,"deg")@PathH
PathH = transl2(90,0)@PathH

#Step 9
PathI = np.array((np.ones(181)*90,np.linspace(0,301,181)))

# Example of Actual and Desired Path Plot. Note that the path tracking is not tuned, and excess travel is visible (extra loops). Dashed line used for desired path, so it is visible when over the vehicle path.
# path 1 
# Define variables for testing
# Initial State


#Vehicle Parameters
current_x = 10
current_y = 0
current_angle = np.pi/2
current_steer = 0
current_V = 0
wheelbase = 7
current_V = 0
r_min = 15
maxSteer = np.arctan(wheelbase/r_min)
dt = .001
DT = .1
tauV = 0.5
tauSteer = 0.15
# The Goal
desired_x = 10
desired_y = 300
desired_V = 15
desired_angle = 0
iterations = 200
kh = 1

PathASimul = np.array([[current_x], [current_y], [current_angle],
                      [current_V], [current_steer]])

for i in range(1,iterations):
    desired_Steer = moveToPointConstV(PathASimul[0, i - 1], 
                                      PathASimul[1, i - 1], 
                                      PathASimul[2, i - 1], desired_x, desired_y, kh)
    new_PathASimul = bicycleMS4(u = np.array([[desired_Steer], [desired_V]]), q = PathASimul[:, -1][:, np.newaxis],
                                dt = dt, DT = DT, L = wheelbase, maxSteer = maxSteer)
    PathASimul = np.hstack((PathASimul, new_PathASimul[:, 1:]))

# path 2

current_x = PathASimul[0, -1]
current_y = PathASimul[1, -1]
desired_x = 40
desired_y = 300
desired_V = 15
kh = 4000
iterations = 30

PathBSimul = np.array([[current_x], [current_y], [current_angle],
                      [current_V], [current_steer]])

for i in range(1,iterations):
    desired_Steer = moveToPointConstV(PathBSimul[0, i - 1], 
                                      PathBSimul[1, i - 1], 
                                      PathBSimul[2, i - 1], desired_x, desired_y, kh)
    new_PathBSimul = bicycleMS4(u = np.array([[desired_Steer], [desired_V]]), q = PathBSimul[:, -1][:, np.newaxis],
                                dt = dt, DT = DT, L = wheelbase, maxSteer = maxSteer)
    PathBSimul = np.hstack((PathBSimul, new_PathBSimul[:, 1:]))


# path 3

current_x = 40
current_y = 300
current_angle = 90 * np.pi/180
current_V = 15
current_steer = 0 * np.pi/180
desired_x = 40
desired_y = 600
desired_angle = 90 * np.pi/180
desired_V = 15
iterations = 20500
k_alpha = 2
k_beta = -.5

PathCSimul = np.array([[current_x], [current_y], [current_angle],
                      [current_V], [current_steer]])

desired_Steer = np.zeros((iterations,1))

for i in range(1, iterations):
    desired_Steer[i][0] = moveToPoseConstV(PathCSimul[0, i - 1],
                                      PathCSimul[1, i - 1],
                                      PathCSimul[2, i - 1], current_V, desired_x,
                                      desired_y, desired_angle, k_alpha, k_beta, wheelbase)
    new_PathCSimul = bicycleMS4(u=np.array([desired_Steer[i], [desired_V]]),
                                q=PathCSimul[:, -1][:, np.newaxis],
                                dt=dt, DT=dt, L=wheelbase, maxSteer=maxSteer)
    PathCSimul = np.hstack((PathCSimul, new_PathCSimul[:, 1:]))

PathCSimul_VAR = np.array((PathCSimul[0,:],PathCSimul[1,:]))
PathCSimul_VAR2 = np.array((PathCSimul[2,:], PathCSimul[3,:], PathCSimul[4,:]))
Correction = np.vstack((PathCSimul_VAR,np.ones((1,PathCSimul_VAR.shape[1]))))
Correction = trot2(180, 'deg')@Correction
Correction = transl2(80, 600)@Correction

PathCSimul = np.vstack((Correction,PathCSimul_VAR2))

# path 4

width = 5
rows = -2
PathDSimul, DistanceD = omegaTurn(r_min, width, rows)

PathDSimul = np.vstack((PathDSimul, np.ones((1,PathDSimul.shape[1]))))

PathDSimul = trot2(270, "deg") @ PathDSimul
PathDSimul = transl2(40, 0) @ PathDSimul


# path 5

current_x = PathDSimul[0, -1]
current_y = PathDSimul[1, -1]
current_angle = 90 * np.pi / 180
current_V = 15
current_steer = 0 * np.pi / 180
lookAheadD = 10
iterations = 200

PathESimul = np.array((np.ones(301) * 50, np.linspace(0, 300, 301)))

PathESimulState = np.array([[current_x], [current_y], [current_angle],
                      [current_V], [current_steer]])

dStr = np.zeros((iterations))
xte = np.zeros((iterations))

dStr[0], xte[0] = purePursuitController(q=PathESimulState[:, -1][:, np.newaxis],
                                        L=wheelbase, ld=lookAheadD,
                                        path=PathESimul)

for i in range(1, iterations):
    dStr[i], xte[i] = purePursuitController(q=PathESimulState[:, -1][:, np.newaxis],
                                            L=wheelbase, ld=lookAheadD,
                                            path=PathESimul)
    new_PathESimulState = bicycleMS4(u=np.array([[dStr[i]], [desired_V]]),
                                q=PathESimulState[:, -1][:, np.newaxis],
                                dt=dt, DT=DT, L=wheelbase,
                                tauV=tauV, tauSteer=tauSteer, maxSteer=maxSteer)
    PathESimulState = np.hstack((PathESimulState, new_PathESimulState[:, 1:]))
    
# path 6


width = 5
rows = 10
PathFSimul, DistanceF = piTurn(r_min, width, rows)

PathFSimul = np.vstack((PathFSimul, np.ones((1,PathFSimul.shape[1]))))

PathFSimul = trot2(90, 'deg')@PathFSimul
PathFSimul = transl2(50, 300)@PathFSimul


# path 7


current_x = PathFSimul[0,-1]
current_y = PathFSimul[1, -1]
current_angle = 90 * np.pi / 180
current_V = 15
current_steer = 0 * np.pi / 180
desired_x = 100
desired_y = 600
desired_V = 15
kh = 5000
iterations = 100

PathGSimul = np.array([[current_x], [current_y], [current_angle],
                      [current_V], [current_steer]])

for i in range(1,iterations):
    desired_Steer = moveToPointConstV(PathGSimul[0, i - 1], 
                                      PathGSimul[1, i - 1], 
                                      PathGSimul[2, i - 1], desired_x, desired_y, kh)
    new_PathGSimul = bicycleMS4(u = np.array([[desired_Steer], [desired_V]]), q = PathGSimul[:, -1][:, np.newaxis],
                                dt = dt, DT = DT, L = wheelbase, maxSteer = maxSteer)
    PathGSimul = np.hstack((PathGSimul, new_PathGSimul[:, 1:]))
    
PathGSimul_VAR = np.array((PathGSimul[0,:],PathGSimul[1,:]))
PathGSimul_VAR2 = np.array((PathGSimul[2,:], PathGSimul[3,:], PathGSimul[4,:]))

PathGSimulState = np.vstack((PathGSimul_VAR,np.ones((1,PathGSimul_VAR.shape[1]))))

PathGSimulState = trot2(180, 'deg')@PathGSimulState
PathGSimulState = transl2(200, 600)@PathGSimulState

PathGSimul = np.vstack((PathGSimulState,PathGSimul_VAR2))


# path 8

width = 5
rows = 0
PathHSimul, DistanceH = omegaTurn(r_min, width, rows)

PathHSimul = np.vstack((PathHSimul, np.ones((1,PathHSimul.shape[1]))))


PathHSimul = trot2(270, "deg") @ PathHSimul
PathHSimul = transl2(100, 0) @ PathHSimul

# path 9


current_x = PathHSimul[0, -1]
current_y = PathHSimul[1, -1]
current_angle = 90 * np.pi / 180
current_V = 15
current_steer = 0 * np.pi / 180
lookAheadD = 100
iterations = 200

PathISimul = np.array((np.ones(301) * 100, np.linspace(0, 300, 301)))

PathISimulState = np.array([[current_x], [current_y], [current_angle],
                      [current_V], [current_steer]])

dStr = np.zeros((iterations))
xte = np.zeros((iterations))

dStr[0], xte[0] = purePursuitController(q=PathISimulState[:, -1][:, np.newaxis],
                                        L=wheelbase, ld=lookAheadD,
                                        path=PathISimul)

for i in range(1, iterations):
    dStr[i], xte[i] = purePursuitController(q=PathISimulState[:, -1][:, np.newaxis],
                                            L=wheelbase, ld=lookAheadD,
                                            path=PathISimul)
    new_PathISimulState = bicycleMS4(u=np.array([[dStr[i]], [desired_V]]),
                                q=PathISimulState[:, -1][:, np.newaxis],
                                dt=dt, DT=DT, L=wheelbase,
                                tauV=tauV, tauSteer=tauSteer, maxSteer=maxSteer)
    PathISimulState = np.hstack((PathISimulState, new_PathISimulState[:, 1:]))
    