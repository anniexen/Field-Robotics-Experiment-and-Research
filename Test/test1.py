import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

from Functions import DifferentialSteering, FieldCoverage, CollisionCheck
from Functions import KinematicCarModel, Maneuvers, MoveToPoint
from Functions import PositionTransformation, PurePursuitController

iterations = 100

pose_SLM2d1 = np.zeros((iterations,3))
pose_SLM2d1[0] = np.array([0, 0, 90*np.pi/180])
dt = 0.05
        
for i in range (1,iterations):
    pose_SLM2d1[i][0], pose_SLM2d1[i][1], pose_SLM2d1[i][2] = M2(W,rw,dt, 5, 5, pose_SLM2d1[i-1][0], pose_SLM2d1[i-1][1], pose_SLM2d1[i-1][2])
    #print(pose_SLM2d1)
    
pose_SLM2d2 = np.zeros((iterations,3))
pose_SLM2d2[0] = np.array([10, 0, 90*np.pi/180])
dt=0.1
        
for i in range (1,iterations):
    pose_SLM2d2[i][0], pose_SLM2d2[i][1], pose_SLM2d2[i][2] = M2(W,rw,dt, 5, 5, pose_SLM2d2[i-1][0], pose_SLM2d2[i-1][1], pose_SLM2d2[i-1][2])
    #print(pose_SLM2d2) 
    
pose_SLM2d3 = np.zeros((iterations,3))
pose_SLM2d3[0] = np.array([20, 0, 90*np.pi/180])
dt=0.5
        
for i in range (1,iterations):
    pose_SLM2d3[i][0], pose_SLM2d3[i][1], pose_SLM2d3[i][2] = M2(W,rw,dt, 5, 5, pose_SLM2d3[i-1][0], pose_SLM2d3[i-1][1], pose_SLM2d3[i-1][2])
    #print(pose_SLM2d2) 


iterations = 112
dt = 0.05

pose_SQM2d1 = np.zeros((iterations,3))
pose_SQM2d1[0] = np.array([0, 0, 90*np.pi/180])
dt = 0.05

V_normal = 5
V = np.zeros(iterations)
omega = np.zeros(iterations)

        
for i in range (iterations):
    
    if i < iterations/8:
        V[i] = V_normal
        omega[i] = 0
    
    elif iterations/8 <= i < iterations*2/8:
        V[i] = 0
        omega[i] = -1*(np.pi/2)/(14*dt)
    
    elif iterations*2/8 <= i < iterations*3/8:
        V[i] = V_normal
        omega[i] = 0
    
    elif iterations*3/8 <= i < iterations*4/8:
        V[i] = 0
        omega[i] = -1*(np.pi/2)/(14*dt)
        
    elif iterations*4/8 <= i < iterations*5/8:
        V[i] = V_normal
        omega[i] = 0
        
    elif iterations*5/8 <= i < iterations*6/8:
        V[i] = 0
        omega[i] = -1*(np.pi/2)/(14*dt)
        
    elif iterations*6/8 <= i < iterations*7/8:
        V[i] = V_normal
        omega[i] = 0
        
    elif iterations*7/8 <= i < iterations:
        V[i] = 0
        omega[i] = -1*(np.pi/2)/(14*dt)
        
vl = V-omega * W/2
vr = V+omega * W/2
wl = vl/rw
wr = vr/rw

for i in range (1,iterations):
    pose_SQM2d1[i][0], pose_SQM2d1[i][1], pose_SQM2d1[i][2] = M2(W,rw,dt, wr[i], wl[i], pose_SQM2d1[i-1][0], pose_SQM2d1[i-1][1], pose_SQM2d1[i-1][2])


iterations = 112
dt = 0.1

pose_SQM2d2 = np.zeros((iterations,3))
pose_SQM2d2[0] = np.array([20, 0, 90*np.pi/180])

V_normal = 5
V = np.zeros(iterations)
omega = np.zeros(iterations)

        
for i in range (iterations):
    
    if i < iterations/8:
        V[i] = V_normal
        omega[i] = 0
    
    elif iterations/8 <= i < iterations*2/8:
        V[i] = 0
        omega[i] = -1*(np.pi/2)/(14*dt)
    
    elif iterations*2/8 <= i < iterations*3/8:
        V[i] = V_normal
        omega[i] = 0
    
    elif iterations*3/8 <= i < iterations*4/8:
        V[i] = 0
        omega[i] = -1*(np.pi/2)/(14*dt)
        
    elif iterations*4/8 <= i < iterations*5/8:
        V[i] = V_normal
        omega[i] = 0
        
    elif iterations*5/8 <= i < iterations*6/8:
        V[i] = 0
        omega[i] = -1*(np.pi/2)/(14*dt)
        
    elif iterations*6/8 <= i < iterations*7/8:
        V[i] = V_normal
        omega[i] = 0
        
    elif iterations*7/8 <= i < iterations:
        V[i] = 0
        omega[i] = -1*(np.pi/2)/(14*dt)
        
vl = V-omega * W/2
vr = V+omega * W/2
wl = vl/rw
wr = vr/rw

for i in range (1,iterations):
    pose_SQM2d2[i][0], pose_SQM2d2[i][1], pose_SQM2d2[i][2] = M2(W,rw,dt, wr[i], wl[i], pose_SQM2d2[i-1][0], pose_SQM2d2[i-1][1], pose_SQM2d2[i-1][2])


iterations = 112
dt = 0.5

pose_SQM2d3 = np.zeros((iterations,3))
pose_SQM2d3[0] = np.array([40, 0, 90*np.pi/180])

V_normal = 5
V = np.zeros(iterations)
omega = np.zeros(iterations)

        
for i in range (iterations):
    
    if i < iterations/8:
        V[i] = V_normal
        omega[i] = 0
    
    elif iterations/8 <= i < iterations*2/8:
        V[i] = 0
        omega[i] = -1*(np.pi/2)/(14*dt)
    
    elif iterations*2/8 <= i < iterations*3/8:
        V[i] = V_normal
        omega[i] = 0
    
    elif iterations*3/8 <= i < iterations*4/8:
        V[i] = 0
        omega[i] = -1*(np.pi/2)/(14*dt)
        
    elif iterations*4/8 <= i < iterations*5/8:
        V[i] = V_normal
        omega[i] = 0
        
    elif iterations*5/8 <= i < iterations*6/8:
        V[i] = 0
        omega[i] = -1*(np.pi/2)/(14*dt)
        
    elif iterations*6/8 <= i < iterations*7/8:
        V[i] = V_normal
        omega[i] = 0
        
    elif iterations*7/8 <= i < iterations:
        V[i] = 0
        omega[i] = -1*(np.pi/2)/(14*dt)
        
vl = V-omega * W/2
vr = V+omega * W/2
wl = vl/rw
wr = vr/rw

for i in range (1,iterations):
    pose_SQM2d3[i][0], pose_SQM2d3[i][1], pose_SQM2d3[i][2] = M2(W,rw,dt, wr[i], wl[i], pose_SQM2d3[i-1][0], pose_SQM2d3[i-1][1], pose_SQM2d3[i-1][2])


wr = 19
wl = 20
iterations = 450

dt = 0.05
pose_CM2d1 = np.zeros((iterations,3)) #Array to hold positions
pose_CM2d1[0] = np.array([0,0,np.pi/180])

for i in range(1,iterations):
    pose_CM2d1[i][0], pose_CM2d1[i][1], pose_CM2d1[i][2] = M2SS(W,rw,dt, wr, wl, pose_CM2d1[i-1][0], pose_CM2d1[i-1][1], pose_CM2d1[i-1][2])

dt = 0.1
pose_CM2d2 = np.zeros((iterations,3)) #Array to hold positions
pose_CM2d2[0] = np.array([100,0,np.pi/180])

for i in range(1,iterations):
    pose_CM2d2[i][0], pose_CM2d2[i][1], pose_CM2d2[i][2] = M2SS(W,rw,dt, wr, wl, pose_CM2d2[i-1][0], pose_CM2d2[i-1][1], pose_CM2d2[i-1][2])
    
dt = 0.5
pose_CM2d3 = np.zeros((iterations,3)) #Array to hold positions
pose_CM2d3[0] = np.array([100,75,np.pi/180])

for i in range(1,iterations):
    pose_CM2d3[i][0], pose_CM2d3[i][1], pose_CM2d3[i][2] = M2SS(W,rw,dt, wr, wl, pose_CM2d3[i-1][0], pose_CM2d3[i-1][1], pose_CM2d3[i-1][2])
