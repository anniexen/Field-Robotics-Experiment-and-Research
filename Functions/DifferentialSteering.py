import numpy as np
import matplotlib.pyplot as plt

#Define initial values
W = 1
rw = 0.5
dt = 0.05
x = 0
y = 0
angle = 90*np.pi/180
initial_Pose = np.array([x,y,angle])
ssl = 0
ssr = 0
delta = 0*np.pi/180
prior_wr = 0
prior_wl = 0

#Model 1 (without skid/slip):
def diffSteerM1(W = 1, rw = 0.5, dt = 0.05, wr = 20, wl =20.0001, x = 0, y = 0, angle = 1.57):
    if wr == wl:
        wr = wr+ wr * np.finfo(float).eps
    vr = wr * rw #right wheel velocity
    vl = wl * rw #left wheel velocity
    V = (vr+vl)/2 #vehicle velocity
    omega = (vr-vl)/W #vehicle turnrate
    R = V/omega #Turning radius
    xICC = x-R*np.sin(angle)
    yICC = y+R*np.cos(angle)
    ICC = np.array([[xICC], [yICC]])
    rotM=np.array([[np.cos(omega*dt), -np.sin(omega*dt)],
                   [np.sin(omega*dt),  np.cos(omega*dt)]])
    newXY = rotM@(np.array([[x],[y]])-ICC)+ICC
    M1_newX = newXY[0,0]
    M1_newY = newXY[1,0]
    M1_newAngle = omega*dt + angle

    return M1_newX, M1_newY, M1_newAngle


#Model 2 (without skid/slip):
def diffSteerM2(W=1, rw=0.5, dt=0.05, wr=20, wl=10, x=0, y=0, angle=90*np.pi/180):
    vr = wr * rw #right wheel velocity
    vl = wl * rw #left wheel velocity
    V = (vr+vl)/2 #vehicle velocity
    omega = (vr-vl)/W #vehicle turnrate
    M2_newX=x+dt*V*np.cos(angle)
    M2_newY=y+dt*V*np.sin(angle)
    M2_newAngle = omega*dt + angle
    return M2_newX, M2_newY, M2_newAngle



#Model 2 (with skid/slip):
def diffSteerM2SS(W=1, rw=0.5, dt=0.05, wr=20, wl=20.0001, x=0, y=0, angle=1.57,ssl=0.02, ssr=0, delta=0, prior_wl=0, prior_wr=0):
    if (wr - prior_wr) >= 0:  # driving
        sr = ssr
        vr = wr * rw * (1-sr)  # driving: right wheel velocity
    else:  # braking
        sr = ssr*-1
        vr = (wr * rw) / (1+sr)  # braking: right wheel velocity
    if (wl - prior_wl) >= 0:  # driving
        sl = ssl
        vl = wl * rw * (1-sl)  # driving: left wheel velocity
    else:  # braking
        sl = ssl*-1
        vl = (wl * rw) / (1+sl)  # braking: left wheel velocity
    Vlong = (vr + vl) / 2  # vehicle velocity
    omega = (vr - vl)/W  # vehicle turnrate
    Vlat = np.tan(delta) * Vlong
    M2SSnewX = x + dt * Vlong * np.cos(angle) - dt * Vlat * np.sin(angle)
    M2SSnewY = y + dt * Vlong * np.sin(angle) + dt * Vlat * np.cos(angle)
    M2SSnewAngle = omega*dt+angle
    return M2SSnewX, M2SSnewY, M2SSnewAngle




