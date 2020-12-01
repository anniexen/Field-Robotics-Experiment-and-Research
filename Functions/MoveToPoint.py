import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

def angdiff(angle1, angle2):
    a = angle1 - angle2
    while a < (-1 * np.pi):
        a = a + 2 * np.pi
    while a >= np.pi:
        a = a - 2 * np.pi
    return a


def moveToPoseConstV(current_x, current_y, current_angle, current_V,
                     desired_x, desired_y, desired_angle,
                     k_alpha, k_beta, wheelbase):
    alpha = (np.arctan2((desired_y - current_y), (desired_x - current_x))
             - current_angle)
    beta = -1 * current_angle - alpha + desired_angle
    omega = k_alpha * alpha + k_beta*beta
    newSteer = np.arctan((omega * wheelbase) / current_V)
    return newSteer

def moveToPose(current_x, current_y, current_angle,
               desired_x, desired_y, desired_angle,
               k_rho, k_alpha, k_beta, wheelbase):
    rho = np.sqrt(np.float_power(desired_x - current_x, 2)
                  + np.float_power(desired_y - current_y, 2))
    alpha = (np.arctan((desired_y - current_y) / (desired_x - current_x))
             - current_angle)
    beta = -1 * current_angle - alpha + desired_angle
    newV = k_rho * rho
    omega = k_alpha * alpha + k_beta*beta
    newSteer = np.arctan((omega * wheelbase) / newV)
    return newV, newSteer

def moveToPointConstV(current_x, current_y, current_angle,
                      desired_x, desired_y, kh):
    desired_angle = np.arctan2((desired_y - current_y),
                               (desired_x - current_x))
    newSteer = kh * angdiff(desired_angle, current_angle)
    return newSteer

def bicycleMS4(u=np.array([[0], [1]]),
               q=np.array([[0], [0], [-1.57], [1], [0]]),
               dt=0.01, DT=0.01, L=2.5, ss=0, delta1=0,
               delta2=0, tauV=0, tauSteer=0, maxSteer=90 * np.pi / 180):
    # Check our input. We need min and max steering angles.
    if u[0, 0] > maxSteer:
        u[0, 0] = maxSteer
    if u[0, 0] < -1 * maxSteer:
        u[0, 0] = -1 * maxSteer

    num_loops = int(DT/dt)+1
    newq = np.zeros((5, num_loops))  # Each column contains a time step.
    # Column 0 of newq will be state at time 0, i.e. initial state, q.
    newq[0, 0] = q[0, 0]
    newq[1, 0] = q[1, 0]
    newq[2, 0] = q[2, 0]
    newq[3, 0] = q[3, 0]
    newq[4, 0] = q[4, 0]
    for i in range(1, num_loops):
        if newq[4, 0] > maxSteer:
            newq[4, 0] = maxSteer
        if newq[4, 0] < -1 * maxSteer:
            newq[4, 0] = -1 * maxSteer
        if tauV < dt:
            tauV = dt
        if tauSteer < dt:
            tauSteer = dt
        newq[3, i] = newq[3, i - 1] * (1 - dt / tauV) + (dt / tauV) * u[1, 0]
        newq[4, i] = (newq[4, i - 1] * (1 - dt / tauSteer) +
                      (dt / tauSteer) * u[0, 0])  # Steering Control Equation
        # Test if we are braking or driving. If the new vecolity newq[3,0]
        # is greater than current velocity q[3,0], we assume that we will be
        # accelerating/driving.
        if (newq[3, i]-newq[3, i - 1]) >= 0:  # driving
            s = ss
            vlong = newq[3, i] * (1 - s)  # driving: wheel velocity
        else:  # braking
            s = ss * -1
            vlong = (newq[3, i]) / (1 + s)  # braking: wheel velocity
        vlat = vlong * np.tan(delta2)
        newq[0, i] = (newq[0, i - 1] + dt * (vlong * np.cos(newq[2, i - 1])
                                             - vlat * np.sin(newq[2, i - 1])))
        newq[1, i] = (newq[1, i - 1] + dt * (vlong * np.sin(newq[2, i - 1])
                                             + vlat * np.cos(newq[2, i - 1])))
        newq[2, i] = (newq[2, i - 1]
                      + (dt * (vlong * np.tan(newq[4, i - 1] + delta1) / L
                               - vlat / L)))
    return newq
