from Functions import DifferentialSteering, FieldCoverage, CollisionCheck
from Functions import KinematicCarModel, Maneuvers, MoveToPoint
from Functions import PositionTransformation, PurePursuitController

from Test import test1, test2, test3, test4

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

#TEST 1
#Straight Path
def main():
    axis = [-10, 90, -20, 80]

    fig, ax = plt.subplots(figsize=(5, 5))

    ax.set_xlim(axis[0], axis[1])
    ax.set_ylim(axis[2], axis[3])
    ax.grid(True)

    plt.plot(pose_SLM2d1[:,0],pose_SLM2d1[:,1], label = "dt = 0.05")
    plt.plot(pose_SLM2d2[:,0],pose_SLM2d2[:,1], label = "dt = 0.1")
    plt.plot(pose_SLM2d3[:,0],pose_SLM2d3[:,1], label = "dt = 0.5")
    plt.title('Model 2 Straight Line')
    plt.legend()

#Square Path
def main():
    axis = [-10, 90, -10, 90]

    fig, ax = plt.subplots(figsize=(5, 5))

    ax.set_xlim(axis[0], axis[1])
    ax.set_ylim(axis[2], axis[3])
    ax.grid(True)

    ax.plot(pose_SQM2d1[:,0],pose_SQM2d1[:,1], label = 'dt = 0.05')
    ax.plot(pose_SQM2d2[:,0],pose_SQM2d2[:,1], label = 'dt = 0.1')
    ax.plot(pose_SQM2d3[:,0],pose_SQM2d3[:,1], label = 'dt = 0.5')
    ax.legend()
    ax.title.set_text('Model 2 Square')

#Cicle Path
def main():
    axis = [-100, 300, -100, 300]

    fig, ax = plt.subplots(figsize=(5, 5))

    ax.set_xlim(axis[0], axis[1])
    ax.set_ylim(axis[2], axis[3])
    ax.grid(True)

    ax.plot(pose_CM2d1[:,0],pose_CM2d1[:,1], label = 'dt = 0.05')
    ax.plot(pose_CM2d2[:,0],pose_CM2d2[:,1], label = 'dt = 0.1')
    ax.plot(pose_CM2d3[:,0],pose_CM2d3[:,1], label = 'dt = 0.5')
    ax.legend()
    ax.title.set_text('Model 2 Circle')

#TEST 2

def main():
    x, y, yaw = 3., 3.,9. #Insert variables x,y,yaw
    plt.axis('equal')
    plot_car(x, y, yaw, truckcolor="r")
    plt.show()


#TEST 3 
def main():
    fig, ax = plt.subplots()  # Create a figure containing a single axes.
    ax.plot(veh_state[0, :], veh_state[1, :], label='Vehicle Path')
    ax.plot(pathToFollow[0, :], pathToFollow[1, :], label='Path To Follow')
    print(f"final location (x,y): ({veh_state[0, -1]},{veh_state[1, -1]})")


#TEST 4
def main():
    fig, ax = plt.subplots()
    ax.plot(PathA[0, :], PathA[1, :],'r--')
    ax.plot(PathB[0, :], PathB[1, :],'r--')
    ax.plot(PathC[0, :], PathC[1, :],'r--')
    ax.plot(PathD[0, :], PathD[1, :],'r--')
    ax.plot(PathE[0, :], PathE[1, :],'r--')
    ax.plot(PathF[0, :], PathF[1, :],'r--')
    ax.plot(PathG[0, :], PathG[1, :],'r--')
    ax.plot(PathH[0, :], PathH[1, :],'r--')
    ax.plot(PathI[0, :], PathI[1, :],'r--')
    plt.ylim(-50, 350)
    plt.xlim(-200, 350)
    ax.set(title = "Desired Path",
    xlabel = "Distance North [m]",
    ylabel = "Distance East [m]")
    plt.ylim(-50, 350)
    plt.xlim(-200, 350)
    plt.figure() 
    
if __name__ == '__main__':
     print("Desired Path")
main()

def main():
    
    fig, axdis = plt.subplots()
    axdis.plot(PathASimul[0, :], PathASimul[1, :],'r--',label='Vehicle Path')
    axdis.plot(PathB[0, :], PathB[1, :],'r--')
    axdis.plot(PathC[0, :], PathC[1, :],'r--')
    axdis.plot(PathD[0, :], PathD[1, :],'r--')
    axdis.plot(PathE[0, :], PathE[1, :],'r--')
    axdis.plot(PathF[0, :], PathF[1, :],'r--')
    axdis.plot(PathG[0, :], PathG[1, :],'r--')
    axdis.plot(PathH[0, :], PathH[1, :],'r--')
    axdis.plot(PathI[0, :], PathI[1, :],'r--')
 
    axdis.plot(PathASimul[0, :], PathASimul[1,:], color = 'orange',label='Desired Path')
    axdis.plot(PathBSimul[0, :], PathBSimul[1, :], color = 'orange') 
    axdis.plot(PathCSimul[0,:], PathCSimul[1,:], color = 'orange')
    axdis.plot(PathDSimul[0,:], PathDSimul[1,:], color = 'orange')
    axdis.plot(PathESimulState[0, :], PathESimulState[1, :], color = 'orange')
    axdis.plot(PathFSimul[0, :], PathFSimul[1, :], color = 'orange')
    axdis.plot(PathGSimul[0,:], PathGSimul[1,:], color = 'orange')
    axdis.plot(PathHSimul[0,:], PathHSimul[1,:], color = 'orange')
    axdis.plot(PathISimulState[0, :], PathISimulState[1, :], color = 'orange')
    plt.ylim(-50, 350)
    plt.xlim(-200, 350)
    axdis.set(title = "Paths",
    xlabel = "Distance North [m]",
    ylabel = "Distance East [m]")
    plt.ylim(-50, 350)
    plt.xlim(-200, 350)
    leg = axdis.legend()
    plt.figure() 

    
if __name__ == '__main__':
     print("BAE 599 Course Algorithms")
main()