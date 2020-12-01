import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

from Functions import DifferentialSteering, FieldCoverage, CollisionCheck
from Functions import KinematicCarModel, Maneuvers, MoveToPoint
from Functions import PositionTransformation, PurePursuitController

""" Draw the wheels of the vehicle"""    

Rear_front = 3.  # rear to front wheel
Width = 3.  # width of car
FrontEnd = 3.3  # distance from rear to vehicle front end
BackEnd = 1.0  # distance from rear to vehicle back end
Steer_Max = 0.6  # in [rad] - maximum steering angle
  
"""Vehicle rectangle vertices"""
  
VRX = [FrontEnd, FrontEnd, -BackEnd, -BackEnd, FrontEnd]
VRY = [Width / 3, -Width / 3, -Width / 3, Width / 3, Width / 3]

rx, ry = 1.,1.5