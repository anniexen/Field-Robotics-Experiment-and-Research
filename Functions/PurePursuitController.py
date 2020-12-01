import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math


def proportional_control(target, current):
    a = Kp * (target - current)
    return a


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


def pure_pursuit_steer_control(state, trajectory, pind):
    ind, Lf = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    return delta, ind


def dist(x1, y1, x2, y2):
    d = np.sqrt(np.float_power(x1 - x2, 2)
                + np.float_power(y1 - y2, 2))
    return d

def transl2(x_tran,y_tran): 
    import numpy as np

    TranMatrix = np.zeros((3,3))
    TranMatrix[0][0]=1
    TranMatrix[0][2]=x_tran
    TranMatrix[1][1]=1
    TranMatrix[1][2]=y_tran
    TranMatrix[2][2]=1
    return TranMatrix
    print(transl2(2.0,3.0))
    
def trot2(angle, deg_or_rad = "rad"): 
    import numpy as np
    
    if deg_or_rad == "deg":
        angle = angle*np.pi / 180
    
   
    RotMatrix = np.array([[np.cos(angle), -1*np.sin(angle), 0], 
                        [np.sin(angle), np.cos(angle), 0],
                        [0,0,1]])
    return RotMatrix
    print(trot2(30,"deg"))
    print(transl2(2, 3)@trot2(30,"deg"))
    

def purePursuitController(q=np.array([[0], [0], [-1.57], [1], [0]]),
                          L=2.5, ld=10,
                          path=np.array((np.linspace(0, 10, 11),
                                         np.linspace(0, 10, 11)))):

    robotX = q[0, 0]
    robotY = q[1, 0]
    robotAngle = q[2, 0]
    min_ld_index = 0
    min_XTE_index = 0
    # Calculate the first point and save as the initial minimum distance
    # We are calculating a minimum to ld and a minimum to robot.
    pathX = path[0, 0]
    pathY = path[1, 0]
    distanceMinld = np.abs(np.sqrt(np.float_power(robotX - pathX, 2)
                                   + np.float_power(robotY - pathY, 2)) - ld)
    distanceMin = np.sqrt(np.float_power(robotX - pathX, 2)
                          + np.float_power(robotY - pathY, 2))
    path_len = path.shape[1]
    # Check every point in the path to see which provides the minXTE and which
    # is closest to ld from the robot.
    for i in range(path_len):
        pathX = path[0, i]
        pathY = path[1, i]
        distance = np.sqrt(np.float_power(robotX - pathX, 2)
                           + np.float_power(robotY - pathY, 2))
        if (distance < distanceMin):
            min_XTE_index = i
            distanceMin = distance
        if (np.abs(distance - ld) < distanceMinld):
            min_ld_index = i
            distanceMinld = np.abs(distance - ld)

    # To calculate ey, express our path in the robot frame
    fShiftAngle = -1 * robotAngle
    fShiftX = -1 * robotX
    fShiftY = -1 * robotY
    matTRbad = np.array([[np.cos(fShiftAngle), -1 * np.sin(fShiftAngle), fShiftX],
                         [np.sin(fShiftAngle), np.cos(fShiftAngle), fShiftY],
                         [0, 0, 1]])
    matT = np.array([[fShiftX],
                     [fShiftY]])
    matR = np.array([[np.cos(fShiftAngle), -1 * np.sin(fShiftAngle)],
                     [np.sin(fShiftAngle), np.cos(fShiftAngle)]])
    path_robot_framebad = matTRbad @ np.vstack((path, np.ones((1, path.shape[1]))))
    path_robot_frameT = matT + path
    path_robot_frameTR = matR @ path_robot_frameT
    ey = path_robot_frameTR[1, min_ld_index]
    steerAngle = np.arctan((2*ey*L)/(np.float_power(ld, 2)))
    return steerAngle, distanceMin          
