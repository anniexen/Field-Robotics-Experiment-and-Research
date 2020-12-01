#Required Libraries
from math import sqrt, cos, sin, tan, pi
import matplotlib.pyplot as plt
import numpy as np  
from scipy.spatial.transform import Rotation as Rot

def transl2(x_tran,y_tran):
    """Creates a homogeneous 2D translation matrix
       
    Parameters
    ----------
    x_tran : float
        The translation in the x-direction
    y_tran : float
        The translation in the y-direction 
     """   
    import numpy as np

    TranMatrix = np.zeros((3,3))
    TranMatrix[0][0]=1
    TranMatrix[0][2]=x_tran
    TranMatrix[1][1]=1
    TranMatrix[1][2]=y_tran
    TranMatrix[2][2]=1
    return TranMatrix
    print(transl2(2.0,3.0))

    """
    Returns
    ----------
    res_mat: np.array
        A homogenous matrix for the requested translation
    """
    
def trot2(angle, deg_or_rad = "rad"):
    """Creates a 2D rotation matrix
    
    Parameters
    ----------
    angle : float
        The angle of rotation for the transformation
    deg_or_rad : str, optional, default = "rad"
        A label for the frame printed near its origin
    
    Returns
    ----------
    res_mat: np.array
        A homogenous matrix for the requested rotation
    
    """
    
    import numpy as np
    
    if deg_or_rad == "deg":
        angle = angle*np.pi / 180
    
   
    RotMatrix = np.array([[np.cos(angle), -1*np.sin(angle), 0], 
                        [np.sin(angle), np.cos(angle), 0],
                        [0,0,1]])
    return RotMatrix
    print(trot2(30,"deg"))
    print(transl2(2, 3)@trot2(30,"deg"))
          
    
def trplot2(T, frame_color='r', frame_Name=False, frame_length = 1, ax = False, axis = [-1, 5, -1, 5]):
    """Plots a 2D reference frame
    
    At a minimum, a homogeneous transformation matrix for the frame must be
    provided. Other parameters are optional.
    
    Parameters
    ----------
    T: np.array
    frame_color : str, optional
        The frame color using the Matplotlib color notation
    frame_Name : str, optional
        A label for the frame printed near its origin
    frame_length, int, optional
        The length of the arms of the frame
    ax: matplotlib.axes.Axes object, optional
        A new figure is created if ax is not provided. Otherwise the function
        will plot on the provided axis, ax.
    axis: list, optional
        A list with the min and max values for the figure in the form:
        [xmin, xmax, ymin, ymax]. Default is from -1 to 5 for both x and y.
    
    """

    import numpy as np
    import matplotlib.pyplot as plt
    
    frame = np.array([[0,0, frame_length],
                      [frame_length, 0 , 0],
                      [1,1,1]])
    res_frame = T@frame
    if ax == False:
        fig, ax = plt.subplots(figsize=(5,5))
    ax.plot()
    ax.set_xlim(axis[0], axis[1])
    ax.set_ylim(axis[2], axis[3])
    ax.grid(True)
    
    ax.annotate("", xy= (res_frame[0, 0], res_frame[1,0]),
            xytext= (res_frame[0, 1],  res_frame[1,1]),
            arrowprops=dict(arrowstyle="->", color= frame_color, shrinkA=0, 
                            shrinkB=0),
            fontsize=16)
    ax.annotate("", xy= (res_frame[0, 2], res_frame[1,2]),
            xytext=(res_frame[0, 1],  res_frame[1,1]),
            arrowprops=dict(arrowstyle="->", color= frame_color, shrinkA=0, 
                            shrinkB=0),
            fontsize=16)
    
    if frame_Name == False:
       ax.annotate(r"$x$", xy = (res_frame[0,2],res_frame[1,2]), #Annotate x
       horizontalalignment='left', verticalalignment='bottom')
       ax.annotate(r"$y$".format(frame_Name), xy = (res_frame[0,0],res_frame[1,0]), #Annotate y
       horizontalalignment='left', verticalalignment='bottom')
    else:
       ax.annotate("{"+"{}".format(frame_Name)+"}", xy = (res_frame[0,1],res_frame[1,1]), #Annotate the orign
       horizontalalignment='right', verticalalignment='top')
       ax.annotate(r"$x_{{{}}}$".format(frame_Name), xy = (res_frame[0,2],res_frame[1,2]), #Annotate x
       horizontalalignment='left', verticalalignment='bottom')
       ax.annotate(r"$y_{{{}}}$".format(frame_Name), xy = (res_frame[0,0],res_frame[1,0]), 
       horizontalalignment='left', verticalalignment='bottom')
    return ax



     
#Mobile Robot Illustration  

#Some background...  
#The position and orientation of the robot at any instant of time are 
#described by the vector q = [x, y, φ]T, where (x, y) denote the position of point
#o (the center of the axis of the driven wheels) and φ is 
#the orientation of the robot in the global coordinate frame.

""" Draw an arrow to show direction of the vehicle"""    
def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """Plot arrow."""
    if not isinstance(x, float):
        for (i_x, i_y, i_yaw) in zip(x, y, yaw):
            plot_arrow(i_x, i_y, i_yaw)
    else:
        plt.arrow(x, y, length * cos(yaw), length * sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width, alpha=0.4)   
    
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

def plot_car(x, y, yaw, steer=0.0, truckcolor="-k"):  # pragma: no cover
    """Plots a vehicle representation  at a given position
    
    Parameters
    ----------
    x : float
        The vehicle location in the x-direction
    y : float
        The vehicle location in the y-direction
    yaw : float
        The pose angle for the vehicle in radians
    steer : float, optional, default = 0.0
        Angle at which to rotate the front (steering) wheels
    truckcolor: str, optional, default = "-k"
        The line format (both color and style in Matplotlib representation)
        for the outline of the vehicle.
    
    """
    c, s = cos(yaw), sin(yaw)
    print(c,s)
    rot = Rot.from_euler('z', -yaw).as_matrix()[0:2, 0:2]
    car_outline_x, car_outline_y = [], []
    for rx, ry in zip(VRX, VRY):
        converted_xy = np.stack([rx, ry]).T @ rot
        car_outline_x.append(converted_xy[0]+x)
        car_outline_y.append(converted_xy[1]+y)

    arrow_x, arrow_y, arrow_yaw = c * 1.5 + x, s * 1.5 + y, yaw
    plot_arrow(arrow_x, arrow_y, arrow_yaw)

    plt.plot(car_outline_x, car_outline_y,truckcolor)
  
#Test example    
def main():
    x, y, yaw = 3., 3.,9. #Insert variables x,y,yaw
    plt.axis('equal')
    plot_car(x, y, yaw, truckcolor="r")
    plt.show()

if __name__ == '__main__':
    main()