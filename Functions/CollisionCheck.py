#To create a path planning algorith based on the MoveToPoint function,
#including a collision check function, as well as a PID controller (spead check). 
#The obstacles are  defined manually. 

#Define obstacles
obstacles = ( )
#Simple circular obstacle
class Obstacle:
    def __init__(self, name, center, radius, color):
        self.name   = name
        self.center = center
        self.radius = radius
        self.color  = color
with_obstacles = True
if (with_obstacles):
    print("Using obstacles in environment ...")
    obj1 = Obstacle("A", ( -5.0,  0.0), 0.5 , [0.5,0.0,0.0])
    obj2 = Obstacle("B", (  5.0,  0.0), 0.5 , [1.0,0.0,0.0])
    obj3 = Obstacle("C", (  0.0, 12.0), 1.5 , [0.5,0.5,0.5])
    obj4 = Obstacle("D", ( 10.0,  5.0), 0.85, [1.0,0.5,0.5])
    obj5 = Obstacle("E", ( -5.0, 10.0), 0.75, [0.5,1.0,0.5])
    obj6 = Obstacle("F", (-10.0,  5.0), 1.5 , [0.5,0.5,1.0])

# Store a tuple of all obstacles
obstacles = (obj1, obj2, obj3, obj4, obj5, obj6)

# Simple path that moves collision free
path = [(0.286, -1.04), (0.5, -2.0), (1.0, -2.125), (1.5, -2.25),
       (1.48, -1.82),  (1.42, -1.42), (1.0, -0.75), (0.75, 1.0), (0.6, 1.5),
       (0.671, 2.18), (1.5, 2.18), (1.7, 1.8), (1.87, 1.6),
       (2.012, 1.369), (2.27, 0.0), (2.48, -1.26)]

# Simple collision checking for circular obstacles
def checkCollisions(self, objects):

        collision = False

        # Check all objects
        for obj in objects:

            #print "object ",obj.name

            # Against all links
            for link in self.links:

                # Vector from link base to object center
                dX = ( (obj.center[0] - link.base[0]), (obj.center[1] - link.base[1]) )

                dist = dX[0]*link.normal[0] + dX[1]*link.normal[1]

                # check distance from obstacle center to line along a link
                if (np.fabs(dist) < (link.width/2.0 + obj.radius)):
                    #print "potential collision"

                    # Calculate the projection of vector to obstacle along the link
                    dY = (dX[0] - dist*link.normal[0], dX[1]-dist*link.normal[1])

                    # Distance along the link
                    proj = dY[0]*np.cos(link.total_angle) + dY[1]*np.sin(link.total_angle)

                    #print "  dY=",dY
                    #print "  proj=",proj
                    if ((proj >= 0.0) and (proj <= (link.length+obj.radius))):
                        #print " Collision between link ",link.name," and obstacle ",obj.name, " at dist=",dist, " proj=",proj
                        return obj.color # return the first collision encountered
        return None

class path:
    """
    Class for path planning
    """
    class Node:
        """
        Calculate Node
        """
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]
        """
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.obstacle_list = obstacle_list
        self.node_list = []
        
    def planning(): 
        veh_state = np.array([[init_x], [init_y], [init_angle],
                      [init_V], [init_steer]])
        spead = 3 #particular spead
        desired_Spead = np.full(
        shape=iterations,
        fill_value=spead,
        dtype=np.int)
           for i in range(1, iterations):
                desired_V, desired_Steer = moveToPoint(veh_state[0, i - 1],
                                           veh_state[1, i - 1],
                                           veh_state[2, i - 1],
                                           desired_x, desired_y, kv, kh)
                new_veh_state = bicycleMS4(u=np.array([[desired_Steer], [desired_Spead[1]]]),
                               q=veh_state[:, -1][:, np.newaxis],
                               dt=dt, DT=dt, L=wheelBase, maxSteer=maxSteer)
                 veh_state = np.hstack((veh_state, new_veh_state[:, 1:]))

        return None  # cannot find path

def main():
    # Parameter
    obstacleList = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2)
    ]  # [x,y,size]
    
    movetopoint = path(start=[5, 5], goal=[5, 5],
              rand_area=[-2, 15], obstacle_list=obstacleList)
    
    PATH = path.planning()