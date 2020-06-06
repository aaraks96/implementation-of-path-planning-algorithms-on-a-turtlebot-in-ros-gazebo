import numpy as np
import cv2
import math
import matplotlib.pyplot as plt
import math
from heapq import *
import sys

class Obstacle:
    def __init__(self, shape, center, dim):
        self.shape = shape
        self.center = center
        self.dim = dim



# To check if the new point lies in the circle obstacle
def check_obstacle_circle(point, dimension, clearance, center, radius):
    #print('circle check')
    increase = dimension + clearance
    point_x = point[0]
    point_y = point[1]
    dist = np.sqrt((point_x - (center[0]))**2 + (point_y - (center[1]))**2)
    if dist <= radius + increase +1:
        #print("in obstacle circle")
        return True
    else:
        return False


# To check if the new point lies in the Square Obstacle
def check_obstacle_square(point, dimension, clearance, corner_pts):
    #print('square check')
    dist_check = dimension + clearance
    p1 = [corner_pts[0], corner_pts[1]]
    p2 = [corner_pts[2], corner_pts[3]]
    p3 = [corner_pts[4], corner_pts[5]]
    p4 = [corner_pts[6], corner_pts[7]]

    point_x = point[0]
    point_y = point[1]

    line1 = point_y - (p1[1] + dist_check)
    line2 = point_x - (p2[0] + dist_check)
    line3 = point_y - (p3[1] - dist_check)
    line4 = point_x - (p4[0] - dist_check)

    flag1 = False
    flag2 = False

    if line1 <= 0 and line3 >= 0:
        flag1 = True
    if line2 <= 0 and line4 >= 0:
        flag2 = True

    if flag1 and flag2 is True:
        #print("in obstacle square")
        return True
    else:
        return False



def check_obstacle_corner(point, dimension, clearance):
    increase = dimension + clearance
    if point[1] > 51-increase:
        return True
    elif point[1] < -51 + increase:
        return True
    elif point[0] <-56 + increase:
        return True
    elif point[0] >  56-increase:
        return True
    else: return False

def call_obstacle_checks(all_obstacles, point, dimension, clearance):
    for obstacle in all_obstacles:
        if obstacle.shape == 'circle':
            if check_obstacle_circle(point, dimension, clearance, obstacle.center, obstacle.dim):
                # print("i'm here, circle")
                return True

        elif obstacle.shape == 'rectangle':
            if check_obstacle_square(point, dimension, clearance, obstacle.dim):
                # print("i'm here, rectangle")
                return True

    # if check_obstacle_corner(point, dimension, clearance):
    #    return True
    if point[0] <=-10 and point[0] >=-50 and point[1]>=20 and point[1]<=50:
        return True
    
    return False

def generate_graph(all_obstacles,dimension,clearance):
    
    graph = {}

    for i in range(-56,56):
        for j in range(-51,51):
            graph[(i,j)] = {'visited': False, 'distance': np.inf, 'valid':True, 'parent':(0,0), 'theta':0, 'action':"no"}
            if call_obstacle_checks(all_obstacles,[i,j],dimension,clearance):
                graph[(i,j)]['valid'] = False
    return graph

            


##################################################################################################################
# The section above this was just to plot and check workspace, the section below will be the algorithm implementation
# A* implementation


def move(point, dimension, clearance, velocity, resolution, theta, wheel_radius=0.38, length=2.3):

    point_x = point[0]
    point_y = point[1]

    #print('resolution, move', resolution)

    theta = theta + (wheel_radius / length) * (velocity[0] - velocity[1]) * resolution
    point_x = point_x + (wheel_radius / 2) * (velocity[0] + velocity[1]) * math.cos(theta) * resolution
    point_y = point_y + (wheel_radius / 2) * (velocity[0] + velocity[1]) * math.sin(theta) * resolution
    new_point = (np.around(point_x), np.around(point_y))

    # if point_y > 0 and point_y < 150 and point_x > 0 and point_x < 250 and not (call_obstacle_checks(new_point, dimension, clearance)):

    if int(point_y) >= -51 and int(point_y) <= 51 and int(point_x) >= -56 and int(point_x) <= 56 :
        return new_point, theta
    else:
        #print('I am here')
        return None, None


def generate_node_location(action, current_point, dimension, clearance, velocity, resolution, theta):
    velocity_1 = velocity[0]
    velocity_2 = velocity[1]

    #print(velocity_1)
    #print(velocity_2)
    #print('theta', theta)
    #print('resolution, gener', resolution)

    if action == 'left_1':
        return move(current_point, dimension, clearance, [0, velocity_1], resolution, theta)
    if action == 'right_1':
        return move(current_point, dimension, clearance, [velocity_1, 0], resolution, theta)
    if action == 'straight_1':
        return move(current_point, dimension, clearance, [velocity_1, velocity_1], resolution, theta)
    if action == 'left_2':
        return move(current_point, dimension, clearance, [0, velocity_2], resolution, theta)
    if action == 'right_2':
        return move(current_point, dimension, clearance, [velocity_2, 0], resolution, theta)
    if action == 'straight_2':
        return move(current_point, dimension, clearance, [velocity_2, velocity_2], resolution, theta)
    if action == 'left_1_2':
        return move(current_point, dimension, clearance, [velocity_1, velocity_2], resolution, theta)
    if action == 'right_1_2':
        return move(current_point, dimension, clearance, [velocity_2, velocity_1], resolution, theta)





def cost_to_goal(point, goal):
    #point = np.array(point)
    #goal = np.array(goal_node_pos)

    euc_dist = (((point[0] - goal[0]) ** 2 + (point[1] - goal[1]) ** 2) ** 0.5)
    return euc_dist



def Astar(graph,start,goal,velocity, threshold, robot_dimension, robot_clearance):
    
    resolution = 1
    current_theta = 0.0
    visited = []
    queue = []
    actions = ["left_1", "right_1", "left_2", "right_2", "straight_1", "straight_2", "left_1_2", "right_1_2"]
    [goal_x,goal_y] = goal
    [start_x,start_y] = start
    graph[(start_x,start_y)]['visited'] = True
    graph[(start_x,start_y)]['distance'] = 0
    #graph[(start_x,start_y)]['parent'] = (start_x,start_y)
    graph[(start_x,start_y)]['theta'] = current_theta
    node_count = 1
    total_distance = cost_to_goal([start_x,start_y],[goal_x,goal_y]) + graph[(start_x,start_y)]['distance'] 

    heappush(queue,(total_distance,(start_x,start_y)))
    


    while(len(queue)!=0):
        current_point = heappop(queue)[1]
        

        #print("cr pt",current_point)
        if current_point == (goal_x,goal_y) or cost_to_goal(current_point, goal) <= threshold:
            print("goal reached")
            break
        current_theta = graph[current_point]['theta']
        for action in actions:
            new_point, theta = generate_node_location(action, current_point, robot_dimension, robot_clearance, velocity, resolution, current_theta)
            
            if new_point != None:
                if graph[new_point]['valid']==True:
                    base_cost = 1
                    if graph[new_point]['visited'] == False:

                        graph[new_point]['visited'] = True
                        
                        if node_count % 1 == 0:
                            #plt.plot(x, y, 'y.')
                            visited[:] = []
                            #plt.pause(0.01)
                        node_count+=1
                        graph[new_point]['action'] = action
                        #print(graph[new_point]['action']) 
                        graph[new_point]['parent'] = current_point
                        #print(graph[new_point]['parent'])
                        graph[new_point]['theta'] = theta
                        graph[new_point]['distance'] = graph[current_point]['distance'] + base_cost
                        total_distance = cost_to_goal([new_point[0],new_point[1]],[goal_x,goal_y]) + graph[new_point]['distance']
                        heappush(queue,(total_distance,new_point))
    
    path = [(current_point[0],current_point[1])]
    act = [graph[(current_point[0],current_point[1])]['action']]
    angles = [graph[(current_point[0],current_point[1])]['theta']]
    parent = (current_point[0],current_point[1])
    while parent!= (start_x,start_y):
        parent = graph[path[-1]]['parent']
        #print("parent",parent)
        acti =  graph[path[-1]]['action']
        angle = graph[path[-1]]['theta']
        angles.append(angle)
        act.append(acti)
        path.append(parent)
        #print("path",path)
        
    
    #min_distance = (graph[(goal_x, goal_y)]['distance'])
    return path, act, angles



def wrapper(start_node, goal_node, velocity, resolution):


    disc = 10
    dimension = 2
    clearance = 0
    threshold = 10
    start_node_pos = [start_node[0],start_node[1]]
    goal_node_pos = [goal_node[0], goal_node[1]]

    obstacle_1 = Obstacle('circle', (-1.65*disc, 4.60*disc), 0.405*disc)
    obstacle_2 = Obstacle('circle', (-1.17*disc, 2.31*disc), 0.405*disc)
    obstacle_3 = Obstacle('circle', (-4.05*disc, 3.25*disc), 0.8*disc)
    obstacle_19 = Obstacle('circle', (-2.45*disc, 3.25*disc), 0.8*disc)
    obstacle_20 = Obstacle('rectangle', (0, 0), [-4.05*disc, 4.05*disc, -2.4*disc, 4.05*disc, -2.4*disc, 2.45*disc, -4*disc, 2.45*disc])
    obstacle_4 = Obstacle('rectangle', (0, 0), [2.77*disc, 5.05*disc, 3.63*disc, 5.05*disc, 3.63*disc, 3.22*disc, 2.77*disc, 3.22*disc])
    obstacle_5 = Obstacle('rectangle', (0, 0), [4.28*disc, 5.05*disc, 4.71*disc, 5.05*disc, 4.71*disc, 4.14*disc, 4.28*disc, 4.14*disc])
    obstacle_6 = Obstacle('rectangle', (0, 0), [1.89*disc, 1.92*disc, 5.55*disc, 1.92*disc, 5.55*disc, 1.16*disc, 1.89*disc, 1.16*disc])
    obstacle_7 = Obstacle('rectangle', (0, 0), [4.97*disc, 0.605*disc, 5.55*disc, 0.605*disc, 5.55*disc, -0.56*disc, 4.97*disc, -0.56*disc])
    obstacle_8 = Obstacle('rectangle', (0, 0), [4.64*disc, -0.56*disc, 5.55*disc, -0.56*disc, 5.55*disc, -1.42*disc, 4.64*disc, -1.42*disc])
    obstacle_9 = Obstacle('rectangle', (0, 0), [4.97*disc, -2.09*disc, 5.55*disc, -2.09*disc, 5.55*disc, -3.26*disc, 4.97*disc,-3.26*disc])
    obstacle_10 = Obstacle('rectangle', (0, 0), [3.72*disc, -3.95*disc, 5.55*disc, -3.95*disc, 5.55*disc, -4.71*disc, 3.72*disc, -4.71*disc])
    obstacle_11 = Obstacle('rectangle', (0, 0), [1.3*disc, -4.71*disc, 5.55*disc, -4.71*disc, 5.55*disc, -5.05*disc, 1.3*disc, -5.05*disc])
    obstacle_12 = Obstacle('rectangle', (0, 0), [2.24*disc, -4.12*disc, 3.41*disc, -4.12*disc, 3.41*disc, -4.71*disc, 2.24*disc, -4.71*disc])
    obstacle_13 = Obstacle('rectangle', (0, 0), [-0.81*disc, -3.18*disc, 1.93*disc, -3.18*disc, 1.93*disc, -4.71*disc, -0.81*disc, -4.71*disc])
    obstacle_14 = Obstacle('circle', (-1.65*disc, -4.6*disc), (0.81 / 2)*disc)
    obstacle_15 = Obstacle('circle', (-1.17*disc, -2.31*disc), (0.81 / 2)*disc)
    obstacle_16 = Obstacle('rectangle', (0, 0), [-1.17*disc, -0.07*disc, -0.25*disc, -0.07*disc, -0.26*disc, -1.9*disc, -1.17*disc, -1.9*disc])
    obstacle_17 = Obstacle('rectangle', (0, 0), [-0.26*disc, -1.64*disc, 1.57*disc, -1.64*disc, 1.57*disc, -2.4*disc, -0.26*disc, -2.4*disc])
    obstacle_18 = Obstacle('rectangle', (0, 0), [2.29*disc, -1.21*disc, 3.815*disc, -1.21*disc, 3.81*disc, -2.38*disc, 2.29*disc, -2.38*disc])

    all_obstacles = [obstacle_1, obstacle_2, obstacle_3, obstacle_4, obstacle_5, obstacle_6, obstacle_7, obstacle_8,
                     obstacle_9, obstacle_10, obstacle_11, obstacle_12, obstacle_13, obstacle_14, obstacle_15,
                     obstacle_16, obstacle_17, obstacle_18, obstacle_19, obstacle_20]


    #check start and goal in obs
    if call_obstacle_checks(all_obstacles, start_node_pos, dimension, clearance) or call_obstacle_checks(all_obstacles, goal_node_pos, dimension, clearance):
        
        return None, None, None

    if cost_to_goal(start_node_pos,goal_node_pos)<= threshold:
        
        return 0,0,0

    graph = generate_graph(all_obstacles,dimension,clearance)

    #velocity = [4, 2]
    #resolution = 3
    #theta = 0.0
    # start_node.theta = 1.57  # for 90 degree clockwise

    points, result, angles = Astar(graph,start_node,goal_node,velocity, threshold, dimension, clearance)
    
    return points, result, angles


# start_node = [0,0]
# goal_node = [100, 400]
# velocity = [10,5]
# resolution = 1

# path, res, g = wrapper(start_node, goal_node, velocity, resolution)
# # print('pts', path)
# print('res', res)

# points = [x for x in g.keys() if not (g[x]['valid'])]

# x = [i[0] for i in points]
# y = [i[1] for i in points]
# for i in points:
#     if start_node[0] == i[0] and start_node[1] == i[1]:
#         print("Start point inside obstacle, exiting")
#         sys.exit(0)
#     if goal_node[0] == i[0] and goal_node[1] == i[1]:
#         print("Goal point inside obstacle, exiting")
#         sys.exit(0)




# x = [i[0] for i in points]
# y = [i[1] for i in points]
# plt.xlim(right=550)
# plt.ylim(top=500)
# plt.plot(x, y)
# plt.plot(0, 0)
# plt.plot(goal_node[0], goal_node[1])
# x = [i[0] for i in path]
# y = [i[1] for i in path]
# plt.plot(x, y)
# plt.show()



"""
if result is not None:
    nodes_list = trackback(result)
    # image = plot_track(nodes_list,image)

    for elem in nodes_list:
        x = elem.point[0]
        y = elem.point[1]
        print('in trackback')
        print(elem.action)

else:
    print("result is none")
"""

