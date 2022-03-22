## ------------------------------------------------------------------------------------------
#                                   Dijkstra Algorithm
## ------------------------------------------------------------------------------------------

# Author: Jai Sharma & Divyansh Agrawal
# Course: ENPM661 - Planning for Autonomous Robots
# Assignment: Project 2 Phase 1 


#%%------------------------------------------------------------------------------------------
#                                    Step 0 --> Import Libraries
## ------------------------------------------------------------------------------------------
import heapq


import numpy as np
import math
import cv2
import pygame
import time

import matplotlib.pyplot as plt


#%%------------------------------------------------------------------------------------------
#                         Step 1 --> Fixed Inputs and Initialization
## ------------------------------------------------------------------------------------------

# Variables That Remain Constant 
MapWidth = 250
MapHeight = 400
MapWidth_Double = MapWidth*2
MapHeight_Double = MapHeight*2

stepSz = 1 
radius = 1 
clearance = 1
buffer = clearance + radius

# Initial orientations are set to 0 degrees --> facing East
thetai = 0 
thetag = 0 

# Create a 3D - Matrix to check for Duplicate Nodes 
rows = MapHeight_Double + 1      # (0.5 units increments)
columns = MapWidth_Double + 1    # (0.5 units increments)
thetaLayer = 12                      # (30 deg increments)
M=np.zeros((rows,columns,thetaLayer))

# Create a Dictionary to relate index to corresponding theta (deg)
thetaDict = {0:0,30:1,60:2,90:3,120:4,150:5,180:6,
             210:7,240:8,270:9,300:10,330:11,360:0}

#%%------------------------------------------------------------------------------------------
#                          Step 2 -->  User Inputs nd Initialization
## ------------------------------------------------------------------------------------------

# Request Input from User 
print("Note: All User Inputs Must be Integers")
print("x coordinates must be Smaller than ", MapWidth)
print("y coordinates must be Smaller than ", MapHeight, "\n")

Xi = int(input("x coordinate of START node -->  "))
Yi = int(input("y coordinate of START node -->  "))

Xg = int(input("x coordinate of GOAL node -->  "))
Yg = int(input("y coordinate of GOAL node -->  "))

# Test Case Inputs -------------------------
# Xi = 125
# Yi = 125
# thetai = 0 # put multiple of 30 deg

# Xg = 350
# Yg = 125
# ------------------------

# Initialize Nodes
start = (Xi,Yi,thetai) 
goal =  (Xg,Yg,thetag) 


#%%------------------------------------------------------------------------------------------
#                                 Step 2 -->  Basic Helper Functions
## ------------------------------------------------------------------------------------------

 # round input number to closest 0.5
def Rounding(input):
    rounded = 2 * round(input) / 2
    return (rounded)

 # calculates Eucilidean distance between two nodes
def Euclidean(node1,node2):  
    x1, x2 = node1[0], node2[0]
    y1, y2 = node1[1], node2[1]
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)  
    return (distance)  


#%%------------------------------------------------------------------------------------------
#                                      Step 3 -->  Define Actions
## ------------------------------------------------------------------------------------------

def Movement(node, orientation):
    x_new = (stepSz)*np.cos(np.deg2rad(orientation)) + node[0]
    y_new = (stepSz)*np.sin(np.deg2rad(orientation)) + node[1]
    new_node = (round(x_new,2),round(y_new,2))
    
    # check if node is within Map Limits
    if new_node[0]>=0.00 and new_node[0]<= 401.00 and new_node[1]>=0.00 and new_node[1]<= 251.00:
        return(new_node,True)
    else:
        return(node,False)
    
        
#%%------------------------------------------------------------------------------------------
#                                    Step 4 -->  Obstacle Mapping
## ------------------------------------------------------------------------------------------

map_cord = []
map_cord_double = []
obstacle_cord=[]
obstacle_cord_display=[]

# All points on map (unit increment of 1.0)
for y in range(MapWidth + 1): 
    for x in range(MapHeight + 1): 
        map_cord.append((x,y))    
        
# All points on map (unit increment of 0.5)
for y in range(MapWidth_Double + 1): 
    for x in range(MapHeight_Double + 1): 
        map_cord_double.append((Rounding(x/2), Rounding(y/2)))
    
# Build Obstacle Space for Path Planning
for pts in map_cord_double:
    x, y = pts[0], pts[1]
    # Circle Obstacle
  
    if((x-300)**2 + (y-175)**2 <= (40 + buffer)**2):
        obstacle_cord.append((x,y))
                
    # Hexagon Obstacle
    if x > (165 - buffer) and x < (235 + buffer):
        if y > (50 - buffer) and y < (150 + buffer):
            if (y - 0.577*x < (24.97 + buffer)): 
                if (y - 0.55*x > (-50 - buffer)):
                    if (y + 0.577*x < (255.829 + buffer)): 
                        if (y + 0.548*x > (169.314 - buffer)):
                            obstacle_cord.append((x,y))
    # Weird Shape Obstacle
    if y > 90  and x < 120:
        if y + 1.232*x > (229.348 - buffer):
            if y - 0.316*x < (173.608 + buffer):
                if (y - 0.857*x > (114.29 - buffer)) or (y + 3.2*x < (436 + buffer)):
                    obstacle_cord.append((x,y))

# Build Obstacle Space for Map Visualization
for pts in map_cord:
    x, y = pts[0], pts[1] 
    # Circle Obstacle
    if((x-300)**2 + (y-175)**2 <= (40)**2):
        obstacle_cord_display.append((x,y))

                
    # Hexagon Obstacle
    if x > 165 and x < 235 and y > 50 and y < 150:
            if (y - 0.577*x < 24.97): 
                if (y - 0.55*x > -50):
                    if (y + 0.577*x < 255.829): 
                        if (y + 0.548*x > 169.314):
                            obstacle_cord_display.append((x,y))
    # Weird Shape Obstacle
    if y > 90 and x < 120:
        if y + 1.232*x > 229.348:
            if y - 0.316*x < 173.608:
                if (y - 0.857*x > 114.29) or (y + 3.2*x < 436):
                    obstacle_cord_display.append((x,y))
                    
# Visualization to confirm correct placement of plots on Map               
listx, listy = [], []
for i in obstacle_cord:
    listx.append(i[0])
    listy.append(i[1])
plt.scatter(listx,listy, s=0.1,c='blue')
plt.axis([0,400,0,250])
plt.title('Obstacle Space with Buffer Space')
plt.grid(which='both')
plt.show()

listx_viz, listy_viz = [], []
for i in obstacle_cord_display:
    listx_viz.append(i[0])
    listy_viz.append(i[1])
plt.scatter(listx_viz,listy_viz,s=5,c='blue')
plt.axis([0,400,0,250])
plt.title('Obstacles on the Map')
plt.grid(which='both')
plt.show()

#%%------------------------------------------------------------------------------------------
#                                   Step 5 -->  Check Obstacle Space
## ------------------------------------------------------------------------------------------

# checks if input (neighbour node) is within Obstacle Space or not
def obstacleCheck(node):
    if node in obstacle_cord:
        print('node is in obstacle space', node)
        return(False)
    else:
        return(True)

#%%------------------------------------------------------------------------------------------
#                                      Step 7 -->  Map Generation
## ------------------------------------------------------------------------------------------

# uses position and orientation data of node to get neighbours and build links 
def getGraph(node,degrOre): 
    w,h = 401,251
    i,j = node[0], node[1] 

    # only find neighbours if the node is within the map
    if i <= w and j <= h and i>=0 and j>=0:
        
        costs = {}
        # zero
        zero = Movement(node, degrOre + 0)[0]#0
        if zero[0]>=0 and zero[1]>=0 and zero[0]<=w and zero[1]<=h:
            costs[zero] = (1, degrOre)
            
        thirty = Movement(node, degrOre + 30)[0]#30
        if thirty[0]>=0 and thirty[1]>=0 and thirty[0]<=w and thirty[1]<=h: 
            costs[thirty] = (1, degrOre + 30)
            
        sixty = Movement(node, degrOre + 60)[0]#60
        if sixty[0]>=0 and sixty[1]>=0 and sixty[0]<=w and sixty[1]<=h:
            costs[sixty] = (1, degrOre + 60)
            
        minusSixty = Movement(node, degrOre - 60)[0]#-60
        if minusSixty[0]>=0 and minusSixty[1]>=0 and minusSixty[0]<=w and minusSixty[1]<=h:
            costs[minusSixty] = (1, degrOre - 60)
            
        minusThirty = Movement(node, degrOre - 30)[0]#-30
        if minusThirty[0]>=0 and minusThirty[1]>=0 and minusThirty[0]<=w and minusThirty[1]<=h:
            costs[minusThirty] = (1, degrOre - 30)
            
        
        costCopy = costs.copy()
        for k,v in costCopy.items():
            if k == node:
                del costs[k]
        return(costs)
    
    else:
        print('node not in map')
        pass

#%%------------------------------------------------------------------------------------------
#                                      Step 8 -->  BackTracking
## ------------------------------------------------------------------------------------------
        
# switching the goal and start nodes within the Backtracking function
def BackTracking(backtrackingDict, goal, start):
    
    ListBacktrack = []
    ListBacktrack.append(start)
    
    # while the goal is not found
    while goal!=[]:
        for key, val in backtracking.items():            
            for __, val2 in val.items():
                if key == start:
                    if val2 not in ListBacktrack:
                        ListBacktrack.append(start)
                    start = val2
                    #checking if node is goal (actual Start)
                    if val2 == goal:
                        goal = []
                        break      
    #returns the backtracked list
    return (ListBacktrack)


#%%------------------------------------------------------------------------------------------
#                                      Step 10 -->  A Star Function
## ------------------------------------------------------------------------------------------
        
OpenList = {}
backtracking = {}
ClosedList = []         # save visited nodes as list of lists

def A_Star(start,goal):
    
    global backtracking
    global ClosedList
    global OpenList

    if (goal in obstacle_cord) or (start in obstacle_cord):
        print('Start Node or Goal Node is in Obstacle Space')
        OpenList= 0
        backtracking = 0
        rounded_N_node = 0
        
    else:
        OpenList[start]=0
        priority_queue = [(0,start,thetai)]
        Goal_Reached = False # check to leave while loop
        i = 0
        while len(priority_queue)>0 and Goal_Reached == False:
            i += 1
            totalC, node, direction = heapq.heappop(priority_queue)  # pops new parent node, based on TotalCost
            print('================================================')
            print('New Node:  ' , node)
            print('New Orientation:  ', direction, 'deg')
            
            # if parent node not in Obstacle Space
            if obstacleCheck(node)==True:
                neighbours = getGraph(node,direction)
                print('Neighbour Nodes are:   ', neighbours)

                # add Neighbour Nodes to OpenList with cost Infinity
                for key, __ in neighbours.items():
                    OpenList[key]=math.inf
                
                graph_list = [] # holds (neighbour node, costs) 
                for key,cost_value in neighbours.items():
                    graph_list.append((key,cost_value))
                    
                # compare the value of the current distance and 
                if totalC > OpenList[node]:
                    continue
                
                for N_node, cost in graph_list:
                    # Calculate costs to the Node
                    Cost2Come = neighbours[N_node][0] # g(n)
                    Cost2Go = Euclidean(N_node,goal)  # h(n)
                    TotalCost = Cost2Come + Cost2Go   # f(n)
  
                    if TotalCost < OpenList[N_node]:                        
                        rounded_N_node = (Rounding(N_node[0]),Rounding(N_node[1]))
                        try:
                            # indexing input orientation for matrix entry
                            direction = ((direction) % 360)

                            # if the neighbour node is not already explored
                            if rounded_N_node not in ClosedList:
                                # checks if duplicate node
                                if M[int(2*rounded_N_node[0])][int(2*rounded_N_node[1])][thetaDict[direction]]==0:
                                    M[int(2*rounded_N_node[0])][int(2*rounded_N_node[1])][thetaDict[direction]]=1
                                    
                                    ClosedList.append(rounded_N_node)
                                    backtracking[rounded_N_node]={}
                                    backtracking[rounded_N_node][TotalCost] = node
                                    
                                    OpenList[rounded_N_node]=TotalCost
                                    direction = neighbours[N_node][1]
                                    heapq.heappush(priority_queue, (TotalCost, rounded_N_node, direction))
                                    
                                    # if the distance between the node and goal is less than 1.5 (radius)
                                    if ((rounded_N_node[0]-goal[0])**2 + (rounded_N_node[1]-goal[1])**2 <= (1.5)**2):
                                        print('GOAL Node Reached !!')
                                        Goal_Reached = True
                                        break
                                    else:
                                        pass
                        except:
                            pass

    print('=================================')
    print('Total Iterations:', i)         
    print('=================================')
    
    return(OpenList,backtracking,ClosedList,rounded_N_node)      


#%%------------------------------------------------------------------------------------------
#                                      Step 11 -->  Call Required Functions
## ------------------------------------------------------------------------------------------

start_time = time.time()

#%% Call A-Star Function

# goalMin1: need the node right before goal node to initiate backtracking
OpenList,backtracking,ClosedList, goalMin1 = A_Star(start,goal)

# print('Closed List:', ClosedList)
# print('=================================')
# print('BackTracking is:', backtracking)
# print('=================================')


#%% Call BackTracking Function

backtracked_final = BackTracking(backtracking,start,goalMin1)
# print('BackTrack Final is:', backtracked_final)


print("Total to Run Algorithm (s) : ",time.time() - start_time)

#%%------------------------------------------------------------------------------------------
#                                      Step 12 -->  Visualization
## ------------------------------------------------------------------------------------------


blank_canvas = np.zeros((251,401,3),np.uint8) 

for c in obstacle_cord_display: 
    x = c[1]
    y = c[0]
    blank_canvas[(x,y)]=[0,0,255] 
    
blank_canvas = np.flipud(blank_canvas) # flip to correct orientation
backtrack_canvas = blank_canvas.copy()  # canvas to plot Backtracking Path
visited_canvas = blank_canvas.copy()  # canvas to plot Closed List

# backtracked path
for path in backtracked_final:    
    x = int(path[0])
    y = int(path[1])
    backtrack_canvas[(250 - y,x)]=[255,0,0]
backtrack_canvas_disp = cv2.resize(backtrack_canvas,(800,500))
cv2.imshow('Backtracked Plot',backtrack_canvas_disp)
cv2.waitKey(0)
cv2.destroyAllWindows()

# visited nodes
for path in ClosedList:
    x = int(path[0])
    y = int(path[1])
    visited_canvas[(250 - y,x)]=[255,255,0] 
visited_canvas_disp = cv2.resize(visited_canvas,(800,500))
cv2.imshow('Visited Nodes',visited_canvas_disp) 
cv2.waitKey(0)
cv2.destroyAllWindows()

#%%------------------------------------------------------------------------------------------
#                                      Step 13 -->  Pygame Animation
## ------------------------------------------------------------------------------------------
# https://realpython.com/pygame-a-primer/

pygame.init()

gameDisplay = pygame.display.set_mode((401,251),pygame.FULLSCREEN)
pygame.display.set_caption('A-Star Path --> Animation')
surf = pygame.surfarray.make_surface(visited_canvas)
clock = pygame.time.Clock()

running = True
while running:
    for event in pygame.event.get(): 
        if event.type == pygame.QUIT:  
            running = False   
    gameDisplay.fill((25,25,25))
    for path in ClosedList: #  Visited Nodes 
        if path not in visited_canvas:
            pygame.time.wait(1)
            x = path[0]
            y = abs(250-path[1])
            pygame.draw.rect(gameDisplay, (0,255,25), [x,y,1,1])
            pygame.display.flip()
    for path in backtracked_final: #  Backtracked Path 
        pygame.time.wait(1)
        x = path[0]
        y = abs(250-path[1])
        pygame.draw.rect(gameDisplay, (0,25,255), [x,y,1,1])
        pygame.display.flip()
    running = False
    
pygame.quit()

print('============================')
print('  Program Fully Executed  ')
print('============================')




