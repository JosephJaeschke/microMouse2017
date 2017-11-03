import numpy
import heapq
import serial
import pygame
import sys
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(24,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(25,GPIO.IN, pull_up_down=GPIO.PUD_UP)

#map parameters (rows x columns x fields)
#grid is 16x16 squares (elements) big
#map is represented by a 16x16 2d array of 1d arrays 5 big
#third dimension is the wall setup. True for wall False otherwise
#last element of wall array is visited. 1 for visited 0 otherwise
#[top,right,bottom,left,visited]
front=0 #ultrasonic sensor value. 1 if at a wall
left=0 #sensor pair values. 1 if at a wall
right=0 #sensor pair values. 1 if at a wall
facing=1 #0=up 1=right 2=down 3=left. DEFAULT FACING DOWN
map=numpy.zeros([16,16,5]) #initialize map to all 0's
arduino = serial.Serial('/dev/ttyACM0',9600) #'/dev/ttyACM0'
pygame.init()
clock=pygame.time.Clock()

def heur(point):
	return abs(point[0]-7.5)+abs(point[1]-7.5)

def setMapSpace(row,col): #set walls of an individual map square
    getSensors()
    if facing==0: #robot facing up
        map[row][col][0]=front
        map[row][col][3]=left
        map[row][col][1]=right
        map[row][col][2]=0
    elif facing==1: #robot facing right
        map[row][col][1]=front
        map[row][col][0]=left
        map[row][col][2]=right
        map[row][col][3]=0
    elif facing==2: #robot facing down
        map[row][col][2]=front
        map[row][col][1]=left
        map[row][col][3]=right
        map[row][col][0]=0
    else: #robot facing left
        map[row][col][3]=front
        map[row][col][2]=left
        map[row][col][0]=right
        map[row][col][1]=0

def getSensors(): #Set front,left,right using serial
    buf=arduino.read(6)
    if buf[1]=='N':
        return
    front=float(buf[1])
    right=float(buf[2])
    left=float(buf[3])
    return

def dfs(row,col): #recursive
    map[row][col][4]=1 #mark spot as visited
    setMapSpace(row,col) #set wall layout of space
    if map[row][col][0]==0 and map[row-1][col][4]==0: #try to dfs up
        moveUp()
        facing=0
        dfs(row-1,col)
        moveDown()
        facing=2
    if map[row][col][1]==0 and map[row][col+1][4]==0: #try to dfs right
        moveRight()
        facing=1
        dfs(row,col+1)
        moveLeft()
        facing=3
    if map[row][col][2]==0 and map[row+1][col][4]==0: #try to dfs down
        moveDown()
        facing=2
        dfs(row+1,col)
        moveUp()
        facing=0
    if map[row][col][3]==0 and map[row][col-1][4]==0: #try to dfs left
        moveLeft()
        facing=3
        dfs(row,col-1)
        moveRight()
        facing=1
    return

#[top,right,bottom,left,...]. 1 if there is a wall
#end at (8,8), (8,9), (9,8), (9,9)
def astar(): #return list of pts (row,col) to shortest path
    goal = set([(7,7),(7,8),(8,7),(8,8)])
    path=[]
    curPos=(1+heur((0,0)),1,(0,0),None) #cost+heur, cost, pos, parent
    heap=[] #fringe
    visited={} #dictionary for parents, key=node, value=parent
    heapq.heappush(heap,curPos)
    visited.setdefault((0,0),None)
    while curPos[2] not in goal and curPos[2] not in visited:
        curPos=heapq.heappop(heap)
        row=curPos[2][0]
        col=curPos[2][1]
        prevcost=curPos[1]
        prevheur=curPos[0]
        #add neighbors to fringe
        if map[row][col][0]==0 and not ((row-1,col) in visited): #up neighbor
            heapq.heappush(heap,(prevcost+1+heur(row-1,col),prevcost+1,(row-1,col),(row,col)))
            visited.setdefault((row-1,col),(row,col))
        if map[row][col][1]==0 and not ((row,col+1) in visited): #right neighbor
            heapq.heappush(heap,(prevcost+1+heur(row,col+1),prevcost+1,(row,col+1),(row,col)))
            visited.setdefault((row,col+1),(row,col))
        if map[row][col][2]==0 and not ((row+1,col) in visited): #down neighbor
            heapq.heappush(heap,(prevcost+1+heur(row+1,col),prevcost+1,(row+1,col),(row,col)))
            visited.setdefault((row+1,col),(row,col))
        if map[row][col][3]==0 and not ((row,col-1) in visited): #left neighbor
            heapq.heappush(heap,(prevcost+1+heur(row,col-1),prevcost+1,(row,col-1),(row,col)))
            visited.setdefault((row,col-1),(row,col))
    path.append(curPos[2]) #add end space to path
    element=visited[curPos[2]] #get parent of end space
    while not (element is None):
        path.append(element)
        element=visited[element]
    revpath=path[::-1]
    return revpath

def travel(thepath): #thepath in form ((0,0),(a,b)...,(y,z))
    prev=thepath.pop(0)
    while len(thepath)!=0:
        at=thepath.pop(0)
        if at[0]==prev[0]:
            if at[1]<prev[1]:
                moveLeft()
                facing=1
            else:
                moveRight()
                facing=3
        elif at[0]<prev[0]:
            moveUp()
            facing=2
        else:
            moveDown()
            facing=0
        prev=at
    halt()
    return

def moveUp():
    #print "UP"
    if facing==0:
        moveOne()
    elif facing==1:
        turnCCW()
        moveOne()
    elif facing==2:
        back()
        moveOne()
    else:
        turnCW()
        moveOne()
    return

def moveRight():
    #print "RIGHT"
    if facing==0:
        turnCW()
        moveOne()
    elif facing==1:
        moveOne()
    elif facing==2:
        turnCCW()
        moveOne()
    else:
        back()
        moveOne()
    return

def moveDown():
    #print "DOWN"
    if facing==0:
        back()
        moveOne()
    elif facing==1:
        turnCW()
        moveOne()
    elif facing==2:
        moveOne()
    else:
        turnCCW()
        moveOne()
    return

def moveLeft():
    #print "LEFT"
    if facing==0:
        turnCCW()
        moveOne()
    elif facing==1:
        back()
        moveOne()
    elif facing==2:
        turnCW()
        moveOne()
    else:
        moveOne()
    return

def turnCW():
    arduino.write("[D]\n")
    time.sleep(0.3)
    return

def turnCCW():
    arduino.write("[A]\n")
    time.sleep(0.3)
    return

def moveOne():
    arduino.write("[W]\n")
    time.sleep(0.3)
    return

def halt():
    arduino.write("[H]\n")
    time.sleep(0.3)
    return

def back():
    arduino.write("[S]\n")
    time.sleep(0.3)
    return

def main():
    spath=[]
    while True:
        input_state=GPIO.input(24)
        input_state2=GPIO.input(25)
        if input_state==False:
            #AUTONOMOUS MAPPING AND SOLVING MODE
            print("button1 pressed")
            dfs(0,0)
            halt()
            spath=astar()
            travel(spath)
            turnCW()
            turnCCW()
        if input_state2==False:
            #REMOTE CONTROL MODE
            print("button2 pressed")
            while True:
                keyPressed=pygame.key.get_pressed()
                if keyPressed[pygame.K_UP] or keyPressed[pygame.K_w]:
                    moveOne()
                elif keyPressed[pygame.K_LEFT] or keyPressed[pygame.K_a]:
                    turnCCW()
                elif keyPressed[pygame.K_RIGHT] or keyPressed[pygame.K_d]:
                    turnCW()
                elif keyPressed[pygame.K_q]:
                    pygame.quit()
                    sys.exit()

turnCW()
turnCCW()
main()
