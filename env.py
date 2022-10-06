import random
import math

import numpy as np
import pygame


class RRTmap:
    def __init__(self,start,goal,mapDimensions,obsdim,obsnum):
        self.start = start
        self.goal = goal
        self.MapDimensions = mapDimensions
        self.mapwidth,self.mapheight = mapDimensions
         #display window settings
        self.MapWindowName='RRT path planing'
        pygame.display.set_caption(self.MapWindowName)
        self.map=pygame.display.set_mode((self.mapwidth,self.mapheight))
        self.map.fill((255,255,255))
        self.nodeRad=2
        self.nodethickness = 0
        self.edgethickness = 1

        self.obstacle=[]
        self.obsdim=obsdim
        self.obssum=obsnum

        #colors
        self.grey=(70,70,70)
        self.blue=(0,0,255)
        self.green=(0,255,0)
        self.red=(255,0,0)
        self.white=(255,255,255)



    def drawMap(self,obstacales):
        pygame.draw.circle(self.map, self.green, self.start, self.nodeRad + 5, 0)
        pygame.draw.circle(self.map, self.green, self.start, self.nodeRad + 25, 0)
        pygame.draw.circle(self.map, self.red, self.goal, self.nodeRad + 20, 1)
        pygame.draw.circle(self.map, self.red, self.goal, self.nodeRad + 40, 1)
        self.drawObs(obstacales)




    def drawpath(self, path):
        for node in path:
            pygame.draw.circle(self.map, self.red, node, self.nodeRad+3,0)
        return









    def drawObs(self,obstacales):
        obstaclesList=obstacales.copy()
        while(len(obstaclesList)>0):
            obstacle=obstaclesList.pop(0)
            pygame.draw.rect(self.map,self.grey,obstacle)


class RRTgraph:
    def __init__(self,start,goal,mapDimensions,obsdim,obsnum):
        (x,y)=start
        self.start=start
        self.goal=goal
        self.goalFlag=False
        self.mapheight, self.mapwidth = mapDimensions
        self.x=[]
        self.y = []
        self.parents = []
        self.x.append(x)
        self.y.append(y)
        self.parents.append(0)

        self.obstacals=[]
        self.obsDim=obsdim
        self.obsNum=obsnum
        self.goalstate = None
        self.path=[]








#for the simulation it would be random obstacales in our solutions those obstacals are set by the 3D object detection using yolo /lidar
    def makeRandomRect(self):
        uppercornerx=int(random.uniform(0,self.mapwidth-self.obsDim))
        uppercornery=int(random.uniform(0,self.mapheight-self.obsDim))
        return(uppercornerx,uppercornery)



    def makeobs(self):
        obs=[]
        for i in range(0,self.obsNum):
            rectang=None
            startgoalcol=True
            while startgoalcol== True :
                upper= self.makeRandomRect()
                rectang=pygame.Rect(upper,(self.obsDim,self.obsDim))
                if rectang.collidepoint(self.start) or rectang.collidepoint(self.goal):
                    startgoalcol=True
                else:
                    startgoalcol=False
            obs.append(rectang)
        self.obstacals=obs.copy()
        return obs









    def add_node(self,n,x,y):
        self.x.insert(n, x)
        self.y.append( y)



    def remove_node(self,n):
        self.x.pop(n)
        self.y.pop(n)



    def add_edge(self,parent,child):
        self.parents.insert(child,parent)




    def remove_edge(self,n):
        self.parents.pop(n)



    def number_of_nodes(self):
        return len(self.x)



    def distance(self,n1,n2):
        A=np.array((self.x[n1], self.y[n1]))
        B=np.array((self.x[n2], self.y[n2]))
        dist=np.linalg.norm(A-B)
        return dist




    
    def sample_envir(self):
        x=int(random.uniform(0,self.mapwidth))
        y=int(random.uniform(0,self.mapheight))
        
        return (x,y)



    def nearest(self,n):
        dmin=self.distance(0,n)
        nnear=0
        for i in range(0,n):
            if self.distance(i, n)< dmin:
                dmin = self.distance(i,n)
                nnear = i
        return nnear



    def step(self, nnear, nrand, dmax = 35):
        d = self.distance(nnear, nrand)
        if d>dmax :
            u=dmax/d
            (xnear,ynear) = (self.x[nnear], self.y[nnear])
            (xrand, yrand) = (self.x[nrand], self.y[nrand])
            (px,py)=(xrand-xnear, yrand-ynear)
            theta=math.atan2(py, px)
            (x,y) = (int(xnear+dmax*math.cos(theta)),
                     int(ynear+dmax*math.sin(theta)))
            self.remove_node(nrand)
            if abs(x-self.goal[0])<dmax and abs(y-self.goal[1])<dmax:
                self.add_node(nrand, self.goal[0], self.goal[1])
                self.goalstate = nrand
                self.goalFlag = True
            else:
                self.add_node(nrand, x, y)








    def isFree(self):
        n =self.number_of_nodes()-1
        (x,y) = (self.x[n],self.y[n])
        obs = self.obstacals.copy()
        while len(obs)>0:
            rectang=obs.pop(0)
            if rectang.collidepoint(x,y):
                self.remove_node(n)
                return False
        return True







    def crossObstacle(self,x1,x2,y1,y2):
        obs=self.obstacals.copy()
        if x1==x2:
            return
        u = float((y2 - y1) / (x2 - x1))

        b=y1-(u*x1)

        while (len(obs)>0) :
            rectang=obs.pop(0)


            for i in np.linspace(min(x1,x2),max(x1,x2),100):
                if x1 == x2:
                    break
                x=i
                y=u*x+b
                if rectang.collidepoint(x,y):
                    return True
        return False






    def connect(self,n1,n2):
        (x1,y1) = (self.x[n1], self.y[n1])
        (x2,y2) = (self.x[n2], self.y[n2])
        if self.crossObstacle(x1,x2,y1,y2):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1,n2)
            return True









    def bias(self,ngoal):
        n=self.number_of_nodes()
        self.add_node(n,ngoal[0],ngoal[1])
        nnear=self.nearest(n)
        self.step(nnear, n)
        self.connect(nnear, n)
        return self.x, self.y, self.parents



    def expand(self):
        n=self.number_of_nodes()
        x,y = self.sample_envir()
        self.add_node(n,x,y)
        if self.isFree():
            xnearest = self.nearest(n)
            self.step(xnearest,n)
            self.connect(xnearest, n)
        return self.x, self.y, self.parents


    def path_to_goal(self):
        if self.goalFlag:
            self.path=[]
            self.path.append(self.goalstate)
            newpos = self.parents[self.goalstate]
            while(newpos != 0) :
                self.path.append(newpos)
                newpos=self.parents[newpos]
            self.path.append(0)
        return self.goalFlag




    def GetPathCoords(self):
        pathCoords=[]
        for node in self.path :
            x,y =(self.x[node],self.y[node])
            pathCoords.append((x,y))
        return pathCoords






    def cost(self):
        pass








