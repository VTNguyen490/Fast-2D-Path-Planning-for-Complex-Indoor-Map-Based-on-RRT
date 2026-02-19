import pygame
import random
import math
import time
import reeds_shepp as rs

class Map:
    def __init__(self,start,goal,map):
        self.start = start
        self.goal = goal

        
        #window setting
        self.wname = 'RRt* planning'
        pygame.display.set_caption(self.wname)
        self.externalmap = pygame.image.load(map)
        self.mh = self.externalmap.get_height()
        self.mw = self.externalmap.get_width()
        self.map = pygame.display.set_mode((self.mw,self.mh))
        self.map.fill((255,255,255))
        self.map.blit(self.externalmap,(0,0))
        self.nodeRad = 1
        self.nodeThicc = 1
        self.edge = 1
        self.gray = (70,70,70)
        self.blue=(0,0,255)
        self.green =(0,255,0) 
        self.red = (255,0,0)
        self.brown = (165,42,42)
        self.lime = (178,255,102)
        self.orange = (255,153,51)
        self.white = (255,255,255)
        self.black  =(0,0,0)
        self.edgeThicc = 2
        

    
    def drawmap(self):
        pygame.draw.circle(self.map,self.blue,self.start,self.nodeRad+5,0)
        pygame.draw.circle(self.map,self.green,self.goal,self.nodeRad+5,0)

    def drawPath(self,path):
        for i in range(1,len(path)):
            pygame.draw.circle(self.map,self.red,path[i],self.nodeRad+2,0)
            if i >= 1:
                pygame.draw.line(self.map,self.blue,path[i-1],path[i],self.edgeThicc)
            

    def cross_product(self,x,y):
        return[x[1]*y[2]-x[2]*y[1],x[2]*y[0]-x[0]*y[2],x[0]*y[1]-x[1]*y[0]]
    
    def draw_circle_alpha(self,surface, color, center, radius):
        target_rect = pygame.Rect(center, (0, 0)).inflate((radius * 2, radius * 2))
        shape_surf = pygame.Surface(target_rect.size, pygame.SRCALPHA)
        pygame.draw.circle(shape_surf, color, (radius, radius), radius)
        surface.blit(shape_surf, target_rect)

    def draw_rs_path(self,path,radius,start,colour):
        x = start[0]
        y = start[1]
        phi = start[2] 
        for sub_path in path:
            for section in sub_path:
                if section.steering.value == 1:
                    if section.gear.value == 1:
                        direct = [math.cos(phi),math.sin(phi),0]
                    elif section.gear.value == -1:
                        direct = [-math.cos(phi),-math.sin(phi),0]
                    else:
                        direct = [0,0,0]
                    c = self.cross_product([-math.sin(phi),math.cos(phi),0],direct)
                    x1 = x+math.sin(phi)-1
                    y1 = y-math.cos(phi)-1
                    x1 = x1*radius
                    y1 = y1*radius
                    if c[2]> 0:
                        pygame.draw.arc(self.map,colour,(x1,y1,2*radius,2*radius),-phi-section.param/radius-3.14/2,-phi-3.14/2,2)
                        pygame.display.update()
                        x = x+math.sin(phi)-math.sin(phi+section.param/radius)
                        y = y-math.cos(phi)+math.cos(phi+section.param/radius)
                        phi = phi+section.param/radius
                    elif c[2]<0:
                        pygame.draw.arc(self.map,colour,(x1,y1,2*radius,2*radius),-phi-3.14/2,-phi+section.param/radius-3.14/2,2)
                        pygame.display.update()
                        x = x+math.sin(phi)-math.sin(phi-section.param/radius)
                        y = y-math.cos(phi)+math.cos(phi-section.param/radius)
                        phi = phi-section.param/radius
                    else:
                        phi = phi-section.steering.value*section.param/radius
                elif section.steering.value == -1:
                    if section.gear.value == 1:
                        direct = [math.cos(phi),math.sin(phi),0]
                    elif section.gear.value == -1:
                        direct = [-math.cos(phi),-math.sin(phi),0]
                    else:
                        direct = [0,0,0]
                    c = self.cross_product([math.sin(phi),-math.cos(phi),0],direct)
                    x1 = x-math.sin(phi)-1
                    y1 = y+math.cos(phi)-1
                    x1 = x1*radius
                    y1 = y1*radius
                    if c[2]> 0:
                        pygame.draw.arc(self.map,colour,(x1,y1,2*radius,2*radius),-phi-section.param/radius+3.14/2,-phi+3.14/2,2)
                        pygame.display.update()
                        x = x-math.sin(phi)+math.sin(phi+section.param/radius)
                        y = y+math.cos(phi)-math.cos(phi+section.param/radius)
                        phi = phi+section.param/radius
                    elif c[2]<0:
                        pygame.draw.arc(self.map,colour,(x1,y1,2*radius,2*radius),-phi+3.14/2,-phi+section.param/radius+3.14/2,2)
                        pygame.display.update()
                        x = x-math.sin(phi)+math.sin(phi-section.param/radius)
                        y = y+math.cos(phi)-math.cos(phi-section.param/radius)
                        phi = phi-section.param/radius
                    else:
                        phi = phi-section.steering.value*section.param/radius
                elif section.steering.value == 0:
                    x1 = x + section.gear.value*math.cos(phi)*section.param/radius
                    y1 = y + section.gear.value*math.sin(phi)*section.param/radius
                    x1 = x1*radius
                    y1 = y1*radius
                    pygame.draw.line(self.map,colour,(radius*x,radius*y),(x1,y1),2)
                    pygame.display.update()
                    x = x + section.gear.value*math.cos(phi)*section.param/radius
                    y = y + section.gear.value*math.sin(phi)*section.param/radius

class Graph:
    def __init__(self,start,goal,mapdim,map):
        (x,y) = start
        self.start = start
        self.goal = goal
        self.mh,self.mw = mapdim
        self.x = []
        self.y = []
        self.parent = []
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)
        self.goalFlag = False
        self.dmax = 25
        self.pathmin = 0
        self.map = map
        self.current_target_goal = False
        self.chunking = []
        self.chunknode = []
        self.res = 32
        self.stuck_check = 0
        self.turning_rad = 20
        wchunk = int(self.mw/self.res)
        hchunk = int(self.mh/self.res)
        for i in range(0,hchunk+1):
            temp = []
            temp2 = []
            for k in range(0,wchunk+1):
                temp.append(0)
                temp2.append([])
            self.chunking.append(temp)
            self.chunknode.append(temp2)
        self.esptime = 0
        self.rrtTime = 0


    def addNode(self,n,x,y):
        self.x.insert(n,x)
        self.y.insert(n,y)

    def removeNode(self,n):
        self.x.pop(n)
        self.y.pop(n)
    
    def addEdge(self,parent,child):
        self.parent.insert(child,parent)

    def removeEdge(self,n):
        self.parent.pop(n)
    
    def nodeNum(self):
        return len(self.x)
    
    def rechunk(self):
        self.chunking = []
        self.chunknode = []
        self.res = int(self.res*0.9)
        wchunk = int(self.mw/self.res)
        hchunk = int(self.mh/self.res)
        for i in range(0,hchunk+1):
            temp = []
            temp2 = []
            for k in range(0,wchunk+1):
                temp.append(0)
                temp2.append([])
            self.chunking.append(temp)
            self.chunknode.append(temp2)
        for i in range(0,len(self.x)):
            w = int(self.x[i]/self.res)
            h = int(self.y[i]/self.res)
            self.chunking[h][w] = self.chunking[h][w]+1
            self.chunknode[h][w].append(i)
    
    def distance(self,n1,n2):
        (x1,y1) = (self.x[n1],self.y[n1])
        (x2,y2) = (self.x[n2],self.y[n2])
        p1 = float(pow(x2-x1,2))
        p2 = float(pow(y2-y1,2))
        return pow(p1+p2,0.5)
    
    def sample(self):
        x = int(random.uniform(0,self.mw))
        y = int(random.uniform(0,self.mh))
        return x,y
    
    def crossObs(self,x1,y1,x2,y2):
        p1 = float(pow(x2-x1,2))
        p2 = float(pow(y2-y1,2))
        dis = int(pow(p1+p2,0.5))
        dis = min(dis,100)
        for i in range(0,dis+1):
            if dis == 0:
                u = i
            else:
                u = i/dis
            x = int(x1 + (x2-x1)*u)
            y = int(y1 + (y2-y1)*u)
            color = self.map.get_at((x,y))
            if (pow(color[0],2)+pow(color[1],2)+pow(color[2],2)) <= 16288:
                return True
        return False
    
    def crossObs_long(self,x1,y1,x2,y2):
        p1 = float(pow(x2-x1,2))
        p2 = float(pow(y2-y1,2))
        dis = int(pow(p1+p2,0.5))
        dis = min(dis,200)
        for i in range(0,dis + 1):
            if dis == 0:
                u = i
            else:
                u = i/dis
            x = int(x1 + (x2-x1)*u)
            y = int(y1 + (y2-y1)*u)
            color = self.map.get_at((x,y))
            if (pow(color[0],2)+pow(color[1],2)+pow(color[2],2)) <= 16288:
                return True
        return False
    
    def connect(self,n1,n2):
        (x1,y1) =  (self.x[n1],self.y[n1])
        (x2,y2) =  (self.x[n2],self.y[n2])
        if self.crossObs(x1,y1,x2,y2):
            return False
        else:
            return True
        
    def connect_long(self,n1,n2):
        (x1,y1) =  (self.x[n1],self.y[n1])
        (x2,y2) =  (self.x[n2],self.y[n2])
        if self.crossObs_long(x1,y1,x2,y2):
            return False
        else:
            return True
        
    def nearest(self,n):
        dmin = self.distance(0,n)
        nnear = 0
        for i in range(0,n):
            if dmin > self.distance(i,n):
                dmin = self.distance(i,n)
                nnear = i
        return nnear
    
    def nearest_esp(self,n):
        dmin = self.distance(0,n)
        nnear = 0
        w = int(self.x[n]/self.res)
        h = int(self.y[n]/self.res)
        l = -1
        while l <= 1:
            m = -1
            while m <= 1:
                try:
                    for i in range(0,len(self.chunknode[h+l][w+m])):
                        index = self.chunknode[h+l][w+m][i]
                        if index != n:
                            if dmin >self.distance(index,n):
                                dmin = self.distance(index,n)
                                nnear = index
                except:
                    nnear = nnear
                    dmin = dmin
                m = m+1
            l = l+1
        return nnear
    
    def step(self,nnear,nrand):
        d = self.distance(nnear,nrand)
        if d > self.dmax:
            u = d/(self.dmax)
            (xnnear,ynnear) = (self.x[nnear],self.y[nnear])
            (xnrand,ynrand) = (self.x[nrand],self.y[nrand])
            x = int(xnnear + (xnrand-xnnear)/u)
            y = int(ynnear +(ynrand-ynnear)/u)
        else:
            x = self.x[nrand]
            y = self.y[nrand]
        self.removeNode(nrand)
        if pow(pow(x-self.goal[0],2)+pow((y-self.goal[1]),2),0.5)<= self.dmax/2:
            self.addNode(nrand,self.goal[0],self.goal[1])
            self.current_goal = True
        else:
            self.addNode(nrand,x,y)
            self.current_goal = False
            
    def rewire(self,node):
        n = self.nodeNum()
        nodelist = []
        dislist = []
        f = False
        for i in range (0,n-1):
            if self.distance(i,node) <= 2*self.dmax :
                f = True
                nodelist.append(i)
                index = i
                distance  = 0
                while index != 0 :
                    distance += self.distance(index,self.parent[index])
                    index  = self.parent[index]
                dislist.append(distance)
        
        if f :
            dmin = dislist[0]+self.distance(nodelist[0],node)
            nodemin = nodelist[0]
        else:
            nodemin = self.nearest(node)
            index = nodemin
            dis = 0
            while index != 0 :
                    dis += self.distance(index,self.parent[index])
                    index  = self.parent[index]
            dmin = dis+self.distance(nodemin,node)

        for i in range (0,len(nodelist)):
            if dislist[i]+self.distance(nodelist[i],node) < dmin:
                dmin  = dislist[i]+ self.distance(nodelist[i],node)
                nodemin = nodelist[i]

        if self.connect(nodemin,node):
            self.addEdge(nodemin,node)
            w = int(self.x[node]/self.res)
            h = int(self.y[node]/self.res)
            self.chunking[h][w] = self.chunking[h][w]+1
            self.chunknode[h][w].append(node)
            if self.current_goal:
                if not self.goalFlag:
                    self.goalFlag = True
                    self.end = time.time()
            for i in range (0,len(nodelist)):
                if nodelist[i] != nodemin :
                    if dislist[i] > self.distance(nodelist[i],node)+dmin :
                        if self.connect(node,nodelist[i]):
                            self.removeEdge(nodelist[i])
                            self.addEdge(node,nodelist[i])
        else:
            self.removeNode(node)
            
    def bias(self,ngoal):
        start = time.time()
        n = self.nodeNum()
        self.addNode(n,ngoal[0],ngoal[1])
        nnear = self.nearest(n)
        self.step(nnear,n)
        if self.current_goal:
            if self.connect(nnear,n):
                self.addEdge(nnear,n)
                w = int(self.x[n]/self.res)
                h = int(self.y[n]/self.res)
                self.chunking[h][w] = self.chunking[h][w]+1
                self.chunknode[h][w].append(n)
                if not self.goalFlag:
                    self.goalFlag = True
                    self.end = time.time()
            else:
                self.removeNode(n)
        else:
            self.rewire(n)
        end = time.time()
        self.rrtTime = self.rrtTime + end -start
        return self.x,self.y,self.parent
    
    def goal_check(self,goal, threshold):
        n = self.nodeNum()
        self.addNode(n,goal[0],goal[1])
        if self.distance(n,n-1) < threshold:
            if self.connect_long(n-1,n):
                self.addEdge(n-1,n)
                if not self.goalFlag:
                    self.goalFlag = True
                    self.end = time.time()
            else:
                self.removeNode(n)
        else:
            self.removeNode(n)

    def expand(self):
        start = time.time()
        n = self.nodeNum()
        x,y = self.sample()
        self.addNode(n,x,y)
        nnear = self.nearest(n)
        self.step(nnear,n)
        self.rewire(n)
        end = time.time()
        self.rrtTime = self.rrtTime + end - start
        return self.x,self.y,self.parent

    def RRT(self):
        start = time.time()
        n = self.nodeNum()
        x,y = self.sample()
        self.addNode(n,x,y)
        nnear = self.nearest(n)
        if self.connect(nnear,n):
            self.addEdge(nnear,n)
        else:
            self.removeNode(n)
        end = time.time()
        self.rrtTime = self.rrtTime + end - start
        return self.x,self.y,self.parent
    
    def expand2(self):
        esp_start = time.time()
        self.expand2_connect = False
        chunkls = []
        conls = []
        for h in range(1,len(self.chunking)-1):
            for w in range(1,len(self.chunking[h])-1):
                check = False
                tmp = []
                if self.chunking[h][w] == 0:
                    if self.chunking[h-1][w] != 0:
                        check = True
                        tmp.append(1)
                    if self.chunking[h+1][w] != 0:
                        check = True
                        tmp.append(2)
                    if self.chunking[h][w-1] != 0:
                        check = True
                        tmp.append(3)
                    if self.chunking[h][w+1] != 0:
                        check = True
                        tmp.append(4)
                    if self.chunking[h-1][w-1] != 0:
                        check = True
                        tmp.append(5)
                    if self.chunking[h-1][w+1] != 0:
                        check = True
                        tmp.append(6)
                    if self.chunking[h+1][w-1] != 0:
                        check = True
                        tmp.append(7)
                    if self.chunking[h+1][w+1] != 0:
                        check = True
                        tmp.append(8)
                    if check:
                        chunkls.append([h,w])
                        conls.append(tmp)
        t1 = 0
        while not self.expand2_connect and t1 <= 10 and chunkls:
            index = int(random.uniform(0,len(chunkls)))
            t3 = 0
            n = self.nodeNum()
            x = chunkls[index][1]*self.res + int(random.uniform(2,self.res-2))
            y = chunkls[index][0]*self.res + int(random.uniform(2,self.res-2))
            self.addNode(n,x,y)
            nnear = self.nearest_esp(n)
            while not self.connect(nnear,n) and t3 <= 4:
                self.removeNode(n)
                x = chunkls[index][1]*self.res + int(random.uniform(2,self.res-2))
                y = chunkls[index][0]*self.res + int(random.uniform(2,self.res-2))
                self.addNode(n,x,y)
                nnear = self.nearest_esp(n)
                t3 = t3+1
            if self.connect(nnear,n):
                self.addEdge(nnear,n)
                w = int(self.x[n]/self.res)
                h = int(self.y[n]/self.res)
                self.chunking[h][w] = self.chunking[h][w]+1
                self.chunknode[h][w].append(n)
                self.expand2_connect = True
                
            else:
                self.removeNode(n)
                t2 = 0
                while not self.expand2_connect and t2 <= 2 :#2
                    i = int(random.uniform(0,len(conls[index])))
                    if conls[index][i] == 1:
                        x2 = chunkls[index][1]*self.res + int(random.uniform(2,self.res-2))
                        y2 = (chunkls[index][0]-1)*self.res + int(random.uniform(2,self.res-2))
                    elif conls[index][i] == 2:
                        x2 = chunkls[index][1]*self.res + int(random.uniform(2,self.res-2))
                        y2 = (chunkls[index][0]+1)*self.res + int(random.uniform(2,self.res-2))
                    elif conls[index][i] == 3:
                        x2 = (chunkls[index][1]-1)*self.res + int(random.uniform(2,self.res-2))
                        y2 = chunkls[index][0]*self.res + int(random.uniform(2,self.res-2))
                    elif conls[index][i] == 4:
                        x2 = (chunkls[index][1]+1)*self.res + int(random.uniform(2,self.res-2))
                        y2 = chunkls[index][0]*self.res + int(random.uniform(2,self.res-2))
                    elif conls[index][i] == 5:
                        x2 = (chunkls[index][1]-1)*self.res + int(random.uniform(2,self.res-2))
                        y2 = (chunkls[index][0]-1)*self.res + int(random.uniform(2,self.res-2))
                    elif conls[index][i] == 6:
                        x2 = (chunkls[index][1]+1)*self.res + int(random.uniform(2,self.res-2))
                        y2 = (chunkls[index][0]-1)*self.res + int(random.uniform(2,self.res-2))
                    elif conls[index][i] == 7:
                        x2 = (chunkls[index][1]-1)*self.res + int(random.uniform(2,self.res-2))
                        y2 = (chunkls[index][0]+1)*self.res + int(random.uniform(2,self.res-2))
                    elif conls[index][i] == 8:
                        x2 = (chunkls[index][1]+1)*self.res + int(random.uniform(2,self.res-2))
                        y2 = (chunkls[index][0]+1)*self.res + int(random.uniform(2,self.res-2))
                    if x2>= self.mw:
                        x2 = self.mw-1
                    elif x2 <= 0:
                        x2 = 1
                    if y2>= self.mh:
                        y2 = self.mh-1
                    elif y2 <= 0:
                        y2 = 1
                    self.addNode(n,x2,y2)
                    nodemin = self.nearest_esp(n)

                    if self.connect(nodemin,n):
                        self.addEdge(nodemin,n)
                        self.addNode(n+1,x,y)
                        if self.connect(n,n+1):
                            self.addEdge(n,n+1)
                            w = int(self.x[n]/self.res)
                            h = int(self.y[n]/self.res)
                            self.chunking[h][w] = self.chunking[h][w]+1
                            self.chunknode[h][w].append(n)
                            w = int(self.x[n+1]/self.res)
                            h = int(self.y[n+1]/self.res)
                            self.chunking[h][w] = self.chunking[h][w]+1
                            self.chunknode[h][w].append(n+1)
                            self.expand2_connect = True
                        else:
                            self.removeEdge(n)
                            self.removeNode(n+1)
                            self.removeNode(n)
                            self.expand2_connect = False  
                    else:
                        self.removeNode(n)
                        self.expand2_connect = False
                    t2 = t2+1
            t1 = t1+1
        if not self.expand2_connect or (not chunkls and not self.goalFlag ):
            self.stuck_check = self.stuck_check+1
        else:
            self.stuck_check = 0
        if self.stuck_check >= 25 and chunkls:
            self.rechunk()
            self.stuck_check = 0
        esp_end  = time.time()
        self.esptime = self.esptime + esp_end-esp_start
        return self.x,self.y,self.parent
    
    def expand3(self):
        chunkls = []
        self.expand3_connect = False
        for h in range(0,len(self.chunking)):
            for w in range(0,len(self.chunking[h])):
                if 1 <= self.chunking[h][w]:
                    chunkls.append([h,w])
        
        n = self.nodeNum()
        if chunkls:
            index = int(random.uniform(0,len(chunkls)))
            x = chunkls[index][1]*self.res + int(random.uniform(2,self.res-2))
            y = chunkls[index][0]*self.res + int(random.uniform(2,self.res-2))
        else:
            x,y = self.sample()
        self.addNode(n,x,y)
        
        nodelist = []
        dislist = []
        f = False
        for i in range (0,n-1):
            if self.distance(i,n) <= self.res*2 :
                f = True
                nodelist.append(i)
                index = i
                distance  = 0
                while index != 0 :
                    distance += self.distance(index,self.parent[index])
                    index  = self.parent[index]
                dislist.append(distance)
        
        if f :
            dmin = dislist[0]+self.distance(nodelist[0],n)
            nodemin = nodelist[0]
        else:
            nodemin = self.nearest(n)
            index = nodemin
            dis = 0
            while index != 0 :
                    dis += self.distance(index,self.parent[index])
                    index  = self.parent[index]
            dmin = dis+self.distance(nodemin,n)
        
        for i in range(0,len(nodelist)):
            if self.connect(nodelist[i],n):
                if dislist[i]+self.distance(nodelist[i],n) < dmin:
                    dmin  = dislist[i]+ self.distance(nodelist[i],n)
                    nodemin = nodelist[i]
        
        if self.connect(nodemin,n):
            self.addEdge(nodemin,n)
            w = int(self.x[n]/self.res)
            h = int(self.y[n]/self.res)
            self.chunking[h][w] = self.chunking[h][w]+1
            for i in range (0,len(nodelist)):
                if nodelist[i] != nodemin :
                    if dislist[i] > self.distance(nodelist[i],n)+dmin :
                        if self.connect(n,nodelist[i]):
                            self.removeEdge(nodelist[i])
                            self.addEdge(n,nodelist[i])
        else:
            self.removeNode(n)
        return self.x,self.y,self.parent
    
    def findPath(self):
        if self.goalFlag:
            pathlist = []
            dislis = []
            n = len(self.x)
            for i in range(0,n):
                path = []
                dis = 0
                if self.x[i] == self.goal[0] and self.y[i] == self.goal[1]:
                    index = i
                    while index != 0:
                        path.append([self.x[index],self.y[index]])
                        dis += self.distance(index,self.parent[index])
                        index = self.parent[index]
                    dislis.append(dis)
                    path.append([self.start[0],self.start[1]])
                    pathlist.append(path)
            dmin = dislis[0]
            pathmin = 0
            for i in range (0,len(dislis)):
                if dislis[i] < dmin:
                    dmin  = dislis[i]
                    pathmin = i
            if self.pathmin != dmin:
                self.pathmin = dmin
                #print(str(self.pathmin))
            return pathlist[pathmin]

    def path_transform(self,Paths):
        paths = []
        for i in range(0,len(Paths)):
            try:
                paths.index(Paths[i])
            except:
                paths.append(Paths[i])
        
        path = []
        i = len(paths)-1
        while i>= 0:
            path.append(paths[i])
            i = i-1

        for i in range(len(path) - 1):
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            theta = math.atan2(dy, dx)
            theta = theta*57.296
            path[i].append(theta)
        path[len(path)-1].append(90)
        return path
                            
    
    
    def cross_product(self,x,y):
        return[x[1]*y[2]-x[2]*y[1],x[2]*y[0]-x[0]*y[2],x[0]*y[1]-x[1]*y[0]]

    def reep_shepp_check(self,path,x,y,phi):
        x1 = x
        y1 = y
        phi1 = phi
        for section in path:
            if section.steering.value == 1:
                t = 0
                if section.gear.value == 1:
                    direct = [math.cos(phi1),math.sin(phi1),0]
                else:
                    direct = [-math.cos(phi1),-math.sin(phi1),0]
                c = self.cross_product([-math.sin(phi1),math.cos(phi1),0],direct)
                if c[2] < 0 :
                    while t<=200:
                        x1 = x1+math.sin(phi1)-math.sin(phi1-section.param/200)
                        y1 = y1-math.cos(phi1)+math.cos(phi1-section.param/200)
                        phi1 = phi1-section.param/200
                        if ((self.turning_rad*x1>self.mw) or (self.turning_rad*x1<1) or (self.turning_rad*y1>self.mh) or (self.turning_rad*y1<1)):
                            return False
                        color = self.map.get_at((int(self.turning_rad*x1),int(self.turning_rad*y1)))
                        if (pow(color[0],2)+pow(color[1],2)+pow(color[2],2)) <= 16288 :
                            return False
                        t = t+1
                elif c[2] >= 0:
                    while t<=200:
                        x1 = x1+math.sin(phi1)-math.sin(phi1+section.param/200)
                        y1 = y1-math.cos(phi1)+math.cos(phi1+section.param/200)
                        phi1 = phi1+section.param/200
                        if ((self.turning_rad*x1>self.mw) or (self.turning_rad*x1<1) or (self.turning_rad*y1>self.mh) or (self.turning_rad*y1<1)):
                            return False
                        color = self.map.get_at((int(self.turning_rad*x1),int(self.turning_rad*y1)))
                        if (pow(color[0],2)+pow(color[1],2)+pow(color[2],2)) <= 16288:
                            return False
                        t = t+1
            elif section.steering.value == 0:
                t = 0
                while t <=200:
                    x1 = x1 + section.gear.value*math.cos(phi1)*section.param/200
                    y1 = y1 + section.gear.value*math.sin(phi1)*section.param/200
                    if ((self.turning_rad*x1>self.mw) or (self.turning_rad*x1<1) or (self.turning_rad*y1>self.mh) or (self.turning_rad*y1<1)):
                        return False
                    color = self.map.get_at((int(self.turning_rad*x1),int(self.turning_rad*y1)))
                    if ((pow(color[0],2)+pow(color[1],2)+pow(color[2],2)) <= 16288):
                        return False
                    t = t+1
            elif section.steering.value == -1:
                t = 0
                if section.gear.value == 1:
                    direct = [math.cos(phi1),math.sin(phi1),0]
                else:
                    direct = [-math.cos(phi1),-math.sin(phi1),0]
                c = self.cross_product([math.sin(phi1),-math.cos(phi1),0],direct)
                if c[2] < 0 :
                    while t<=200:
                        x1 = x1-math.sin(phi1)+math.sin(phi1-section.param/200)
                        y1 = y1+math.cos(phi1)-math.cos(phi1-section.param/200)
                        phi1 = phi1-section.param/200
                        if ((self.turning_rad*x1>self.mw) or (self.turning_rad*x1<1) or (self.turning_rad*y1>self.mh) or (self.turning_rad*y1<1)):
                            return False
                        color = self.map.get_at((int(self.turning_rad*x1),int(self.turning_rad*y1)))
                        if (pow(color[0],2)+pow(color[1],2)+pow(color[2],2)) <= 16288:
                            return False
                        t = t+1
                elif c[2] >= 0:
                    while t<=200:
                        x1 = x1-math.sin(phi1)+math.sin(phi1+section.param/200)
                        y1 = y1+math.cos(phi1)-math.cos(phi1+section.param/200)
                        phi1 = phi1+section.param/200
                        if ((self.turning_rad*x1>self.mw) or (self.turning_rad*x1<1) or (self.turning_rad*y1>self.mh) or (self.turning_rad*y1<1)):
                            return False
                        color = self.map.get_at((int(self.turning_rad*x1),int(self.turning_rad*y1)))
                        if (pow(color[0],2)+pow(color[1],2)+pow(color[2],2)) <= 16288:
                            return False
                        t = t+1
        return True      

    def find_RS_path(self, path):
        rs_path = []
        rs_path2 = []
        for i in range(0,len(path)):
            path[i][0] = path[i][0]/self.turning_rad
            path[i][1] = path[i][1]/self.turning_rad
        i = 0
        while i < len(path)-1:
            possible_path = []
            sub_path = []
            for l in range(0,len(path)-i-1):
                paths = rs.get_all_paths(path[i],path[len(path)-1-l])
                for p in paths:
                    if self.reep_shepp_check(p,path[i][0],path[i][1],path[i][2]/57.296):
                        possible_path.append(p)
                if possible_path:
                    sub_path = min(possible_path, key=rs.path_length)
                    break
                elif len(path)-1-l == i+1:
                    paths = rs.get_all_paths(path[i],path[len(path)-1-l])
                    for p in paths:
                        if self.reep_shepp_check(p,path[i][0],path[i][1],path[i][2]/57.296):
                            possible_path.append(p)
                    if possible_path:
                        sub_path = min(possible_path, key=rs.path_length)
                    else:
                        current_node = path[i]
                        while not possible_path:
                            iter = 0
                            frame_x = path[i+1][0]*self.turning_rad
                            frame_y = path[i+1][1]*self.turning_rad
                            dis = pow(pow(current_node[0]-path[i+1][0],2)+pow(current_node[1]-path[i+1][1],2),0.5)*self.turning_rad
                            while iter < 20:
                                t = 0
                                while t< 10: 
                                    sub_pospath = []
                                    x = int(random.uniform(0,dis*5))/100
                                    y = int(random.uniform(0,30))-15
                                    length = pow(pow((current_node[0]-path[i+1][0]),2)+pow((current_node[1]-path[i+1][1]),2),0.5)
                                    next_x = frame_x+ x*(current_node[0]-path[i+1][0])/length + y*(current_node[1]-path[i+1][1])/length
                                    next_y = frame_y + x*(current_node[1]-path[i+1][1])/length + y*(current_node[0]-path[i+1][0])/length
                                    temp_paths = rs.get_all_paths(current_node,[next_x/self.turning_rad,next_y/self.turning_rad,path[i+1][2]])
                                    for p in temp_paths:
                                        if self.reep_shepp_check(p,current_node[0],current_node[1],current_node[2]/57.296):
                                            sub_pospath.append(p)
                                    if sub_pospath:
                                        sub_sub_path = min(sub_pospath, key=rs.path_length)
                                        for section in sub_sub_path:
                                            sub_path.append(section)
                                        current_node = [next_x/self.turning_rad,next_y/self.turning_rad,path[i+1][2]]
                                        break
                                    else:
                                        t = t+1
                                if sub_pospath:
                                    # print("---------------------")
                                    # print("frame:"+str(int(frame_x))+","+str(int(frame_y)))
                                    # print(int(next_x),int(next_y))
                                    break
                                else:
                                    iter = iter +1
                                    frame_x = frame_x +dis/20*(current_node[0]-path[i+1][0])/length
                                    frame_y = frame_y +dis/20*(current_node[1]-path[i+1][1])/length
                            paths = rs.get_all_paths(current_node,path[i+1])
                            for p in paths:
                                if self.reep_shepp_check(p,current_node[0],current_node[1],current_node[2]/57.296):
                                    possible_path.append(p)
                            if possible_path:
                                sub_sub_path = min(possible_path, key=rs.path_length)
                                for section in sub_sub_path:
                                    sub_path.append(section)
                                break
                        
            rs_path.append(sub_path)
            new_path = []
            for n in range(0,i+1):
                new_path.append(path[n])
            for k in range(len(path)-1-l,len(path)):
                new_path.append(path[k])
            path = new_path
            i = i+1

        # pathlen = 0
        for sub_path in rs_path:
            for section in sub_path:
                section.param = section.param*self.turning_rad
                # pathlen = pathlen + section.param  
        return rs_path
    


