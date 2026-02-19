from function import Map, Graph
import reeds_shepp as rs
import pygame
import time


class RRT_star:

    def __init__(self, start, goal, map_path,iter_after_found_goal, find_rs_path = False):
        self.start = start
        self.goal = goal    
        self.map_path = map_path
        self.map  = Map(start,goal,map_path)
        map_blank = pygame.image.load(map_path)
        self.graph = Graph(start,goal,(self.map.mh,self.map.mw),map_blank)
        self.map.drawmap()
        self.iter_after_found_goal = iter_after_found_goal
        self.find_rs_path = find_rs_path
    
    
    def find_path(self):
        iter = 0
        af = 0
        pygame.display.update()
        start = time.time()
        
        while (not self.graph.goalFlag) or af < self.iter_after_found_goal  :
            x,y,parent = self.graph.RRT() 
            self.graph.goal_check(self.goal,70)    

            # Visual update every 100 iterations
            if iter %100 == 0:
                map = Map(self.start,self.goal,self.map_path)
                self.map.drawmap()
                for i in range (0,len(x)):
                    pygame.draw.circle(map.map,map.red,(x[i],y[i]),map.nodeRad,map.nodeThicc)
                    pygame.draw.line(map.map,map.lime,(x[i],y[i]),(x[parent[i]],y[parent[i]]),map.edgeThicc)
                if self.graph.goalFlag:
                    paths = self.graph.findPath()
                    map.drawPath(paths)
                pygame.display.update()
            iter += 1

        if self.graph.goalFlag:
            paths = self.graph.findPath()
            map.drawPath(paths)
            pygame.image.save(map.map, "solution.jpg")
        RRT_time = self.graph.end -start
        pygame.display.update()
        path = self.graph.path_transform(paths)

        if not self.find_rs_path:
            return RRT_time,path,None, None
        else:
            rs_path = self.graph.find_RS_path(path)
            end = time.time()
            RS_time = end - start
            return RRT_time, path,RS_time, rs_path

if __name__ == '__main__':
    # (40,600),(1070,80)(40,100),(275,475)
    plan = RRT_star(start= (150,40),goal= (300,40) , map_path= 'Map/R.jpg', iter_after_found_goal=1000,find_rs_path=False)
    RRT_time, path ,RS_time, rs_path = plan.find_path()
    print("Find path done !")
    print("Time to find RRT path: " , RRT_time)
    if plan.find_rs_path:
        print("Time to find Reeds-Shepp path: " , RS_time)
        plan.map.draw_rs_path(rs_path,plan.graph.turning_rad,[path[0][0],path[0][1],path[0][2]/57.296],(255,0,0))
    pygame.event.clear()
    pygame.event.wait(0)