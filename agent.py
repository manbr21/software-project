from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point
from RRT import RRTStar
from RRT import Node

DIST_ERROR: float = 0.05

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)

    def decision(self):
        if len(self.targets) == 0:
            return
        
        obstacles = []

        where_to_go = Point(0,0)

        for i in range(len(self.opponents) + 1):
            if i != 0:
                obstacles.append((self.opponents[i].x, self.opponents[i].y, 0.3)) #creating a list of obstacles w/ size

        if self.index == -1:
            self.rrt = RRTStar([self.robot.x, self.robot.y], [self.targets[0].x, self.targets[0].y], obstacles, [3, 2], 0.25, 1500)
            self.rrt.plan()
            self.index = 0 #possibly a path
            if(self.rrt.path == None): #try again
                self.index = -1
            
        final_target = False

        if self.index != -1:
            where_to_go = Point(self.rrt.path[self.index][0], self.rrt.path[self.index][1])
            
            for i in range(len(self.rrt.path)):
                if i != 0:
                    if where_to_go.dist_to(Point(self.opponents[i].x, self.opponents[i].y)) < 0.3:
                        print("COLIDIU")
            if self.index == len(self.rrt.path) - 1: #last node is the target
                final_target = True

            target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, where_to_go, final_target)
            self.set_vel(target_velocity)
            self.set_angle_vel(target_angle_velocity)

            if final_target == False:
                if self.pos.dist_to(where_to_go) < DIST_ERROR:
                    self.index += 1
            else:
                if self.pos.dist_to(where_to_go) < 0.18: #doesnt need to be closer than that
                    self.index += 1

        if self.index != -1 and not(self.index in range(len(self.rrt.path))): #finished the path
            self.index = -1

        return

    def post_decision(self):
        pass
