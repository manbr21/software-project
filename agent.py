from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point
from RRT import RRTStar

DIST_ERROR: float = 0.1
FUTURE_TIME: float = 0.3
OBSTACLES_SIZE: float = 0.3 # w/ a margin (0.18 + 0.12)

class ExampleAgent(BaseAgent):

    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)
        self.current_index = -1  # wich node the robot tries to go
        self.rrt = RRTStar #to make things easier
        self.collide = 0 #indicate possible collision
        self.my_target = None #closest target

    def future_point(self, x, y, vx, vy, dt): #calculating the nex point by admiting constant speed (estimating)
        final_x = x + vx * dt
        final_y = y + vy * dt
        return final_x, final_y
    
    def put_obstacles(self): #opponents = obstacles | other targets = obstacles
        obstacles = []
        if self.current_index == -1 or self.collide == 1: #if we are looking for a new path
            for i in self.opponents:
                future_x, future_y = self.future_point(self.opponents[i].x, self.opponents[i].y, self.opponents[i].v_x, self.opponents[i].v_y, FUTURE_TIME)
                obstacles.append((future_x, future_y, OBSTACLES_SIZE)) #creating a tuple of obstacles w/ size
            for i in range(len(self.targets)): 
                if self.targets[i] != self.my_target:
                    obstacles.append((self.targets[i].x, self.targets[i].y, OBSTACLES_SIZE))
        
        return obstacles
    
    def calculate_path(self, obstacles):
        if self.current_index == -1 or self.collide == 1: #if we are looking for a new path
            self.rrt = RRTStar([self.robot.x, self.robot.y], [self.my_target.x, self.my_target.y], obstacles, [3, 2], 0.25, 10000)
            self.rrt.plan()
            self.current_index = 0 if self.rrt.path else -1 
                
            if(self.rrt.path == -1): #try again
                print("couldn't find a path, retrying")
            else:
                if self.collide == 1:
                    self.collide = 2 #path found, isnt looking for collision
    
    def is_last_node(self):
        if self.current_index == len(self.rrt.path) - 1: #last node is the target
            return True
        return False
    
    def check_collisions(self, where_to_go):
        if self.collide == 0: #if we are looking for collisions
                will_collide = False #to stop the loop
                for i in self.opponents:
                    future_x, future_y = self.future_point(self.opponents[i].x, self.opponents[i].y, self.opponents[i].v_x, self.opponents[i].v_y, FUTURE_TIME)
                    if Point(future_x, future_y).dist_to(where_to_go) < OBSTACLES_SIZE and will_collide == False: #potential collision and isnt the last node
                        #recalculate path
                        if not self.is_last_node(): #to avoid bugs related to targets and opponents colliding
                            will_collide = True
                            self.collide = 1   
    
    def go_to_node(self, where_to_go): #use the default moving function
        target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, where_to_go)
        self.set_vel(target_velocity)
        self.set_angle_vel(target_angle_velocity)

    def reached_node(self, where_to_go):
        if self.pos.dist_to(where_to_go) < DIST_ERROR:
            self.collide = 0 #look for collision
            self.current_index += 1

    def decision(self):
        if len(self.targets) == 0:
            return
        
        if self.my_target is not None: #target designated

            obstacles = self.put_obstacles()

            self.calculate_path(obstacles) #use RRT to calculate path

            if self.current_index != -1: #if could find a path
                where_to_go = Point(self.rrt.path[self.current_index][0], self.rrt.path[self.current_index][1])
                self.check_collisions(where_to_go)
                
                if self.collide != 1: #if no collision, go to the next node
                    self.go_to_node(where_to_go)
                    self.reached_node(where_to_go) #if node is reached, next

            if self.current_index != -1 and self.current_index >= len(self.rrt.path): #finished the path
                self.current_index = -1
        
        return

    def post_decision(self):
        pass
