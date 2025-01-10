from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point
from utils.Geometry import Geometry
import math

M_TO_MM: float = 1000.0
FACTOR: float = 20

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)

    def decision(self):
        if len(self.targets) == 0:
            return

        #theta -> angle beetween x and target

        target = Point(self.targets[0].x * M_TO_MM, self.targets[0].y * M_TO_MM)

        my_pos = Point(self.robot.x * M_TO_MM, self.robot.y * M_TO_MM)
        my_theta = (target - my_pos).angle()

        collide = 0
        
        for i in range(len(self.opponents)):
            if i != 0:
                op_pos = Point(self.opponents[i].x * M_TO_MM, self.opponents[i].y * M_TO_MM)
                op_theta = (target - op_pos).angle()
                #print(op_theta, " id: " ,self.opponents[i].id)

                my_dist_to_op = my_pos.dist_to(op_pos)
                #my_dist_to_target = my_pos.dist_to(target)

                my_theta = (target - my_pos).angle()

                right = 0
                up = 0

                if self.targets[0].x - self.robot.x > 0:
                    right = 1
                if self.targets[0].y - self.robot.y > 0:
                    up = 1 

                #print(my_theta)
                if(abs(my_theta - op_theta) < 0.08 and my_dist_to_op < 800):
                    if (right == 1 and self.opponents[i].x > self.robot.x) or (up == 1 and self.opponents[i].y > self.robot.y) or (right == 0 and self.opponents[i].x <= self.robot.x) or (up == 0 and self.opponents[i].y <= self.robot.y):
                        collide = 1
                        #print("same angle ", self.opponents[i].id)
                        if self.targets[0].y - self.robot.y < 0: #
                            if self.targets[0].x - self.robot.x < 0:
                                where_2_go = Point(self.opponents[i].x + FACTOR, self.opponents[i].y - FACTOR)
                            else:
                                where_2_go = Point(self.opponents[i].x + FACTOR, self.opponents[i].y - FACTOR)
                        else:
                            if self.targets[0].x - self.robot.x < 0:
                                where_2_go = Point(self.opponents[i].x - FACTOR, self.opponents[i].y + FACTOR)
                            else:
                                where_2_go = Point(self.opponents[i].x - FACTOR, self.opponents[i].y + FACTOR)
                        target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, where_2_go)
                        self.set_vel(target_velocity)
                        self.set_angle_vel(target_angle_velocity)
                    
        if(not collide):
            target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, self.targets[0])
            self.set_vel(target_velocity)
            self.set_angle_vel(target_angle_velocity)                      
            
        return

    def post_decision(self):
        pass
