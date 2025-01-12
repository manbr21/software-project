from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point

ERROR: float = 0.2
DODGE: float = 0.5
MIN_DIST_TO_DODGE: float = 720
M_TO_MM: float = 1000.0
MARGIN: float = 50

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)

    def decision(self):
        if len(self.targets) == 0:
            return
        
        target_is_down = 0
        target_is_right = 0

        if self.targets[0].x > self.robot.x:
            target_is_right = 1
        if self.targets[0].y > self.robot.y:
            target_is_down = 1

        collide = 0
        for i in range(len(self.opponents)):
            if i != 0:
                inside = 0
                if target_is_down == 1:
                    if target_is_right == 1:
                        if self.opponents[i].x <= self.targets[0].x and self.opponents[i].x >= self.robot.x and self.opponents[i].y <= self.targets[0].y and self.opponents[i].y >= self.robot.y:
                            inside+=1
                    else:
                        if self.opponents[i].x >= self.targets[0].x and self.opponents[i].x <= self.robot.x and self.opponents[i].y <= self.targets[0].y and self.opponents[i].y >= self.robot.y:
                            inside+=1
                else:
                    if target_is_right == 1:
                        if self.opponents[i].x <= self.targets[0].x and self.opponents[i].x >= self.robot.x and self.opponents[i].y >= self.targets[0].y and self.opponents[i].y <= self.robot.y:
                            inside+=1
                    else:
                        if self.opponents[i].x >= self.targets[0].x and self.opponents[i].x <= self.robot.x and self.opponents[i].y >= self.targets[0].y and self.opponents[i].y <= self.robot.y:
                            inside+=1
                if inside == 1:
                        
                    #print("inside", self.opponents[i].id)
                    my_pos = Point(self.robot.x * M_TO_MM, self.robot.y * M_TO_MM)
                    #print(my_pos)
                    target = Point(self.targets[0].x * M_TO_MM, self.targets[0].y * M_TO_MM)

                    my_theta = abs((target - my_pos).angle())

                    op_pos = Point(self.opponents[i].x * M_TO_MM, self.opponents[i].y * M_TO_MM)
                    op_theta = abs((target - op_pos).angle())

                    if abs(my_theta - op_theta) < ERROR and my_pos.dist_to(op_pos) < MIN_DIST_TO_DODGE:
                        collide = 1
                    
                        if target_is_down == 1:
                            if target_is_right == 1:
                                where_2_go = Point(self.opponents[i].x + DODGE, self.opponents[i].y - DODGE)
                            else:
                                where_2_go = Point(self.opponents[i].x - DODGE, self.opponents[i].y - DODGE)
                        else:
                            if target_is_right == 1:
                                where_2_go = Point(self.opponents[i].x + DODGE, self.opponents[i].y + DODGE)
                            else:
                                where_2_go = Point(self.opponents[i].x - DODGE, self.opponents[i].y + DODGE)
                        

                        target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, where_2_go)
                        self.set_vel(target_velocity)
                        self.set_angle_vel(target_angle_velocity)

        if not collide:
            target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, self.targets[0])
            self.set_vel(target_velocity)
            self.set_angle_vel(target_angle_velocity)
            
        return

    def post_decision(self):
        pass
