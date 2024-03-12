import sys
from enum import Enum
from typing import List
from queue import LifoQueue
from solution.core.base import Point
from solution.log.logger import logger

point_direction_to_robot_cmd = {
    Point(0, -1) : 2,
    Point(0, 1) : 3,
    Point(-1, 0) : 1,
    Point(1, 0) : 0,
    Point(0, 0): 4
}
    

class Robot_Extended_Status(Enum):
    StayStill = 0
    SetBerth = 1
    BackBerth = 2
    GotoFromBerth = 3
    FetchGoods = 4
    OffloadGoods = 5
    GotoFetchFromBerth = 6
    OnBerth = 7
    GotGoods = 8
    BackBerthAndPull = 9

class Context_Go_To_Fetch_From_Berth:
    
    def __init__(self) -> None:

        self.init_down = False
        self.paths: LifoQueue[Point] = LifoQueue()
        self.fail = False
    
    def init(self):
        self.paths = LifoQueue()
        self.got_goods = False

    def quit(self):
        self.init_down = False

class Robot():
    def __init__(self, startX=0, startY=0, goods=0, status=0, mbx=0, mby=0):
        self.pos = Point(x = startX, y = startY)
        #self.x = startX
        #self.y = startY
        self.goods = goods
        self.status = status
        self.mbx = mbx
        self.mby = mby
        self.extended_status = Robot_Extended_Status.StayStill

        # runtime
        self.context_go_to_fetch_from_berth = Context_Go_To_Fetch_From_Berth()

    @property
    def x(self):
        return self.pos.x
    @x.setter
    def x(self, value):
        self.pos.x = value

    @property
    def y(self):
        return self.pos.y
    @y.setter
    def y(self, value):
        self.pos.y = value

    def run(self, robot_id: int, move_matrix: List[List[Point]],
            berth_id: int = 0, 
            robots = None, 
            berths = None,  
            target_pos: Point = Point(-1, -1)):
        # robot根据状态执行每一帧的动作
        if self.extended_status == Robot_Extended_Status.BackBerth:
            self.back_berth(robot_id, move_matrix, berth_id=berth_id, berths=berths)

        elif self.extended_status == Robot_Extended_Status.GotoFetchFromBerth:
            self.go_to_fetch_from_berth(robot_id, move_matrix, target_pos=target_pos)

        elif self.extended_status == Robot_Extended_Status.BackBerthAndPull:
            self.back_berth_and_pull(robot_id, move_matrix, berth_id=berth_id, berths=berths)
    
    def back_berth(self, robot_id: int, move_matrix: List[List[Point]],
                            berth_id: int = 0, berths = []):
        action = move_matrix[self.y][self.x]
        # 泵碰撞？/
        if (action not in point_direction_to_robot_cmd):
            self.extended_status = Robot_Extended_Status.BackBerth
            return
        if self.pos != berths[berth_id].pos:
            print("move", robot_id, point_direction_to_robot_cmd[action])
        else:
            self.extended_status = Robot_Extended_Status.OnBerth

    def back_berth_and_pull(self, robot_id: int, move_matrix: List[List[Point]],
                            berth_id: int = 0, berths = []):
        action = move_matrix[self.y][self.x]
        
        # 碰撞？？？
        if (action not in point_direction_to_robot_cmd):
            self.extended_status = Robot_Extended_Status.BackBerth
            return
        
        if self.pos != berths[berth_id].pos:
            print("move", robot_id, point_direction_to_robot_cmd[action])
            if (self.pos + action == berths[berth_id].pos):
                print("pull", robot_id)
        else:
            berths[berth_id].num_gds += 1
            logger.info("_______________________%s", berths[berth_id].num_gds)
            self.extended_status = Robot_Extended_Status.OnBerth

    def go_to_fetch_from_berth(self, robot_id: int, 
                               move_matrix: List[List[Point]], 
                               target_pos: Point = Point(-1, -1)):
        if (self.context_go_to_fetch_from_berth.init_down == False):
            
            self.context_go_to_fetch_from_berth.init()

            if (target_pos == Point(-1, -1)):
                logger.info("go_to_fetch_from_berth got None target pos")

            # 如何处理 target不可达？？？
            
            cur_pos = target_pos
            self.context_go_to_fetch_from_berth.paths.put(cur_pos)
            cur_pos = cur_pos + move_matrix[cur_pos.y][cur_pos.x]
            while (cur_pos != self.pos): # 此时self.pos就是在berth上
                self.context_go_to_fetch_from_berth.paths.put(cur_pos)
                cur_pos = cur_pos + move_matrix[cur_pos.y][cur_pos.x]
            self.context_go_to_fetch_from_berth.init_down = True
        else:
            if (self.pos != target_pos
                   and self.context_go_to_fetch_from_berth.paths.empty() is False):
                next_pos = self.context_go_to_fetch_from_berth.paths.get()
                #logger.info(next_pos)
                action = next_pos - self.pos
                # 碰撞
                if (action not in point_direction_to_robot_cmd):
                    self.extended_status = Robot_Extended_Status.BackBerth
                    return
                print("move", robot_id, point_direction_to_robot_cmd[action])
                if (next_pos == target_pos):
                    print("get", robot_id)
            elif (self.pos == target_pos):
                self.extended_status = Robot_Extended_Status.GotGoods
                self.context_go_to_fetch_from_berth.quit()
    
            
