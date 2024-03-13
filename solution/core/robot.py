import sys
from enum import Enum
from typing import List
from queue import LifoQueue

from log import logger, My_Timer
from path_planing import Point
from path_planing import Robot_Actions

from .berth import Berth

robot_action_value_to_cmd = {
    Robot_Actions.RIGHT : 0,
    Robot_Actions.LEFT  : 1,
    Robot_Actions.UP    : 2,
    Robot_Actions.DOWN  : 3,
    Robot_Actions.HOLD  : 5,
}

class Robot_Extended_Status(Enum):
    StayStill = 0

    OnBerth = 1

    BackBerthAndPull = 2

    GotoFetchFromBerth = 3

    GotGoods = 4

class Robot():
    def __init__(self, startX=0, startY=0, goods=0, status=0, mbx=0, mby=0):
        self.pos = Point(x = startX, y = startY)
        #self.x = startX
        #self.y = startY
        self.goods = goods
        self.status = status
        self.mbx = mbx
        self.mby = mby

        self.robot_id = -1
        self.berth_id = -1
        self.extended_status = Robot_Extended_Status.StayStill
        self.paths_stk = LifoQueue()
        self.num_paths_predict = 5

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

    def collision_check(self, 
                move_matrix: List[List[Point]],
                robots = [],
                berths: List[Berth] = [], 
                target_pos: Point = Point(-1, -1)):
        pass
    
    def DFS_path_planing(self, 
                move_matrix: List[List[Point]],
                robots = [],
                berths: List[Berth] = [], 
                target_pos: Point = Point(-1, -1)):
            
            timer = My_Timer()
            if self.extended_status == Robot_Extended_Status.GotoFetchFromBerth:
                # 如何处理target pos不可达到
                self.paths_stk = LifoQueue()

                cur_pos = target_pos
                self.paths_stk.put(cur_pos)
                cur_pos = cur_pos + move_matrix[cur_pos.y][cur_pos.x]
                # robot当前位置要么是港口，要么是沿着之前BFS的路径走了一段距离
                while (cur_pos != berths[self.berth_id].pos and cur_pos != self.pos):
                    self.paths_stk.put(cur_pos)
                    # logger.info("debug %s | %s", cur_pos, berths[berth_id].pos)
                    cur_pos = cur_pos + move_matrix[cur_pos.y][cur_pos.x]

            elif self.extended_status == Robot_Extended_Status.BackBerthAndPull:
                self.paths_stk = LifoQueue()
                tmp_stk = LifoQueue()
                cur_pos = self.pos
                cur_pos = cur_pos + move_matrix[cur_pos.y][cur_pos.x]
                while (cur_pos != berths[self.berth_id].pos): # 此时self.pos就是在berth上
                    tmp_stk.put(cur_pos)
                    #logger.info("debug %s | %s", cur_pos, berths[berth_id].pos)
                    cur_pos = cur_pos + move_matrix[cur_pos.y][cur_pos.x]
                tmp_stk.put(cur_pos)
                
                while not tmp_stk.empty():
                    self.paths_stk.put(tmp_stk.get())

            elif self.extended_status == Robot_Extended_Status.OnBerth:
                self.paths_stk = LifoQueue()
                for _ in range(self.num_paths_predict):
                    self.paths_stk.put(self.pos)
            #logger.info("DFS cost: %s", timer.click()*1000)
                    
    def paths_execution(self, 
                move_matrix: List[List[Point]],
                robots = [],
                berths: List[Berth] = [], 
                target_pos: Point = Point(-1, -1)):
        
        def print_move_cmd():
            if (action != Robot_Actions.HOLD):
                print("move", self.robot_id, robot_action_value_to_cmd[action])

        if not self.paths_stk.empty():
            next_pos = self.paths_stk.get()
            action = next_pos - self.pos
            if (action not in robot_action_value_to_cmd):
                logger.info("下一步action出错，不在预定的actions中，实际为%s", action)
                self.extended_status = Robot_Extended_Status.BackBerthAndPull
                return
        else:
            next_pos = self.pos
            action = Robot_Actions.HOLD

        if self.extended_status == Robot_Extended_Status.GotoFetchFromBerth:
            print_move_cmd()
            if (next_pos == target_pos):
                print("get", self.robot_id)
                self.extended_status = Robot_Extended_Status.GotGoods
        
        elif self.extended_status == Robot_Extended_Status.BackBerthAndPull:
            print_move_cmd()
            if (next_pos == berths[self.berth_id].pos):
                self.extended_status = Robot_Extended_Status.OnBerth
                if self.goods == 1:
                    print("pull", self.robot_id)
                    berths[self.berth_id].num_gds += 1
        
        elif self.extended_status == Robot_Extended_Status.OnBerth:
            pass
        
    def run(self, 
                move_matrix: List[List[Point]],
                robots = [],
                berths: List[Berth] = [], 
                target_pos: Point = Point(-1, -1)):
        # robot根据状态执行每一帧的动作
        self.DFS_path_planing(move_matrix, robots, berths, target_pos)
        self.collision_check(move_matrix, robots, berths, target_pos)
        self.paths_execution(move_matrix, robots, berths, target_pos)