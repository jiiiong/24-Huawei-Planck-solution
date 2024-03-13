import sys
from enum import Enum
from typing import List, Dict
from queue import LifoQueue, PriorityQueue

from log import logger, My_Timer
from path_planing import Point, UNREACHABLE
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
    UnableBackBerth = 21

    GotoFetchFromBerth = 3

    GotGoods = 4

    CollisionAvoidance = 5

priority_for_robot_extended_status = {
    Robot_Extended_Status.BackBerthAndPull: 200,

    Robot_Extended_Status.GotoFetchFromBerth: 300,

    Robot_Extended_Status.OnBerth: 400
}

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

        self.is_collision_avoiding = False
        self.alarming_area_size = 2
        self.alarming_area = set()
        self.collision_robots: Dict[int, int]= dict()
        self.num_paths_predict = 2


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

    def cal_alarming_area(self, n, pose_list = [Point(0 ,0)]):
        if n == 0:
            for item in pose_list:
                self.alarming_area.add(item)
        else:
            for action in [Robot_Actions.UP, Robot_Actions.DOWN, Robot_Actions.LEFT, Robot_Actions.RIGHT]:
                tmp_set = []
                for item in pose_list:
                    tmp_set.append(item+action)
                for item in tmp_set:
                    self.alarming_area.add(item)
                self.cal_alarming_area(n-1, tmp_set[:])

    def collision_check(self, 
                move_matrix: List[List[Point]],
                robots = [],
                berths: List[Berth] = [], 
                target_pos: Point = Point(-1, -1)):
        # 检查区域是否有其他车辆
        timer = My_Timer()
        for move in self.alarming_area:
            test_pos = self.pos + move
            for i, robot in enumerate(robots):
                self.collision_robots = dict()
                if test_pos == robot.pos and i != self.robot_id:
                    self.collision_robots[i] = priority_for_robot_extended_status[robot.extended_status] + robot.robot_id
                    self.is_collision_avoiding = True
                    
        #logger.info("collision check time: %s", timer.click()*1000)
        
        # 根据未来的路径，车辆状态决定运动
            
        # 维护一个移动优先级队列，
    
    def DFS_path_planing(self, 
                move_matrix: List[List[Point]],
                robots = [],
                berths: List[Berth] = [], 
                target_pos: Point = Point(-1, -1)):
            
            timer = My_Timer()
            
            # 如果发生碰撞
            if (self.status == 0):
                self.extended_status = Robot_Extended_Status.StayStill

            if self.extended_status == Robot_Extended_Status.GotoFetchFromBerth:
                # 处理target pos不可达到
                if (self.paths_stk.empty() 
                    and (berths[self.berth_id].pos != target_pos) 
                    and (target_pos != UNREACHABLE)):
                    cur_pos = target_pos
                    self.paths_stk.put(cur_pos)
                    cur_pos = cur_pos + move_matrix[cur_pos.y][cur_pos.x]
                    while (cur_pos != berths[self.berth_id].pos):
                        self.paths_stk.put(cur_pos)
                        # logger.info("debug %s | %s", cur_pos, berths[berth_id].pos)
                        cur_pos = cur_pos + move_matrix[cur_pos.y][cur_pos.x]

            elif self.extended_status == Robot_Extended_Status.BackBerthAndPull:
                # 若无法返回港口的，先设置为不可达状态，再map到其他港口？
                if move_matrix[self.pos.y][self.pos.x] == UNREACHABLE:
                    self.extended_status = Robot_Extended_Status.UnableBackBerth
                elif self.paths_stk.empty() and (self.pos != berths[self.berth_id].pos):
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

            elif self.extended_status in [Robot_Extended_Status.OnBerth,
                                          Robot_Extended_Status.StayStill,
                                          Robot_Extended_Status.UnableBackBerth]:
                self.paths_stk = LifoQueue()
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
            next_pos = self.paths_stk.get(False)
            action = next_pos - self.pos
            if (action not in robot_action_value_to_cmd):
                logger.info("下一步action出错，不在预定的actions中，实际为%s， cur %s", action, self.pos)
                # self.extended_status = Robot_Extended_Status.BackBerthAndPull
                return
        else:
            next_pos = self.pos
            action = Robot_Actions.HOLD

        print_move_cmd()

        if self.extended_status == Robot_Extended_Status.GotoFetchFromBerth:
            # if (next_pos == target_pos):
            if (self.paths_stk.empty()):
                # 可能取不到货，货已经消失
                print("get", self.robot_id)
                self.extended_status = Robot_Extended_Status.GotGoods
        
        elif self.extended_status == Robot_Extended_Status.BackBerthAndPull:
            # if (next_pos == berths[self.berth_id].pos):
            if (self.paths_stk.empty()):
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
        #self.collision_check(move_matrix, robots, berths, target_pos)
        self.paths_execution(move_matrix, robots, berths, target_pos)