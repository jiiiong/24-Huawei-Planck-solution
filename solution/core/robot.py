import sys
from enum import Enum
from typing import List
from queue import LifoQueue

from log import logger
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

class Args_Robot_Run():
    def __init__(self) -> None:
        robot_id: int
        move_matrix: List[List[Point]]
        berth_id: int = 0
        robots = []
        berths = []
        target_pos: Point = Point(-1, -1)

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
        
        self.paths_stk = LifoQueue()
        self.num_paths_predict = 5

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

        self.collision_check()

    def back_berth(self, robot_id: int, move_matrix: List[List[Point]],
                            berth_id: int = 0, berths = []):
        action = move_matrix[self.y][self.x]
        # 泵碰撞？/
        if (action not in robot_action_value_to_cmd):
            self.extended_status = Robot_Extended_Status.BackBerth
            return
        if self.pos != berths[berth_id].pos:
            print("move", robot_id, robot_action_value_to_cmd[action])
        else:
            self.extended_status = Robot_Extended_Status.OnBerth

    def back_berth_and_pull(self, robot_id: int, move_matrix: List[List[Point]],
                            berth_id: int = 0, berths = []):
        action = move_matrix[self.y][self.x]
        
        # 碰撞？？？
        if (action not in robot_action_value_to_cmd):
            self.extended_status = Robot_Extended_Status.BackBerth
            return
        
        if self.pos != berths[berth_id].pos:
            print("move", robot_id, robot_action_value_to_cmd[action])
            if (self.pos + action == berths[berth_id].pos):
                print("pull", robot_id)
        else:
            berths[berth_id].num_gds += 1
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
                if (action not in robot_action_value_to_cmd):
                    self.context_go_to_fetch_from_berth.quit()
                    self.extended_status = Robot_Extended_Status.BackBerth
                    return
                print("move", robot_id, robot_action_value_to_cmd[action])
                if (next_pos == target_pos):
                    print("get", robot_id)
            elif (self.pos == target_pos):
                self.extended_status = Robot_Extended_Status.GotGoods
                self.context_go_to_fetch_from_berth.quit()
    
    def collision_check(self):
        pass
    
    def path_planing(self, robot_id: int, move_matrix: List[List[Point]],
            berth_id: int = 0, 
            robots = None, 
            berths = None,  
            target_pos: Point = Point(-1, -1)):
        if self.extended_status == Robot_Extended_Status.BackBerth:
            self.back_berth(robot_id, move_matrix, berth_id=berth_id, berths=berths)

        elif self.extended_status == Robot_Extended_Status.GotoFetchFromBerth:
            self.go_to_fetch_from_berth(robot_id, move_matrix, target_pos=target_pos)

        elif self.extended_status == Robot_Extended_Status.BackBerthAndPull:
            self.back_berth_and_pull(robot_id, move_matrix, berth_id=berth_id, berths=berths)
    
    def DFS_path_planing(self, robot_id: int, move_matrix: List[List[Point]],
            berth_id: int = 0, 
            robots = [], 
            berths: List[Berth] = [],  
            target_pos: Point = Point(-1, -1)):

            if self.extended_status == Robot_Extended_Status.GotoFetchFromBerth:
                # 如何处理target pos不可达到
                self.paths_stk = LifoQueue()

                cur_pos = target_pos
                self.paths_stk.put(cur_pos)
                cur_pos = cur_pos + move_matrix[cur_pos.y][cur_pos.x]
                # robot当前位置要么是港口，要么是沿着之前BFS的路径走了一段距离
                while (cur_pos != berths[berth_id].pos and cur_pos != self.pos):
                    self.paths_stk.put(cur_pos)
                    # logger.info("debug %s | %s", cur_pos, berths[berth_id].pos)
                    cur_pos = cur_pos + move_matrix[cur_pos.y][cur_pos.x]

            elif self.extended_status == Robot_Extended_Status.BackBerthAndPull:
                self.paths_stk = LifoQueue()
                tmp_stk = LifoQueue()
                cur_pos = self.pos
                cur_pos = cur_pos + move_matrix[cur_pos.y][cur_pos.x]
                while (cur_pos != berths[berth_id].pos): # 此时self.pos就是在berth上
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

    def paths_execution(self, robot_id: int, move_matrix: List[List[Point]],
            berth_id: int = 0, 
            robots = [], 
            berths: List[Berth] = [],  
            target_pos: Point = Point(-1, -1)):
        
        def print_move_cmd():
            if (action != Robot_Actions.HOLD):
                print("move", robot_id, robot_action_value_to_cmd[action])

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
                print("get", robot_id)
                self.extended_status = Robot_Extended_Status.GotGoods
        
        elif self.extended_status == Robot_Extended_Status.BackBerthAndPull:
            print_move_cmd()
            if (next_pos == berths[berth_id].pos):
                self.extended_status = Robot_Extended_Status.OnBerth
                if self.goods == 1:
                    print("pull", robot_id)
                    berths[berth_id].num_gds += 1
        
        elif self.extended_status == Robot_Extended_Status.OnBerth:
            pass
        
    def run_new(self, robot_id: int, move_matrix: List[List[Point]],
            berth_id: int = 0, 
            robots = [], 
            berths: List[Berth] = [],  
            target_pos: Point = Point(-1, -1)):
        # robot根据状态执行每一帧的动作
        self.DFS_path_planing(robot_id, move_matrix, berth_id, robots, berths, target_pos)
        self.collision_check()
        self.paths_execution(robot_id, move_matrix, berth_id, robots, berths, target_pos)