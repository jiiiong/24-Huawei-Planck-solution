from __future__ import annotations
import sys
from enum import Enum
from typing import List, Dict
from queue import LifoQueue, PriorityQueue, Queue

from log import logger, error_logger, My_Timer
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
    Uninitialized = -1
    BackBerthAndPull = 2
    UnableBackBerth = 21
    OnBerth = 1
    GotoFetchFromBerth = 3
    GotGoods = 4
    CollisionAvoidance = 5

priority_for_robot_extended_status = {
    Robot_Extended_Status.UnableBackBerth : -200,
    Robot_Extended_Status.Uninitialized : -100,
    Robot_Extended_Status.CollisionAvoidance: 0,
    Robot_Extended_Status.GotGoods: 100,
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
        self.berths: List[Berth] = []
        self._extended_status = Robot_Extended_Status.Uninitialized
        self.paths_stk = LifoQueue()

        self.alarming_area_size = 2
        self.alarming_area = set()
        # self.surronding_robots_with_priority: Dict[int, int]= dict()
        # self.collision_robots_id: List[int] = []
        self.original_extended_status = Robot_Extended_Status.Uninitialized
        self.original_paths_stk = LifoQueue()
        self.original_pos = None
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
    
    @property
    def extended_status(self):
        return self._extended_status
    @extended_status.setter
    # 检测状态转换是否正确，仅做转换错误的输出，不做实际限制
    def extended_status(self, value: Robot_Extended_Status):
        invalid = False
        msg = ""

        # 目标状态为BackBerthAndPull
        if (value == Robot_Extended_Status.BackBerthAndPull):
            if (self.extended_status == Robot_Extended_Status.Uninitialized
                and self.paths_stk.empty()):
                pass
            elif (self.extended_status == Robot_Extended_Status.GotGoods
                  and self.paths_stk.empty()):
                pass
            else:
                invalid = True
        # 目标状态为Onberth时
        elif (value == Robot_Extended_Status.OnBerth):
            # 上一个状态为BackBerthAndPull，且到达目的地
            if (self.extended_status == Robot_Extended_Status.BackBerthAndPull 
                and self.paths_stk.empty() 
                and self.pos == self.berths[self.berth_id].pos):
                pass
            else:
                invalid = True
        # 目标状态为GotoFetchFromBerth
        elif (value == Robot_Extended_Status.GotoFetchFromBerth):
            if (self.extended_status == Robot_Extended_Status.OnBerth
                and self.paths_stk.empty()):
                pass
            else:
                invalid = True
        # 目标状态为GotGoods
        elif (value == Robot_Extended_Status.GotGoods):
            # 上一个状态为BackBerthAndPull，且到达目的地
            if (self.extended_status == Robot_Extended_Status.GotoFetchFromBerth
                and self.paths_stk.empty() ):
                # and self.pos == self.berths[self.berth_id].pos
                pass
            else:
                invalid = True
        # 目标状态为UnableBackBerth
        elif (value == Robot_Extended_Status.UnableBackBerth):
            if (self.extended_status == Robot_Extended_Status.BackBerthAndPull
                and self.paths_stk.empty()):
                pass
            else:
                invalid = True

        if invalid:
            error_logger.error("机器人%s 状态转换出错，由%s, 到%s\n其他信息: %s",
                                self.robot_id, self._extended_status, value, msg)   
            
        self._extended_status = value

    def next_n_pos(self, n:int = 1) -> List[Point]: 
        poses: List[Point] = []
        count = 0
        tmp_fifoqueue = LifoQueue()
        final_pos = self.pos
        while (not self.paths_stk.empty()) and (count < n):
            tmp = self.paths_stk.get()
            poses.append(tmp)
            final_pos = tmp
            tmp_fifoqueue.put(tmp)
            count += 1
        while count < n:
            poses.append(final_pos)
            count += 1
        while (not tmp_fifoqueue.empty()):
            self.paths_stk.put(tmp_fifoqueue.get())
        return poses
        # logger.info("next n for %s, \n%s", robot.robot_id, poses)

    # 启动避障状态，该状态会通过 搜索算法 尽可能远离其他机器人的位置，并记录移动的路径
    def enable_collision_avoidance(self, 
                move_matrix: List[List[Point]],
                robots: List[Robot] = [],
                berths: List[Berth] = [], 
                target_pos: Point = Point(-1, -1)):
        self.original_extended_status = self.extended_status
        self.original_paths_stk = self.paths_stk
        self.paths_stk = LifoQueue()
        self.extended_status = Robot_Extended_Status.CollisionAvoidance

    def disable_collision_avoidance(self):
        self.extended_status = self.original_extended_status
        self.paths_stk = self.original_paths_stk

    def collision_check(self, 
                move_matrix: List[List[Point]],
                robots: List[Robot] = [],
                berths: List[Berth] = [], 
                target_pos: Point = Point(-1, -1)):
        timer = My_Timer()
        
        # 每一帧都初始化 附近的机器人 and 会碰撞的机器人
        # {robot_id : priority, ...}
        surronding_robots_with_priority = {self.robot_id: priority_for_robot_extended_status[self.extended_status] + self.robot_id }
        # [robot_id, ...]
        collision_robots_id = [self.robot_id]

        # # 检查警戒范围内是否有其他机器人
        # for move in self.alarming_area:
        #     test_pos = self.pos + move
        #     for _, robot in enumerate(robots):
        #         # 如果警戒范围内存在其他机器人
        #         if test_pos == robot.pos and robot.robot_id != self.robot_id:
        #             # 将机器人加入surrounding字典，并让其value为优先级
        #             surronding_robots_with_priority[robot.robot_id] = priority_for_robot_extended_status[robot.extended_status] + robot.robot_id
        #             # 如果那个机器人下一帧会与自己碰撞，则加入碰撞机器人队列
        #             if (self.next_n_pos(self, 1) == self.next_n_pos(robot, 1)):
        #                 collision_robots_id.append(robot.robot_id)

        # 检查警戒范围内是否有其他机器人                
        for robot in robots:
            # 如果警戒范围内存在其他机器人
            if  (self.pos.distance(robot.pos) <= self.alarming_area_size) and (robot.robot_id != self.robot_id) :
                # 将机器人加入surrounding字典，并让其value为优先级
                surronding_robots_with_priority[robot.robot_id] = priority_for_robot_extended_status[robot.extended_status] + robot.robot_id
                # 如果那个机器人下一帧会与自己碰撞，则加入碰撞机器人队列
                if (self.next_n_pos(1)[0] == robot.next_n_pos(1)[0]):
                    collision_robots_id.append(robot.robot_id)

        # 如果存在会碰撞的机器人
        if len(collision_robots_id) > 1:
            max_priority = -1
            max_id = -1
            for id in collision_robots_id:
                if surronding_robots_with_priority[id] > max_priority:
                    max_id = id
                    max_priority = surronding_robots_with_priority[id]
            for id in collision_robots_id:
                if id != max_id:
                    robots[id].enable_collision_avoidance(move_matrix, robots, berths, target_pos)
        # 如果没有会撞得，且当前为避障模式，则恢复原来模式
        elif (self.extended_status == Robot_Extended_Status.CollisionAvoidance):
            self.disable_collision_avoidance()
        #logger.info("collision check time: %s", timer.click()*1000)

    def path_planing(self, 
                move_matrix: List[List[Point]],
                robots: List[Robot] = [],
                berths: List[Berth] = [], 
                target_pos: Point = Point(-1, -1)):
            
            timer = My_Timer()
            
            # GotoFetchFromBerth只能由OnBerth状态转入，OnBerth时Paths为空
            if self.extended_status == Robot_Extended_Status.GotoFetchFromBerth:
                # 根据BackBerth的性质，若Paths为空，要么是刚进入未初始化，或者是已经走到了，或者目标不可达
                # 排除其他两种情况，如果是刚进入未初始化，则初始化
                
                if (target_pos == UNREACHABLE):
                    pass # 处理target pos不可达到
                if (self.paths_stk.empty() 
                    and (berths[self.berth_id].pos != target_pos)): 
                    cur_pos = target_pos
                    self.paths_stk.put(cur_pos)
                    cur_pos = cur_pos + move_matrix[cur_pos.y][cur_pos.x]
                    while (cur_pos != berths[self.berth_id].pos):
                        self.paths_stk.put(cur_pos)
                        # logger.info("debug %s | %s", cur_pos, berths[berth_id].pos)
                        cur_pos = cur_pos + move_matrix[cur_pos.y][cur_pos.x]

            elif self.extended_status == Robot_Extended_Status.BackBerthAndPull:
                # 若无法返回港口的，则转入状态UnableBackBerth
                if move_matrix[self.pos.y][self.pos.x] == UNREACHABLE:
                    self.extended_status = Robot_Extended_Status.UnableBackBerth
                    self.run(move_matrix, robots, berths, target_pos)
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

            elif self.extended_status == Robot_Extended_Status.CollisionAvoidance:
                self.paths_stk = LifoQueue()

            elif self.extended_status in [Robot_Extended_Status.OnBerth, Robot_Extended_Status.UnableBackBerth]:
                self.paths_stk = LifoQueue()
            #logger.info("DFS cost: %s", timer.click()*1000)

    def paths_execution(self, 
                move_matrix: List[List[Point]],
                robots: List[Robot] = [],
                berths: List[Berth] = [], 
                target_pos: Point = Point(-1, -1)):
        if not self.paths_stk.empty():
            next_pos = self.paths_stk.get(False)
            action = next_pos - self.pos
            if (action not in robot_action_value_to_cmd):
                error_logger.error("下一步action出错，不在预定的actions中，实际为%s， cur %s", action, self.pos)
                # self.extended_status = Robot_Extended_Status.BackBerthAndPull
                return
        else:
            next_pos = self.pos
            action = Robot_Actions.HOLD

        if (action != Robot_Actions.HOLD):
            print("move", self.robot_id, robot_action_value_to_cmd[action])

    def extended_status_transfer(self, 
                move_matrix: List[List[Point]],
                robots: List[Robot] = [],
                berths: List[Berth] = [], 
                target_pos: Point = Point(-1, -1)):
        
        if self.extended_status == Robot_Extended_Status.GotoFetchFromBerth:
            if (self.paths_stk.empty()):
                # 可能取不到货，货已经消失
                print("get", self.robot_id)
                self.extended_status = Robot_Extended_Status.GotGoods
        
        elif self.extended_status == Robot_Extended_Status.BackBerthAndPull:
            if (self.paths_stk.empty()):
                self.extended_status = Robot_Extended_Status.OnBerth
                if self.goods == 1:
                    print("pull", self.robot_id)
                    berths[self.berth_id].num_gds += 1

        elif self.extended_status == Robot_Extended_Status.OnBerth:
            pass
        
    def run(self, 
                move_matrix: List[List[Point]],
                robots: List[Robot] = [],
                berths: List[Berth] = [], 
                target_pos: Point = Point(-1, -1)):
        # robot根据状态执行每一帧的动作
        self.path_planing(move_matrix, robots, berths, target_pos)
        # self.collision_check(move_matrix, robots, berths, target_pos)
        self.paths_execution(move_matrix, robots, berths, target_pos)

    def update_extended_status(self, 
                move_matrix: List[List[Point]],
                robots: List[Robot] = [],
                berths: List[Berth] = [], 
                target_pos: Point = Point(-1, -1)):
        self.extended_status_transfer(move_matrix, robots, berths, target_pos)


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
        
    def collision_check_pre(self, 
                move_matrix: List[List[Point]],
                robots: List[Robot] = [],
                berths: List[Berth] = [], 
                target_pos: Point = Point(-1, -1)):
        # 检查区域是否有其他车辆
        timer = My_Timer()
        
        # 每一帧都初始化 附近的机器人 and 会碰撞的机器人
        self.surronding_robots_with_priority = dict()
        self.surronding_robots_with_priority[self.robot_id] = priority_for_robot_extended_status[self.extended_status] + self.robot_id
        self.collision_robots_id = [self.robot_id]
        # 检查警戒范围内是否有其他机器人
        for move in self.alarming_area:
            test_pos = self.pos + move
            for i, robot in enumerate(robots):
                # 如果警戒范围内存在其他机器人
                if test_pos == robot.pos and i != self.robot_id:
                    # 将机器人加入surrounding字典，并让其value为优先级
                    self.surronding_robots_with_priority[i] = priority_for_robot_extended_status[robot.extended_status] + robot.robot_id
                    # 如果那个机器人下一帧会与自己碰撞，则加入碰撞机器人队列
                    if (self.next_n_pos(1) == robot.next_n_pos(1)):
                        self.collision_robots_id.append(robot.robot_id)
                    # 否则，不对该机器人做操作，表示下一步正常进行

        # 如果存在会碰撞的机器人
        if len(self.collision_robots_id) > 1:
            for id in self.collision_robots_id:
                pass
                    
        #logger.info("collision check time: %s", timer.click()*1000)
        
        # 根据未来的路径，车辆状态决定运动
            
        # 维护一个移动优先级队列，