from __future__ import annotations
import sys
import random
from enum import Enum
from typing import List, Dict
from queue import LifoQueue, PriorityQueue, Queue
import copy

from log import logger, error_logger, My_Timer
from path_planing import Point, UNREACHABLE
from path_planing import Robot_Actions
from path_planing import one_move_avoidance

from .env import Env
from .berth import Goods
from .utils import enum_stk_and_recover, enum_stk_and_empty, enum_stk

robot_action_value_to_cmd = {
    Robot_Actions.RIGHT : 0,
    Robot_Actions.LEFT  : 1,
    Robot_Actions.UP    : 2,
    Robot_Actions.DOWN  : 3,
    Robot_Actions.HOLD  : 4,
}

class Robot_Extended_Status(Enum):
    Uninitialized = -1
    BackBerthAndPull = 2
    UnableBackBerth = 21
    OnBerth = 1
    GotoFetchFromBerth = 3
    GotGoods = 4
    CollisionAvoidance = 5
    Recovery = 6


priority_for_robot_extended_status = {
    Robot_Extended_Status.UnableBackBerth : -100,
    Robot_Extended_Status.Recovery: -50,
    Robot_Extended_Status.CollisionAvoidance: 0,
    Robot_Extended_Status.GotGoods: 100,
    Robot_Extended_Status.BackBerthAndPull: 200,
    Robot_Extended_Status.GotoFetchFromBerth: 300,
    Robot_Extended_Status.OnBerth: 400,
    Robot_Extended_Status.Uninitialized : 500
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

        # 在第一帧开始前初始化的内容
        self.env = Env()
        self.robot_id = -1
        self.berth_id = -1

        # 状态，每个状态经过规划、避障、执行；状态更新在第二帧调度前进行
        self._extended_status = Robot_Extended_Status.Uninitialized
        # 路径规划用，定义为接下来要经过的位置，空代表呆在原地
        self.paths_stk: LifoQueue[Point] = LifoQueue()

        # 避障用的变量
        self.alarming_area_size = 2
        self.surronding_robots_with_priority: Dict[int, int]= dict()
        self.collision_robots_id: List[int] = []
        # 进入避障状态时保存原有状态的变量 
        self.original_extended_status = Robot_Extended_Status.Uninitialized
        self.original_paths_stk = LifoQueue()
        # 为记录走过的路用
        self.last_pos = Point(x = startX, y = startY)
        self.num_paths_predict = 3

        # 碰撞后恢复用
        self.last_status = 1
        self.empty_paths = True
        self.suppose_pos = Point(x = startX, y = startY)

        # 解决单行道问题
        self.priority: int = -1
        self.master_robot_id: int = -1

        # 取货用
        self.target_gds: Goods = Goods(0, [0])

    # 兼容原有代码用
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

    def convert_extended_status(self, value: Robot_Extended_Status):
        berths = self.env.berths
        valid = True
        last_status = self.extended_status

        if (value == Robot_Extended_Status.BackBerthAndPull):
            # 转入条件：
            #   目标港口非不可达
            # 状态要求：
            #   priority, _extended_status, paths_stk
            if (self.env.move_matrix_list[self.berth_id][self.y][self.x] != UNREACHABLE):
                self.priority = priority_for_robot_extended_status[value]
                self._extended_status = value
                self.path_update()
            else:
                valid = False
        
        elif (value == Robot_Extended_Status.OnBerth):
            # 转入条件：
            #   当前处于港口
            # 状态要求：
            #   当前处于港口
            #   paths_stk为空
            #   priority, _extended_status, 
            if (self.pos == self.env.berths[self.berth_id].pos):
                self.priority = priority_for_robot_extended_status[value]
                self._extended_status = value
                self.path_update()
            else:
                valid = False
            
        elif (value == Robot_Extended_Status.GotoFetchFromBerth):
            # 转入条件：
            #   当前处于港口
            #   由go_to_fetch_gds_from_berth启动，设置target_gds
            # 状态要求：
            #   priority, _extended_status, paths_stk, target_gds
            if (self.pos == self.env.berths[self.berth_id].pos):
                self.priority = priority_for_robot_extended_status[value]
                self._extended_status = value
                self.path_update()
            else:
                valid = False

        elif (value == Robot_Extended_Status.GotGoods):
            # 转入条件：
            #   必须由GotoFetchFromBerth转入
            # 状态要求：
            #   priority, _extended_status, paths_stk
            if self.extended_status == Robot_Extended_Status.GotoFetchFromBerth:
                self.priority = priority_for_robot_extended_status[value]
                self._extended_status = value
                self.path_update()
            else:
                valid = False

        elif (value == Robot_Extended_Status.UnableBackBerth):
            # 转入条件：
            #   目标港口不可达
            # 状态要求：
            #   priority, _extended_status, paths_stk
            if (self.env.move_matrix_list[self.berth_id][self.pos.y][self.pos.x] == UNREACHABLE):
                self.priority = priority_for_robot_extended_status[value]
                self._extended_status = value
                self.path_update()
            else:
                valid = False

        elif (value == Robot_Extended_Status.CollisionAvoidance):
            # 转入条件：
            #   发生碰撞，且为低优先级
            # 状态要求：
            #   需要为负责对象让路
            #   priority, _extended_status, paths_stk
            #   responsible_robot_id
            self.priority = priority_for_robot_extended_status[value]
            self._extended_status = value
            self.path_update()
        
        if valid:
            error_logger.error("zhen: %s id: %s from %s to %s",
                               self.env.global_zhen, self.robot_id, last_status, value)
            return True
        else:
            error_logger.error("zhen:%s id: pos: %s \n    状态转换出错，由%s, 到%s",
                                self.env.global_zhen, self.robot_id, self.pos, last_status, value)
            return False

    def go_to_fetch_gds_from_berth(self, target_gds: Goods):
        self.target_gds = target_gds
        self.convert_extended_status(Robot_Extended_Status.GotoFetchFromBerth)

    def get_priority_for_A(self, A_robot: Robot):
        if ((self.extended_status == Robot_Extended_Status.CollisionAvoidance) 
            and (self.master_robot_id != A_robot.robot_id)):
            return 600 + self.robot_id
        elif ((self.extended_status == Robot_Extended_Status.CollisionAvoidance) 
              and (self.master_robot_id == A_robot.robot_id)):
            return -100 + self.robot_id
        else:
            return priority_for_robot_extended_status[A_robot.extended_status] + A_robot.robot_id
        
    # 考虑避障时看到原来的路径
    def next_n_pos(self, n:int = 1) -> List[Point]:
        poses: List[Point] = []
        count = 0
        final_pos = Point(self.pos.x, self.pos.y)
        
        for stk in [self.paths_stk, self.original_paths_stk]:
            for next_pos in enum_stk_and_recover(stk):
                if count < n:
                    poses.append(next_pos)
                    final_pos = next_pos
                    count += 1
        while count < n:
            poses.append(final_pos)
            count += 1

        return poses

    def collision_check(self):  
        for robot in self.env.robots:
            # 如果警戒范围内存在其他机器人
            distance = self.pos.distance(robot.pos)
            if  (distance <= self.alarming_area_size) and (robot.robot_id != self.robot_id):
                pos1 = self.next_n_pos(1)[0]
                pos2 = robot.next_n_pos(1)[0]
                if (pos1 == pos2 or (pos1 == robot.pos and pos2 == self.pos)):
                    return True
        return False
    
    def collision_check_and_update(self):        
        # 每一帧都初始化 附近的机器人 and 会碰撞的机器人
        # {robot_id : priority, ...}
        self.surronding_robots_with_priority = {self.robot_id: self.get_priority_for_A(self)}
        # [robot_id, ...]
        self.collision_robots_id = [self.robot_id]

        # 检查警戒范围内是否有其他机器人 
        robots = self.env.robots               
        for robot in robots:
            # 如果警戒范围内存在其他机器人
            distance = self.pos.distance(robot.pos)
            if  (distance <= self.alarming_area_size) and (robot.robot_id != self.robot_id):
                # 将机器人加入surrounding字典，并让其value为优先级
                self.surronding_robots_with_priority[robot.robot_id] = self.get_priority_for_A(robot)
                # 如果那个机器人下一帧会与自己碰撞，则加入碰撞机器人队列
                pos1 = self.next_n_pos(1)[0]
                pos2 = robot.next_n_pos(1)[0]
                if (pos1 == pos2 or (pos1 == robot.pos and pos2 == self.pos)):
                    self.collision_robots_id.append(robot.robot_id)
                    
        if (len(self.collision_robots_id) > 1):
            return True
        else:
            return False

    def find_responsible_robot_id(self):
        pass

    def collision_avoid(self):
        robots = self.env.robots
        move_matrix = self.env.move_matrix_list[self.berth_id]
        okk = True
        # 如果存在会碰撞的机器人
        if (self.collision_check_and_update()):
            max_priority = -1
            max_id = -1
            for id in self.collision_robots_id:
                if self.surronding_robots_with_priority[id] > max_priority:
                    max_id = id
                    max_priority = self.surronding_robots_with_priority[id]

            for id in self.collision_robots_id:
                # 当前robot非优先级最高
                if (id != max_id):
                    # 如果还未进入避障状态，启动避障状态，并重新计算路径
                    if (robots[id].extended_status != Robot_Extended_Status.CollisionAvoidance):
                        robots[id].enable_collision_avoidance(max_id)
                    else:
                        robots[id].path_update()
                        # robots[id].path_planing()
            # 另起一行，为的是在执行最高优先级之前，所有低优先级的路径已经规划好    
            for id in self.collision_robots_id:
                # 最高优先级的对象
                if (id == max_id):
                    # 如果正在避障，不可以直接解除避障状态，因为他可能正在必然当前不可见的更高优先级的robot
                    # 如果机器人不撞了，则认为可以解除避障
                    # collision check修复了避障时无法看见原来路径的错误！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
                    if (robots[id].extended_status == Robot_Extended_Status.CollisionAvoidance
                        and robots[id].collision_check() is False
                        ):# and robots[id].paths_stk.empty()
                        robots[id].try_disable_collision_avoidance()
                    # 该最高优先级的部分区域已经让路，如何考虑其他区域？
                    # 暂时不考虑？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？
                    # elif(robots[id].extended_status != Robot_Extended_Status.CollisionAvoidance
                    #      and robots[id].collision_check() is True):
                    #     robots[id].enable_collision_avoidance(move_matrix, target_pos)
        return okk
        # 如果没有会撞得，且当前为避障模式，则恢复原来模式 xxxxxxxxx错误，会导致循环
        # elif (self.extended_status == Robot_Extended_Status.CollisionAvoidance):
        #     self.disable_collision_avoidance(move_matrix, robots, berths, target_pos)

    def try_find_avoidance_path(self):
        
        success = False
        avoidance_paths_stk = LifoQueue()
        # collision_check同时会更新self.surrounding, self.collision_robots_ud

        if (self.collision_check() == False):
            success = True
            return success, avoidance_paths_stk
        
        avoidance_matrix_len = 5 # 7/2
        # 获得障碍图
        l = self.pos.x - avoidance_matrix_len
        r = self.pos.x + avoidance_matrix_len
        t = self.pos.y - avoidance_matrix_len
        b = self.pos.y + avoidance_matrix_len
        if l < 0: l = 0
        if r > 199:  r = 199
        if t < 0: t = 0
        if b > 199: b = 199

        avoidance_matrix: List[List[int]] = []
        col = -1
        for y in range(t, b+1):
            avoidance_matrix.append([])
            col += 1
            for x in range(l, r+1):
                avoidance_matrix[col].append(self.env.value_matrix[y][x])
        
        # 1. 尝试针对所有周围的对象进行避障
        predict_steps = 2
        list_avoidance_paths = []
        while (success == False and predict_steps > 0):
            ins_avoidance_matrix = copy.deepcopy(avoidance_matrix)
            # 将机器人的位置和其路径视为障碍物
            for robot_id in self.surronding_robots_with_priority:
                # 不考虑自己
                if robot_id == self.robot_id:
                    continue
                robot = self.env.robots[robot_id]
                poses: List[Point] = []
                poses.append(Point(x = robot.x, y = robot.y))
                poses += robot.next_n_pos(predict_steps)
                for pos in poses:
                    ins_avoidance_matrix[pos.y-t][pos.x-l] = 0

            # 尝试一条避障路径
            list_avoidance_paths, success = one_move_avoidance(ins_avoidance_matrix,
                                                            Point(self.pos.x-l, self.pos.y-t))
            predict_steps -= 1
        

        # 2. 如果无法避障成功, 尝试避开master robot
        robots = self.env.robots
        if success == False:
            success_second = False
            predict_steps = 2
            list_avoidance_paths = []
            while (success_second == False and predict_steps > 0):
                ins_avoidance_matrix = copy.deepcopy(avoidance_matrix)
                # 只考虑master robot
                robot = self.env.robots[self.master_robot_id]
                poses: List[Point] = []
                poses.append(Point(x = robot.x, y = robot.y))
                poses += robot.next_n_pos(predict_steps)
                for pos in poses:
                    ins_avoidance_matrix[pos.y-t][pos.x-l] = 0
                # 尝试一条避障路径
                list_avoidance_paths, success_second = one_move_avoidance(ins_avoidance_matrix,
                                                            Point(self.pos.x-l, self.pos.y-t))
        
        for item in list_avoidance_paths:
            item = Point(item.x + l, item.y + t)
            avoidance_paths_stk.put(item)

        return success, avoidance_paths_stk

        # 尝试规划避让路径

    def gen_recoverable_paths(self, following_stk: LifoQueue):
        # 如果呆在原地，则会导致生成两步原地？？？？？？？？？？  
        cur_pos = Point(self.pos.x, self.pos.y)
        reverse_stk = LifoQueue()

        if (following_stk.empty()):
            return

        # 构造回溯位置，包括原始位置
        for item in enum_stk_and_recover(following_stk):
            reverse_stk.put(item)
        # 对避障路径为原点进行特殊处理，
        stay = False
        if (reverse_stk.qsize() == 1):
            for item in enum_stk_and_recover(reverse_stk):
                if item == cur_pos:
                    stay = True
        if not stay:
            reverse_stk.put(cur_pos)

        following_stk.get() # 去除一个重复的终点
        # 将新的回归路径拼接到原来的回归路径上
        last_index = -1
        for i, item in enumerate(enum_stk_and_recover(self.paths_stk)):
            if (item == cur_pos):
                last_index = i
        if (last_index != -1):
            for i, item in enumerate(enum_stk(self.paths_stk)):
                if i == last_index:
                    break

        for item in enum_stk(reverse_stk):
            self.paths_stk.put(item)
        for item in enum_stk(following_stk):
            self.paths_stk.put(item)

    # 启动避障状态，该状态会通过 搜索算法 尽可能远离其他机器人的位置，并记录移动的路径
    # 启动时会直接重新计算路径
    def enable_collision_avoidance(self, master_robot_id: int):
                
        self.master_robot_id = master_robot_id
        self.original_extended_status = self.extended_status
        self.original_paths_stk = self.paths_stk
        self.paths_stk = LifoQueue()
        self.convert_extended_status(Robot_Extended_Status.CollisionAvoidance)
    
    # 退出避障
    # 将collision期间的paths附加到原来状态的paths上
    def try_disable_collision_avoidance(self):
        # 如果处于这些状态，如果paths_stk中还存在其他步骤，那么回归这些状态时，他们的位置在该在的位置的假设将错误
        if (self.extended_status != Robot_Extended_Status.CollisionAvoidance):
            error_logger.error("尝试从避障转入避障，错误")
            return 
        if self.original_extended_status in [Robot_Extended_Status.OnBerth, Robot_Extended_Status.GotGoods]:
            if (not self.paths_stk.empty()):
                return False
            else:
                self.paths_stk = self.original_paths_stk
                self.original_paths_stk = LifoQueue()
        # 对于back、fetch、unableback、等将需要接下来的路直接加入原paths？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？
        else:
            tmp_stk = LifoQueue()
            while not self.paths_stk.empty():
                tmp_stk.put(self.paths_stk.get())
            self.paths_stk = self.original_paths_stk
            while not tmp_stk.empty():
                self.paths_stk.put(tmp_stk.get())
            self.original_paths_stk = LifoQueue()

        self._extended_status = self.original_extended_status
        return True
        #self.run(move_matrix, robots, berths, target_pos)

    def path_update(self):  
            move_matrix = self.env.move_matrix_list[self.berth_id]
            berths = self.env.berths

            # 更新路径为到目标点的路径
            if self.extended_status == Robot_Extended_Status.GotoFetchFromBerth:
                target_pos = self.target_gds.pos
                if (move_matrix[target_pos.y][target_pos.x] == UNREACHABLE):
                    self.convert_extended_status(Robot_Extended_Status.OnBerth)
                elif (berths[self.berth_id].pos == target_pos): 
                    self.paths_stk = LifoQueue()
                else:
                    self.paths_stk = LifoQueue()
                    cur_pos = target_pos
                    self.paths_stk.put(cur_pos)
                    cur_pos = cur_pos + move_matrix[cur_pos.y][cur_pos.x]
                    while (cur_pos != berths[self.berth_id].pos):
                        self.paths_stk.put(cur_pos)
                        cur_pos = cur_pos + move_matrix[cur_pos.y][cur_pos.x]
                
            elif self.extended_status == Robot_Extended_Status.BackBerthAndPull:
               
                if (self.pos == berths[self.berth_id].pos):
                    self.paths_stk = LifoQueue()
                else:
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
                success, tmp_paths_stk = self.try_find_avoidance_path()
                # 尝试能够回溯的避障路径
                self.gen_recoverable_paths(tmp_paths_stk)

                if success is False:
                    robots = self.env.robots
                    # 针对新规划的路径（只对负责的对象避障）进行避障
                    self.collision_check_and_update()
                    for robot_id in self.collision_robots_id:
                        if robot_id != self.robot_id:
                            if self.get_priority_for_A(robots[robot_id]) < self.get_priority_for_A(self):
                                if (robots[robot_id].extended_status != Robot_Extended_Status.CollisionAvoidance):
                                    robots[robot_id].enable_collision_avoidance(self.robot_id)
                                else:
                                    robots[robot_id].path_update()


            elif self.extended_status in [Robot_Extended_Status.OnBerth, 
                                          Robot_Extended_Status.GotGoods,
                                          Robot_Extended_Status.UnableBackBerth]:
                self.paths_stk = LifoQueue()
            
            return True

    def paths_execution(self):
        
        if not self.paths_stk.empty():
            self.empty_paths = False
            next_pos = self.paths_stk.get(False)
            action = next_pos - self.pos
            if (action not in robot_action_value_to_cmd):
                error_logger.error("zhen：%s，id %s ,cur_pos: %s, 目标位置: %s", self.env.global_zhen, self.robot_id,self.pos, next_pos)
                return
        else:
            self.empty_paths = True
            next_pos = self.pos
            action = Robot_Actions.HOLD
        
        self.last_pos = self.pos
        self.suppose_pos = next_pos
        
        if (action != Robot_Actions.HOLD):
            print("move", self.robot_id, robot_action_value_to_cmd[action])

    # 根据状态的执行结果改变状态，区别于run中的状态变化（如何区别？）
    def update_extended_status(self):
        '''在规划、执行后进行，在下一帧进行更新；
            当前帧的状态更新放在下一帧的最开始，
            是因为机器人的状态，下一帧才能被更新'''
        berths = self.env.berths
        # 避免碰撞时self.path_stks错误，根据status恢复
        if self.status == 0 or self.last_status == 0: # 若当前帧刚解除，但上一帧是眩晕状态，也需要恢复
            self.last_status = self.status
            if self.pos != self.suppose_pos:
                self.paths_stk.put(self.suppose_pos)
            elif self.empty_paths == False:
                self.paths_stk.put(self.suppose_pos)

            # if (self.extended_status != Robot_Extended_Status.CollisionAvoidance
            # and self.collision_check() == True):
            #     self.enable_collision_avoidance()
        
        elif self.extended_status == Robot_Extended_Status.GotoFetchFromBerth:
            if (self.paths_stk.empty()):
                print("get", self.robot_id)
                self.convert_extended_status(Robot_Extended_Status.GotGoods)
        
        elif self.extended_status == Robot_Extended_Status.BackBerthAndPull:
            if (self.paths_stk.empty()):
                self.convert_extended_status(Robot_Extended_Status.OnBerth)
                if self.goods == 1:
                    print("pull", self.robot_id)
                    berths[self.berth_id].cur_num_gds += 1
                    berths[self.berth_id].total_num_gds += 1
                    berths[self.berth_id].total_earn += 180
        
        # 如果发生碰撞，则不会移动，所以不需要担心避障时碰撞导致无法记录路径
        elif self.extended_status == Robot_Extended_Status.CollisionAvoidance:
            if (self.collision_check() == False):
                    self.try_disable_collision_avoidance()

    def change_berth(self, new_berth_id: int):
        move_matrix = self.env.move_matrix_list[new_berth_id]
        if (move_matrix[self.y][self.x] != UNREACHABLE):
            self.berth_id = new_berth_id
            self.paths_stk = LifoQueue()
            self.convert_extended_status(Robot_Extended_Status.BackBerthAndPull)
        else:
            error_logger.error("new berth unreachable")

    def debug_robot(self):
        # pass
        logger.info("at zhen %d", self.env.global_zhen)
        logger.info("robot %d, at (%d, %d) with state %d Extstate %s", self.robot_id, self.pos.x, self.pos.y, self.status, self.extended_status)
        logger.info("   previous Extstate: %s", self.original_extended_status)
        if self.paths_stk.empty():
            logger.info("   path_stk empty")
        next_pose = self.next_n_pos(1)[0]
        logger.info("   next pos is (%d, %d)", next_pose.x, next_pose.y)
        if self.original_paths_stk.empty():
            logger.info("   original_paths_stk empty")
        