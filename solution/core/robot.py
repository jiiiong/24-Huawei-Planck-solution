from __future__ import annotations
import sys
import random
from enum import Enum
from typing import List, Dict
from queue import LifoQueue, PriorityQueue, Queue

from log import logger, error_logger, My_Timer
from path_planing import Point, UNREACHABLE
from path_planing import Robot_Actions
from path_planing import one_move_avoidance

from .berth import Berth
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
        self.robot_id = -1
        self.berth_id = -1
        self.robots: List[Robot] = []
        self.berths: List[Berth] = []
        self.move_matrix_list: List[(List[List[Point]])] = []
        self.value_matrix: List[List[int]] = []

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
            elif (self.extended_status == Robot_Extended_Status.CollisionAvoidance): 
                ###############################可能还需要保证不再碰撞
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
            elif (self.extended_status == Robot_Extended_Status.CollisionAvoidance
                  and self.paths_stk.empty()):
                pass
            elif (self.extended_status == Robot_Extended_Status.GotoFetchFromBerth 
                and self.paths_stk.empty() 
                and self.pos == self.berths[self.berth_id].pos):
                pass
            else:
                invalid = True
        # 目标状态为GotoFetchFromBerth
        elif (value == Robot_Extended_Status.GotoFetchFromBerth):
            if (self.extended_status == Robot_Extended_Status.OnBerth
                and self.paths_stk.empty()
                and self.pos == self.berths[self.berth_id].pos):
                pass
            elif (self.extended_status == Robot_Extended_Status.CollisionAvoidance):
                pass
            else:
                invalid = True
        # 目标状态为GotGoods
        elif (value == Robot_Extended_Status.GotGoods):
            # 上一个状态为BackBerthAndPull，且到达目的地
            if (self.extended_status == Robot_Extended_Status.GotoFetchFromBerth
                and self.paths_stk.empty()):
                # and self.pos == self.berths[self.berth_id].pos
                pass
            elif (self.extended_status == Robot_Extended_Status.CollisionAvoidance
                  and self.paths_stk.empty()):
                pass
            else:
                invalid = True
        # 目标状态为UnableBackBerth
        elif (value == Robot_Extended_Status.UnableBackBerth):
            if (self.extended_status == Robot_Extended_Status.BackBerthAndPull
                and self.move_matrix_list[self.berth_id][self.pos.y][self.pos.x] == UNREACHABLE):
                pass
            elif (self.extended_status == Robot_Extended_Status.CollisionAvoidance):
                pass
            else:
                invalid = True

        elif (value == Robot_Extended_Status.CollisionAvoidance):
            if (self._extended_status == Robot_Extended_Status.CollisionAvoidance):
                invalid = True
        
        if invalid:
            error_logger.error("机器人%s  pos: %s \n    状态转换出错，由%s, 到%s",
                                self.robot_id, self.pos, self._extended_status, value)   
            
        self._extended_status = value

    # 考虑避障时看到原来的路径？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？
    def next_n_pos(self, n:int = 1) -> List[Point]:
        poses: List[Point] = []
        count = 0
        tmp_fifoqueue = LifoQueue()
        tmp_fifoqueue_for_original = LifoQueue()
        final_pos = Point(self.pos.x, self.pos.y)
        
        for stk in [self.paths_stk, self.original_paths_stk]:
            for next_pos in enum_stk_and_recover(self.paths_stk):
                if count < n:
                    poses.append(next_pos)
                    final_pos = next_pos
                    count += 1
        # 当前的
        # while (count < n) and (not self.paths_stk.empty()):
        #     tmp = self.paths_stk.get()
        #     poses.append(tmp)
        #     final_pos = tmp
        #     tmp_fifoqueue.put(tmp)
        #     count += 1
        # 避障时为原来的队列
        # 非避障是为空
        # while (count < n) and (not self.original_paths_stk.empty()):
        #     tmp = self.original_paths_stk.get()
        #     poses.append(tmp)
        #     final_pos = tmp
        #     tmp_fifoqueue_for_original.put(tmp)
        #     count += 1
        while count < n:
            poses.append(final_pos)
            count += 1
        # while (not tmp_fifoqueue_for_original.empty()):
        #     self.original_paths_stk.put(tmp_fifoqueue_for_original.get())
        # while (not tmp_fifoqueue.empty()):
        #     self.paths_stk.put(tmp_fifoqueue.get())

        return poses
        # logger.info("next n for %s, \n%s", robot.robot_id, poses)

    def collision_check(self, 
                move_matrix: List[List[Point]],
                target_pos: Point = Point(-1, -1)):
        timer = My_Timer()
        
        # 每一帧都初始化 附近的机器人 and 会碰撞的机器人
        # {robot_id : priority, ...}
        self.surronding_robots_with_priority = {self.robot_id: priority_for_robot_extended_status[self.extended_status] + self.robot_id }
        # [robot_id, ...]
        self.collision_robots_id = [self.robot_id]

        # 检查警戒范围内是否有其他机器人                
        for robot in self.robots:
            # 如果警戒范围内存在其他机器人
            distance = self.pos.distance(robot.pos)
            if  (distance <= self.alarming_area_size) and (robot.robot_id != self.robot_id) :
                # 将机器人加入surrounding字典，并让其value为优先级
                self.surronding_robots_with_priority[robot.robot_id] = priority_for_robot_extended_status[robot.extended_status] + robot.robot_id
                # 如果那个机器人下一帧会与自己碰撞，则加入碰撞机器人队列
                
                # windows = 1
                # poses1 = self.next_n_pos(windows)
                # poses2 = robot.next_n_pos(windows)
                # for _i in range(windows):
                # ❌，如果对撞，则会被检测为不碰撞
                #     if (poses1[_i] == poses2[_i]):
                #         self.collision_robots_id.append(robot.robot_id)
                #         break
                pos1 = self.next_n_pos(1)[0]
                pos2 = robot.next_n_pos(1)[0]
                if (pos1 == pos2 or (pos1 == robot.pos and pos2 == self.pos)):
                    self.collision_robots_id.append(robot.robot_id)
                    
        if (len(self.collision_robots_id) > 1):
            return True
        else:
            return False

    def collision_avoid(self, 
                move_matrix: List[List[Point]],
                target_pos: Point = Point(-1, -1)):
        # 表示是否规划成功
        okk = True
        # 如果存在会碰撞的机器人
        if (self.collision_check(move_matrix, target_pos)):
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
                    if (self.robots[id].extended_status != Robot_Extended_Status.CollisionAvoidance):
                        self.robots[id].enable_collision_avoidance(move_matrix, target_pos)
                    # 如果已经进入避障状态，是否保证path已经规划
                    # 如果是这一帧启动的，则必然已经规划过，但是不一定针对当前max_id进行的规划
                    # 如果是上一帧启动的，则其规划是针对上一帧的，需要重新计算
                    # 问题在于究竟针对谁进行避障，暂时先不考虑？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？
                    else:
                        pass
            # 另起一行，为的是在执行最高优先级之前，所有低优先级的路径已经规划好    
            for id in self.collision_robots_id:
                # 最高优先级的对象
                if (id == max_id):
                    # 如果正在避障，不可以直接解除避障状态，因为他可能正在必然当前不可见的更高优先级的robot
                    # 如果机器人不撞了，则认为可以解除避障
                    # collision check修复了避障时无法看见原来路径的错误！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
                    if (self.robots[id].extended_status == Robot_Extended_Status.CollisionAvoidance
                        and self.robots[id].collision_check(move_matrix, target_pos) is False
                        ):# and robots[id].paths_stk.empty()
                        self.robots[id].try_disable_collision_avoidance(move_matrix, target_pos)
                    # 该最高优先级的部分区域已经让路，如何考虑其他区域？
                    # 暂时不考虑？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？
                    else:
                        pass
        return okk
        # 如果没有会撞得，且当前为避障模式，则恢复原来模式 xxxxxxxxx错误，会导致循环
        # elif (self.extended_status == Robot_Extended_Status.CollisionAvoidance):
        #     self.disable_collision_avoidance(move_matrix, robots, berths, target_pos)

    def find_avoidance_path(self, 
                move_matrix: List[List[Point]],
                target_pos: Point = Point(-1, -1)) -> LifoQueue[Point]:
        avoidance_paths_stk = LifoQueue()

        # collision_check同时会更新self.surrounding, self.collision_robots_ud
        if (self.collision_check(move_matrix, target_pos) == False):
            return avoidance_paths_stk
        
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
                avoidance_matrix[col].append(self.value_matrix[y][x])
        # 将机器人的位置和其路径视为障碍物
        for robot_id in self.surronding_robots_with_priority:
            # 不考虑自己
            if robot_id == self.robot_id:
                continue
            robot = self.robots[robot_id]
            poses: List[Point] = []
            poses.append(Point(x = robot.x, y = robot.y))
            poses.append(robot.next_n_pos(1)[0])
            for pos in poses:
                avoidance_matrix[pos.y-t][pos.x-l] = 0

        # 尝试一条避障路径
        list_avoidance_paths, success = one_move_avoidance(avoidance_matrix,
                                                           Point(self.pos.x-l, self.pos.y-t))
        for item in list_avoidance_paths:
            item = Point(item.x + l, item.y + t)
            avoidance_paths_stk.put(item)
        
        return avoidance_paths_stk

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
    def enable_collision_avoidance(self, 
                move_matrix: List[List[Point]],
                target_pos: Point = Point(-1, -1)):
        self.original_extended_status = self.extended_status
        self.original_paths_stk = self.paths_stk
        self.paths_stk = LifoQueue()
        self.extended_status = Robot_Extended_Status.CollisionAvoidance
        self.path_planing(move_matrix, target_pos)
    
    # 退出避障
    # 将collision期间的paths附加到原来状态的paths上
    def try_disable_collision_avoidance(self, 
                move_matrix: List[List[Point]],
                target_pos: Point = Point(-1, -1)):
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

        self.extended_status = self.original_extended_status
        return True
        #self.run(move_matrix, robots, berths, target_pos)

    def path_planing(self, 
                move_matrix: List[List[Point]],
                target_pos: Point = Point(-2, -2)):  
            
            timer = My_Timer()
            
            # GotoFetchFromBerth只能由OnBerth状态转入，OnBerth时Paths为空
            if self.extended_status == Robot_Extended_Status.GotoFetchFromBerth:
                # 转入GotoFetchFromBerth时，paths必须为空，且位置在berth
                # 首先，如果从目标港口出发物品不可达（其实外部已保证），则停留在港口不动，此时满足转入港口的条件
                # 否则必须规划路径，为了避免重复初始化，考虑以第一次转入的paths为空为判断条件
                # 但当目标点已经是当前位置，或者已经走到目标点时，paths会变成空
                # 路径规划算法在paths为空时会导致生成一个冗余的next_pos...后果未知，避免为好
                if (move_matrix[target_pos.y][target_pos.x] == UNREACHABLE):
                    self.extended_status = Robot_Extended_Status.OnBerth
                    return False # 中途改变状态，重新规划路径
                elif (self.paths_stk.empty()
                    and (self.berths[self.berth_id].pos != target_pos)): 
                    cur_pos = target_pos
                    self.paths_stk.put(cur_pos)
                    cur_pos = cur_pos + move_matrix[cur_pos.y][cur_pos.x]
                    while (cur_pos != self.berths[self.berth_id].pos):
                        self.paths_stk.put(cur_pos)
                        # logger.info("debug %s | %s", cur_pos, berths[berth_id].pos)
                        cur_pos = cur_pos + move_matrix[cur_pos.y][cur_pos.x]

            elif self.extended_status == Robot_Extended_Status.BackBerthAndPull:
                # 若无法返回港口的，则转入状态UnableBackBerth
                if move_matrix[self.pos.y][self.pos.x] == UNREACHABLE:
                    self.extended_status = Robot_Extended_Status.UnableBackBerth
                    return False # 中途改变状态，重新计算路径
                # 转入BackBerthAndPull时总是保证 paths为空；
                # 实际上，每一帧都可以为重新计算路径，但为了节省计算，仅在第一次进入时计算paths，“此时paths=空可以为判断条件”
                # 值得注意的是，若转入该状态时已经到达，此时计算路径会导致生成一个冗余的原地pos，“排除这种情况”
                # 若因为某些原因，paths全部走完为空时没有转换到其他状态，再次进入这个状态会保证仍然正确
                elif self.paths_stk.empty() and (self.pos != self.berths[self.berth_id].pos):
                        tmp_stk = LifoQueue()
                        cur_pos = self.pos
                        cur_pos = cur_pos + move_matrix[cur_pos.y][cur_pos.x]
                        while (cur_pos != self.berths[self.berth_id].pos): # 此时self.pos就是在berth上
                            tmp_stk.put(cur_pos)
                            #logger.info("debug %s | %s", cur_pos, berths[berth_id].pos)
                            cur_pos = cur_pos + move_matrix[cur_pos.y][cur_pos.x]
                        tmp_stk.put(cur_pos)
                        
                        while not tmp_stk.empty():
                            self.paths_stk.put(tmp_stk.get())

            elif self.extended_status == Robot_Extended_Status.CollisionAvoidance:
                
                tmp_paths_stk = self.find_avoidance_path(move_matrix, target_pos)
                # 尝试能够回溯的避障路径
                self.gen_recoverable_paths(tmp_paths_stk)

            elif self.extended_status in [Robot_Extended_Status.OnBerth, Robot_Extended_Status.UnableBackBerth]:
                self.paths_stk = LifoQueue()
            
            return True
            #logger.info("DFS cost: %s", timer.click()*1000)

    def paths_execution(self,zhen):
        # # 有用吗？是否可以删除
        # # 如果下一帧就是原地动作，是否还有存在的必要
        # if self.status == 0:
        #     self.suppose_pos = self.pos
        #     self.empty_paths = self.paths_stk.empty()
        #     return
        
        if not self.paths_stk.empty():
            self.empty_paths = False
            next_pos = self.paths_stk.get(False)
            action = next_pos - self.pos
            if (action not in robot_action_value_to_cmd):
                error_logger.error("帧数：%s，cur_pos: %s, 目标位置: %s", zhen, self.pos, next_pos)
                # self.extended_status = Robot_Extended_Status.BackBerthAndPull
                return
        else:
            self.empty_paths = True
            next_pos = self.pos
            action = Robot_Actions.HOLD
        
        self.last_pos = self.pos
        self.suppose_pos = next_pos
        
        if (action != Robot_Actions.HOLD):
            print("move", self.robot_id, robot_action_value_to_cmd[action])

    def run(self, 
                move_matrix: List[List[Point]],
                target_pos: Point = Point(-1, -1)):
        # robot根据状态执行每一帧的动作
        while(1):
            if self.path_planing(move_matrix, target_pos) is False:
                continue
            if self.collision_avoid(move_matrix, target_pos) is False:
                continue
            else:
                break

    # 根据状态的执行结果改变状态，区别于run中的状态变化（如何区别？）
    def update_extended_status(self, 
                move_matrix: List[List[Point]],
                target_pos: Point = Point(-1, -1)):
        '''在规划、执行后进行，在下一帧进行更新；
            当前帧的状态更新放在下一帧的最开始，
            是因为机器人的状态，下一帧才能被更新'''

        if self.status == 0 or self.last_status == 0:# 若当前帧刚解除，但上一帧是眩晕状态，也需要恢复
        #if self.status == 0: 
            self.last_status = self.status

            if self.pos != self.suppose_pos:
                self.paths_stk.put(self.suppose_pos)
            elif self.empty_paths == False:
                self.paths_stk.put(self.suppose_pos)

            if (self.extended_status != Robot_Extended_Status.CollisionAvoidance#):
                and self.collision_check(move_matrix, target_pos) == True):
                self.enable_collision_avoidance(move_matrix, target_pos)
        
        elif self.extended_status == Robot_Extended_Status.GotoFetchFromBerth:
            # 转入GotoFetchFromBerth时paths必须为空，为保证切换条件正确，
            # 必须保证在update之前 经过规划操作，否则可能误以为到达
            # 实际上，每次update都放在了第二帧
            if (self.paths_stk.empty()):
                # 可能取不到货，货已经消失
                #？？？
                #？？？
                #？？？
                print("get", self.robot_id)
                self.extended_status = Robot_Extended_Status.GotGoods
        
        elif self.extended_status == Robot_Extended_Status.BackBerthAndPull:
            # 转入BackBerthAndPull时paths必须为空，为保证切换条件正确，
            # 必须保证在update之前 经过规划操作，否则可能误以为到达
            # 实际上，每次update都放在了第二帧
            if (self.paths_stk.empty()):
                self.extended_status = Robot_Extended_Status.OnBerth
                if self.goods == 1:
                    print("pull", self.robot_id)
                    self.berths[self.berth_id].num_gds += 1
                    self.berths[self.berth_id].total_earn += 180
        
        # 如果发生碰撞，则不会移动，所以不需要担心避障时碰撞导致无法记录路径
        elif self.extended_status == Robot_Extended_Status.CollisionAvoidance:
            # # 记录移动的位置
            # if self.pos != self.last_pos:
            #     self.walked_paths_during_collision_avoidance.put(self.pos)
            # 每一帧开始时，检测是否可能还会碰撞
            # 考虑避障期间会移动，若paths非空，则表明未归位，是否可以回归状态并将这些位置添加到原状态的paths上
            if (#self.paths_stk.empty() # 没有需要避障的道路and 
                self.collision_check(move_matrix, target_pos) == False):
                    self.try_disable_collision_avoidance(move_matrix, target_pos)
        
        elif self.extended_status == Robot_Extended_Status.OnBerth:
            pass

        elif self.extended_status == Robot_Extended_Status.Uninitialized:
            pass