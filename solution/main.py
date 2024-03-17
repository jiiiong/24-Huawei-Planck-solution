# import debugpy
# debugpy.listen(5678)
# print("Waiting for debugger attach")
# debugpy.wait_for_client()
# debugpy.breakpoint()

import sys
from queue import Queue, PriorityQueue
import random
import time
from typing import List, Tuple, Dict, Set

from log import logger, error_logger
from core import enum_stk_and_recover
from core import  Robot, Berth, Boat
from core import Robot_Extended_Status
from path_planing import Point, UNREACHABLE
from path_planing import BFS
from path_planing import chMap2ValueMatrix
from path_planing import Mission


n = 200
robot_num = 10
berth_num = 10
N = 210

# robot = [Robot() for _ in range(robot_num + 10)]
# berth = [Berth() for _ in range(berth_num + 10)] 为什么要+10
robots: List[Robot] = [Robot() for _ in range(robot_num)]
berths: List[Berth] = [Berth() for _ in range(berth_num)]
boats:  List[Boat]  = [Boat() for _ in range(10)]

money = 0
boat_capacity = 0
id = 0
ch: List[List[str]] = []    #每行只有一个元素
gds = [[0 for _ in range(N)] for _ in range(N)]


# for path_planing 
value_matrix:   List[List[int]]   = [[] for _ in range(10)] # 用来表示每个位置的开销，0代表不可通行
cost_matrix_list:  List[ (List[List[int]]) ]   = []
move_matrix_list:  List[ (List[List[Point]]) ] = []

# global queue for goods
# List[PriorityQueue[Tuple[int, Point]]]
berth_gds_priority_queue_list: List[PriorityQueue] = [PriorityQueue() for _ in range(berth_num)]

check_num = [0]
back_count = 71

def Init():
    for _ in range(0, n):
        line = input()
        ch.append([c for c in line.split(sep=" ")])
    for _ in range(berth_num):
        line = input()
        berth_list = [int(c) for c in line.split(sep=" ")]
        id = berth_list[0]
        # 以y为行，x为列
        berths[id].y = berth_list[1] + 2
        berths[id].x = berth_list[2] + 2
        berths[id].transport_time = berth_list[3]
        berths[id].loading_speed = berth_list[4]
        # logger.info("transport time: %d, loading speed: %d,",berths[id].transport_time, berths[id].loading_speed)
    global boat_capacity
    boat_capacity = int(input())
    # logger.info("boat capacity: %s", boat_capacity)
    okk = input()

    # 初始化所有港口的BFS
    myInit()

    print("OK")
    sys.stdout.flush()

def Input():
    id, money = map(int, input().split(" "))
    #logger.info("id: %s", id)
    num = int(input())
    for i in range(num):
        y, x, val = map(int, input().split())
        gds[y][x] = val
        #logger.info("%d, %d, %d", y, x, val)

        # 暂时测试物品队列用
        for i in range(robot_num):
            if (cost_matrix_list[i][y][x] >= 0 and val > 100):
                berth_gds_priority_queue_list[i].put((cost_matrix_list[i][y][x], Point(x, y)))
        
    for i in range(robot_num):
        robots[i].goods, robots[i].y, robots[i].x, robots[i].status = map(int, input().split())

    for i in range(5):
        boats[i].status, boats[i].pos = map(int, input().split())
    okk = input()
    return id

def myInit():
    t = time.time()
    value_matrix = chMap2ValueMatrix(ch)
    for b in berths:
        move_matrix, cost_matrix = BFS(value_matrix, b.pos)
        move_matrix_list.append(move_matrix)
        cost_matrix_list.append(cost_matrix)
        # from path_planing.utils import applyMoveMatrix2ChMap, saveMatrix2File
        # saveMatrix2File(applyMoveMatrix2ChMap(ch, move_matrix))
    t = time.time() - t
    # logger.info("myInit time: %ds", t)

class Scheduler:
    def __init__(self) -> None:
        self.target_pos_list: List[Point] = [Point(-1, -1) for _ in range(robot_num)]
        # hui 新增一个用来去除重复任务的set
        self.target_pos_in_mission: Set[Mission] =set()

    def init_robots(self, robots: List[Robot], berths: List[Berth]):
        for i, robot in enumerate(robots):
            robot.robot_id = i
            robot.berth_id = i
            robot.berths = berths
            robot.move_matrix_list = move_matrix_list
            robot.suppose_pos = robot.pos
            robot.last_pos = robot.pos
            # robot[robot_id].cal_alarming_area(robot[robot_id].alarming_area_size)
            # logger.info("%s", robot[robot_id].alarming_area)
   
    def go_to_fetch_from_berth(self, robot_id: int):
        if berth_gds_priority_queue_list[robot_id].empty() is False:
            target_pos: Point = berth_gds_priority_queue_list[robot_id].get(False)[1]
            # # hui 通过这个target_pos_in_mission来去除重复任务
            # while target_pos in self.target_pos_in_mission:
            #     target_pos = berth_gds_priority_queue_list[robot_id].get()[1]
            # mission_instance = Mission(target_pos, robot_id, robot_id)
            # self.target_pos_in_mission.add(mission_instance)
            # logger.info("添加进mission集合的任务 :%s\n",mission_instance)

            global robots
            # 避免分配当前港口拿不到的物品
            if move_matrix_list[robots[robot_id].berth_id][target_pos.y][target_pos.x] != UNREACHABLE:
                # logger.info("target pos is %s", target_pos)
                robots[robot_id].extended_status = Robot_Extended_Status.GotoFetchFromBerth
                self.target_pos_list[robot_id] = target_pos

    def back_berth_and_pull(self, robot_id: int):
        robots[robot_id].extended_status = Robot_Extended_Status.BackBerthAndPull

def visualize_next_n_move(start_pos: Point, next_n_move: List[Point]):
        from path_planing.utils import applyNextnMove2ChMap, saveMatrix2File
        saveMatrix2File(applyNextnMove2ChMap(ch, start_pos, next_n_move))

if __name__ == "__main__":

    Init()
    # logger.info("----------------init----------------------")
    
    scheduler = Scheduler()

    for zhen in range(1, 15001):
        id = Input()
        # 在第一帧开始前初始化小车的信息（因为小车的坐标和id在第一帧输入后才能确认）
        # 可以集成到myinit中。先不管
        if (zhen == 1):
            scheduler.init_robots(robots, berths)

        # 🚗🚗🚗🚗🚗🚗🚗🚗🚗🚗🚗🚗🚗🚗🚗🚗🚗🚗🚗🚗🚗🚗🚗🚗🚗🚗🚗🚗🚗🚗🚗🚗🚗🚗

        #  🐞
        # logger.info("Z: %s, %s, %s",zhen, robots[3].pos, robots[3].extended_status)
        # for stk in [robots[3].paths_stk, robots[3].original_paths_stk]:
        #     poses = []
        #     for item in enum_stk_and_recover(stk):
        #         poses.append(item)
        #     logger.info("%s", poses)
        for i in range(robot_num):
            # ## 🐞
            # if i in check_num:
            #     logger.info("robot status %s", robots[i].status)
            # 🐞之前将self.pos引用传入栈导致出错
            # if i in check_num:
            #     logger.info("A: %s, %s, %s",zhen, robots[i].pos, robots[i].extended_status)
            #     for stk in [robots[i].paths_stk, robots[i].original_paths_stk]:
            #         poses = []
            #         for item in enum_stk_and_recover(stk):
            #             poses.append(item)
            #         logger.info("%s", poses)
            robots[i].update_extended_status(move_matrix_list[i], robots, berths, scheduler.target_pos_list[i])
            ## 🐞 用，将运行路线打印出来
            # if i in check_num:
            #     logger.info("B: %s, %s, %s",zhen, robots[i].pos, robots[i].extended_status)
            #     for stk in [robots[i].paths_stk, robots[i].original_paths_stk]:
            #         poses = []
            #         for item in enum_stk_and_recover(stk):
            #             poses.append(item)
            #         logger.info("%s", poses)
        
        for i in range(robot_num):
            # 碰撞了的化？？？？？？？？    
            if robots[i].extended_status == Robot_Extended_Status.Uninitialized:
                # 该转换符合robot状态机规则，paths此时为空
                scheduler.back_berth_and_pull(i)
            elif robots[i].extended_status == Robot_Extended_Status.OnBerth:
                # 若OnBerth满足性质，则该状态转换正确
                scheduler.go_to_fetch_from_berth(i)
            elif (robots[i].extended_status == Robot_Extended_Status.GotGoods):
                # 符合规则
                # if (robots[i].goods == 1):
                # mission_instance = Mission(scheduler.target_pos_list[i] ,robots[i].robot_id, robots[i].robot_id)
                # if mission_instance in scheduler.target_pos_in_mission:
                #     scheduler.target_pos_in_mission.remove(mission_instance)
                scheduler.back_berth_and_pull(i)
            robots[i].run(move_matrix_list[i], robots, berths, scheduler.target_pos_list[i])
        
        for i in range(robot_num):
            # # 🐞
            # if i in check_num:
            #     logger.info("C: %s, %s, %s",zhen, robots[i].pos, robots[i].extended_status)
            #     for stk in [robots[i].paths_stk, robots[i].original_paths_stk]:
            #         poses = []
            #         for item in enum_stk_and_recover(stk):
            #             poses.append(item)
            #         logger.info("%s\n", poses)
            robots[i].paths_execution(zhen)
            ## 🐞
            # if i in check_num:
            #     logger.info("D: %s, %s, %s",zhen, robots[i].pos, robots[i].extended_status)
            #     for stk in [robots[i].paths_stk, robots[i].original_paths_stk]:
            #         poses = []
            #         for item in enum_stk_and_recover(stk):
            #             poses.append(item)
            #         logger.info("%s", poses)

        # # 🐞
        # logger.info("E: %s, %s, %s",zhen, robots[3].pos, robots[3].extended_status)
        # for stk in [robots[3].paths_stk, robots[3].original_paths_stk]:
        #     poses = []
        #     for item in enum_stk_and_recover(stk):
        #         poses.append(item)
        #     logger.info("%s", poses)
        # logger.info("\n")

        # boats shceduling
        # 🚢🚢🚢🚢🚢🚢🚢🚢🚢🚢🚢🚢🚢🚢🚢🚢🚢🚢🚢🚢🚢🚢🚢🚢🚢🚢🚢🚢🚢
        for i in range(5):
            endone = False
            if (boats[i].pos == -1 and boats[i].status == 1 or zhen == 1):
                print("ship", i, i)
            elif (boats[i].pos == i and boats[i].status == 1):
                back_count = back_count - 1
                if (back_count == 0 or (zhen > 13000 and not endone)):
                    if (zhen>13000):
                        endone = True
                    print("go", i)
                    back_count = boat_capacity

        print("OK")
        sys.stdout.flush()
