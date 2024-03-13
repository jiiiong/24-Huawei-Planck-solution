# import debugpy
# debugpy.listen(5678)
# print("Waiting for debugger attach")
# debugpy.wait_for_client()
# debugpy.breakpoint()

import sys
from queue import Queue, PriorityQueue
import random
import time
from typing import List, Tuple

from log import logger
from core import  Robot, Berth, Boat
from core import Robot_Extended_Status
from path_planing import Point
from path_planing import BFS
from path_planing import chMap2ValueMatrix


n = 200
robot_num = 10
berth_num = 10
N = 210

# robot = [Robot() for _ in range(robot_num + 10)]
# berth = [Berth() for _ in range(berth_num + 10)] 为什么要+10
robot: List[Robot] = [Robot() for _ in range(robot_num)]
berth: List[Berth] = [Berth() for _ in range(berth_num)]
boat:  List[Boat]  = [Boat() for _ in range(10)]

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
berth_gds_priority_queue_list = [PriorityQueue() for _ in range(berth_num)]

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
        berth[id].y = berth_list[1] + 2
        berth[id].x = berth_list[2] + 2
        berth[id].transport_time = berth_list[3]
        berth[id].loading_speed = berth_list[4]
        logger.info("transport time: %d, loading speed: %d,",berth[id].transport_time, berth[id].loading_speed)
    global boat_capacity
    boat_capacity = int(input())
    logger.info("boat capacity: %s", boat_capacity)
    okk = input()

    # 初始化所有港口的BFS
    myInit()

    print("OK")
    sys.stdout.flush()

def Input():
    id, money = map(int, input().split(" "))
    logger.info("id: %s", id)
    num = int(input())
    for i in range(num):
        y, x, val = map(int, input().split())
        gds[y][x] = val
        #logger.info("%d, %d, %d", y, x, val)

        # 暂时测试物品队列用
        for i in range(1):
            if (cost_matrix_list[i][y][x] >= 0 and val > 100):
                berth_gds_priority_queue_list[i].put( (cost_matrix_list[i][y][x], Point(x, y)))
        
    for i in range(robot_num):
        robot[i].goods, robot[i].y, robot[i].x, robot[i].status = map(int, input().split())
    for i in range(5):
        boat[i].status, boat[i].pos = map(int, input().split())
    okk = input()
    return id

def myInit():
    t = time.time()
    value_matrix = chMap2ValueMatrix(ch)
    for b in berth:
        move_matrix, cost_matrix = BFS(value_matrix, b.pos)
        move_matrix_list.append(move_matrix)
        cost_matrix_list.append(cost_matrix)
        # from path_planing.utils import applyMoveMatrix2ChMap, saveMatrix2File
        # saveMatrix2File(applyMoveMatrix2ChMap(ch, move_matrix))
    t = time.time() - t
    logger.info("myInit time: %ds", t)

class Scheduler:
    def __init__(self) -> None:
        self.target_pos_list = [Point(-1, -1) for _ in range(robot_num)]

    def init_scheduler(self, robot_id: int):
        robot[robot_id].robot_id = robot_id
        robot[robot_id].berth_id = robot_id
        self.back_berth_and_pull(robot_id)
    
    def go_to_fetch_from_berth(self, robot_id: int):
        
        if berth_gds_priority_queue_list[robot_id].empty() is False:
            target_pos = berth_gds_priority_queue_list[robot_id].get()[1]
            logger.info("target pos is %s", target_pos)
            robot[robot_id].extended_status = Robot_Extended_Status.GotoFetchFromBerth
            self.target_pos_list[robot_id] = target_pos
    
    # def back_berth(self, robot_id: int):
    #     if robot[robot_id].pos != berth[robot_id].pos:
    #         robot[robot_id].extended_status = Robot_Extended_Status.BackBerth

    def back_berth_and_pull(self, robot_id: int):
        if robot[robot_id].pos != berth[robot_id].pos:
            robot[robot_id].extended_status = Robot_Extended_Status.BackBerthAndPull

    
if __name__ == "__main__":

    Init()
    logger.info("----------------init----------------------")
    
    scheduler = Scheduler()

    for zhen in range(1, 15001):
        id = Input()
        for i in range(robot_num):
            if (zhen == 1):
                scheduler.init_scheduler(i)
            if robot[0].extended_status == Robot_Extended_Status.OnBerth:
                scheduler.go_to_fetch_from_berth(0)
            elif (robot[0].extended_status == Robot_Extended_Status.GotGoods):
                scheduler.back_berth_and_pull(0)
            # move_matrix: List[List[Point]],
            # robots = [],
            # berths: List[Berth] = [], 
            # target_pos: Point = Point(-1, -1)
            robot[i].run_new(move_matrix=move_matrix_list[i],
                             robots = robot,
                             berths = berth,
                             target_pos = scheduler.target_pos_list[i])

        if (boat[0].pos == -1 and boat[0].status == 1 or zhen == 1):
            print("ship", 0, 0)
        elif (boat[0].pos == 0 and boat[0].status == 1):
            back_count = back_count - 1
            if (back_count == 0):
                print("go", 0)
                back_count = boat_capacity

        print("OK")
        sys.stdout.flush()
