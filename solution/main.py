# import debugpy
# debugpy.listen(5678)
# print("Waiting for debugger attach")
# debugpy.wait_for_client()
# debugpy.breakpoint()

import sys
from queue import Queue, PriorityQueue, LifoQueue
import random
import time
from typing import List, Tuple, Dict, Set

from log import logger, error_logger
from core import Env
from core import enum_stk_and_recover
from core import  Robot, Berth, Boat, Goods
from core import Robot_Extended_Status
from path_planing import Point, UNREACHABLE
from path_planing import BFS, BFS_divide
from path_planing import chMap2ValueMatrix
from scheduler import Scheduler


def Init(env: Env):
    for _ in range(0, env.n):
        line = input()
        env.ch.append([c for c in line.split(sep=" ")])
    for _ in range(env.berth_num):
        line = input()
        berth_list = [int(c) for c in line.split(sep=" ")]
        id = berth_list[0]
        # 以y为行，x为列
        env.berths[id].y = berth_list[1] + 1
        env.berths[id].x = berth_list[2] + 1
        env.berths[id].transport_time = berth_list[3]
        env.berths[id].loading_speed = berth_list[4]
        # error_logger.error("boat_id: %s, transport time: %d, loading speed: %d", id, env.berths[id].transport_time, env.berths[id].loading_speed)
    boat_capacity = int(input())
    env.boat_capacity = boat_capacity
    #error_logger.error("boat capacity: %s", boat_capacity)
    okk = input()

    # 初始化所有港口的BFS
    myInit(env)

    print("OK")
    sys.stdout.flush()

def Input(scheduler: Scheduler):
    id, money = map(int, input().split(" "))
    env.global_zhen = id
    # logger.info("%s  %s", money, 0)
    num = int(input())
    #logger.info("%d",num)
    for i in range(num):
        y, x, val = map(int, input().split())
        env.gds[y][x] = val
        env.total_map_gds_value += val

        # 调度物品
        scheduler.schedule_gds(Goods(gen_zhen=id, global_zhen_ref=env.global_zhen_ref, pos=Point(x,y), price=val))

    for i in range(env.robot_num):
        env.robots[i].goods, env.robots[i].y, env.robots[i].x, env.robots[i].status = map(int, input().split())

    for i in range(5):
        env.boats[i].status, env.boats[i].pos = map(int, input().split())
    okk = input()
    return id, money

def myInit(env: Env):
    env.value_matrix = chMap2ValueMatrix(env.ch)
    #env.divide_matrix = BFS_divide(env.value_matrix, [berth.pos for berth in env.berths])
    for b in env.berths:
        move_matrix, cost_matrix = BFS(env.value_matrix, b.pos)
        env.move_matrix_list.append(move_matrix)
        env.cost_matrix_list.append(cost_matrix)
        
def berths_zhen_handler():
    if (env.global_zhen % 100 == 0):
        for berth in env.berths:
            berth.clear_queue()

def robots_zhen_handler():
    robot_num = env.robot_num
    robots = env.robots
    move_matrix_list = env.move_matrix_list

    # 下一帧更新上一帧结束后robots状态的转换
    for i in range(robot_num):
        # robot所有的操作基于robot.pos的位置是正确的
        robots[i].update_extended_status()
        if i == 7:
            robots[i].debug_robot()    
    # 调度器调度robots
    scheduler.schedule_robots()

    # 避障
    for i in range(robot_num):
        robots[i].collision_avoid()
    
    for i in range(robot_num):
        robots[i].paths_execution()

def boats_zhen_handler():
    scheduler.schedule_boats()


# 定义全局变量
check_num = []
# check_num = [5]

record_robot_list = []


init_robot_list = []

if __name__ == "__main__":

    # 初始化环境变量
    env = Env()
        # 为了解决循环依赖问题
    robots:List[Robot] = [Robot() for _ in range(env.robot_num)]
    berths:List[Berth] = [Berth() for _ in range(env.berth_num)]
    boats:List[Boat]   = [Boat() for _ in range(5)]
    env.init_env(robots, berths, boats)
        # 使用第一次输出初始化各种地图
    Init(env)

    # 初始化调度器
    scheduler = Scheduler(env)
    # 初始化港口，必须在input之前
    scheduler.init_berths()

    for zhen in range(1, 15001):
        # 更新环境变量中的全局时间
        # 获取输出，并调度物品
        id, money = Input(scheduler)
        zhen = id

        earn = []
        for berth in env.berths:
            # earn.append(berth.earn_when_n[0])
            earn.append(berth.cur_num_gds)
        logger.info("%s %s", zhen, " ".join([str(item) for item in earn]))
        

        if (zhen == 1):
            scheduler.init_robots()
            scheduler.init_boats()
            
        if zhen < 20 and zhen % 2 == 1:
            robot_on_run_id = -1
            shortest_length = 9999
            for robot in env.robots:
                if robot.robot_id not in record_robot_list:
                    if shortest_length > env.cost_matrix_list[robot.berth_id][robot.y][robot.x]:
                        shortest_length = env.cost_matrix_list[robot.berth_id][robot.y][robot.x]
                        robot_on_run_id = robot.robot_id
            record_robot_list.append(robot_on_run_id)
            check_num.append(robot_on_run_id)
            logger.info("append %d with dist %d", robot_on_run_id, shortest_length)

        berths_zhen_handler()

        robots_zhen_handler()

        boats_zhen_handler()

        if (zhen == 14999):
           error_logger.error("全局货物价值：%s", env.total_map_gds_value)
           error_logger.error("港口堆积价值：" + str(sum([berth.total_earn for berth in env.berths])) + 
                              " python -c \"print(" + "+".join([str(berth.total_earn) for berth in env.berths]) + ")\"")
           error_logger.error("收获比例：%s", money / sum([berth.total_earn for berth in env.berths]))
        
        print("OK")
        sys.stdout.flush()
