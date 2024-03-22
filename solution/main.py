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



# hui
# 找到一个当前堆积货物最多的空闲的港口
        
# def visualize_next_n_move(start_pos: Point, next_n_move: List[Point]):
#         from path_planing.utils import applyNextnMove2ChMap, saveMatrix2File
#         saveMatrix2File(applyNextnMove2ChMap(ch, start_pos, next_n_move))

# hui
# 找到一个当前堆积货物最多的空闲的港口
def getIdealBerthId(berths:List[Berth] ,boats:List[Boat]):
    boatsWorkingBerthList: List[int] = []
    gdsOfBerth: List[int] = []
    for i in range(5):
        boatsWorkingBerthList.append(boats[i].pos)
    # logger.info("boatsWorkingBerthList:%s",boatsWorkingBerthList)
    for i in range(10):
        gdsOfBerth.append(berths[i].cur_num_gds)
    # logger.info("gdsOfBerth:%s",gdsOfBerth)
    sorted_berths_with_gds = sorted(enumerate(gdsOfBerth, start=0), key=lambda x: x[1], reverse=False)
    # 使用列表来实现栈，将排序后的港口ID放入栈中
    stack = [berth_id for berth_id, _ in sorted_berths_with_gds]
    # logger.info("stack:%s",stack)
    IdealBerthId = -1
    for i in range(10):
        berthid = stack.pop()
        if berthid not in boatsWorkingBerthList:
            IdealBerthId =  berthid
            return IdealBerthId 
    return -1

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
        error_logger.error("boat_id: %s, transport time: %d, loading speed: %d", id, env.berths[id].transport_time, env.berths[id].loading_speed)
    boat_capacity = int(input())
    env.boat_capacity = boat_capacity
    error_logger.error("boat capacity: %s", boat_capacity)
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
        # logger.info("%d, %d, %d", y, x, val)
        # 暂时测试物品队列用
        scheduler.schedule_gds(Goods(gen_zhen=id, global_zhen_ref=env.global_zhen_ref, pos=Point(x,y), price=val))
        # logger.info(" ".join([str(berth.gds_priority_queue.qsize()) for berth in berths]))


    for i in range(env.robot_num):
        env.robots[i].goods, env.robots[i].y, env.robots[i].x, env.robots[i].status = map(int, input().split())

    for i in range(5):
        env.boats[i].status, env.boats[i].pos = map(int, input().split())
    okk = input()
    return id, money

def myInit(env: Env):
    t = time.time()
    env.value_matrix = chMap2ValueMatrix(env.ch)
    #env.divide_matrix = BFS_divide(env.value_matrix, [berth.pos for berth in env.berths])
    for b in env.berths:
        move_matrix, cost_matrix = BFS(env.value_matrix, b.pos)
        env.move_matrix_list.append(move_matrix)
        env.cost_matrix_list.append(cost_matrix)
        # from path_planing.utils import applyMoveMatrix2ChMap, saveMatrix2File
        # saveMatrix2File(applyMoveMatrix2ChMap(ch, move_matrix))
    t = time.time() - t
    # logger.info("myInit time: %ds", t)

def berths_zhen_handler():
    if (env.global_zhen % 100 == 0):
        for berth in env.berths:
            berth.clear_queue()

def robots_zhen_handler():
    robot_num = env.robot_num
    robots = env.robots
    move_matrix_list = env.move_matrix_list

    for i in range(robot_num):
        # ## 🐞
        # if i in check_num:
        #     logger.info("robot status %s", robots[i].status)
        # #🐞之前将self.pos引用传入栈导致出错
        # if i in check_num:
        #     logger.info("A: %s, %s, %s",zhen, robots[i].pos, robots[i].extended_status)
        #     for stk in [robots[i].paths_stk, robots[i].original_paths_stk]:
        #         poses = []
        #         for item in enum_stk_and_recover(stk):
        #             poses.append(item)
        #         logger.info("%s", poses)
        if not i in check_num:
            continue
        robots[i].update_extended_status(move_matrix_list[i], scheduler.target_pos_list[i])
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
        if not i in check_num:
            continue
        if robots[i].extended_status == Robot_Extended_Status.Uninitialized:
            # 该转换符合robot状态机规则，paths此时为空
            scheduler.back_berth_and_pull(i)
        elif robots[i].extended_status == Robot_Extended_Status.OnBerth:
            # 若OnBerth满足性质，则该状态转换正确
            scheduler.go_to_fetch_from_berth(i)
        elif (robots[i].extended_status == Robot_Extended_Status.GotGoods):
            # 符合规则
            scheduler.back_berth_and_pull(i)
            
    for i in range(robot_num):
        if not i in check_num:
            continue
        robots[i].run(scheduler.target_pos_list[i])
    
    for i in range(robot_num):
        if not i in check_num:
            continue
        # # 🐞
        # if i in check_num:
        #     logger.info("C: %s, %s, %s",zhen, robots[i].pos, robots[i].extended_status)
        #     for stk in [robots[i].paths_stk, robots[i].original_paths_stk]:
        #         poses = []
        #         for item in enum_stk_and_recover(stk):
        #             poses.append(item)
        #         logger.info("%s", poses)
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

def boats_zhen_handler():
    scheduler.schedule_boats_5()


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
