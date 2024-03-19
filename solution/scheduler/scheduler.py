from typing import List, Tuple, Set
from queue import LifoQueue, Queue

from log import logger, error_logger
from core import Env
from core import Robot, Robot_Extended_Status
from core import Berth, Goods
from path_planing import Point, UNREACHABLE

class Scheduler:
    def __init__(self, env: Env) -> None:
        self.env = env
        self.target_pos_list: List[Point] = [Point(-1, -1) for _ in range(self.env.robot_num)]    

    def init_robots(self):
        for i, robot in enumerate(self.env.robots):
            robot.robot_id = i
            robot.berth_id = i
            robot.env = self.env
            robot.suppose_pos = robot.pos
            robot.last_pos = robot.pos
    
    def init_berths(self):
        for berth_id, berth in enumerate(self.env.berths):
            berth.berth_id = berth_id
            berth.env = self.env

    def go_to_fetch_from_berth(self, cur_robot_id: int):
        global robots
        robots = self.env.robots
        berths = self.env.berths
        cur_berth_id = robots[cur_robot_id].berth_id
        cur_berth  = berths[cur_berth_id]
        success, goods = cur_berth.fetch_goods()
            # # hui 通过这个target_pos_in_mission来去除重复任务
            # while target_pos in self.target_pos_in_mission:
            #     target_pos = berth.gds_priority_queue.get()[1]
            # mission_instance = Mission(target_pos, robot_id, robot_id)
            # self.target_pos_in_mission.add(mission_instance)
            #logger.info("添加进mission集合的任务 :%s\n",mission_instance)

            # 避免分配当前港口拿不到的物品
        if success:
            if self.env.move_matrix_list[cur_berth_id][goods.y][goods.x] != UNREACHABLE:
                # logger.info("target pos is %s", target_pos)
                robots[cur_robot_id].extended_status = Robot_Extended_Status.GotoFetchFromBerth
                self.target_pos_list[cur_robot_id] = goods.pos
                goods.fetched = True

    def back_berth_and_pull(self, robot_id: int):
        self.env.robots[robot_id].extended_status = Robot_Extended_Status.BackBerthAndPull

    def schedule_gds(self, goods: Goods):
        if goods.price < 180:
            return
        cost_matrix_list = self.env.cost_matrix_list
        bid_cost_list: List[Tuple[int, float]] = []
        for berth_id in range(self.env.berth_num):
            cost = cost_matrix_list[berth_id][goods.y][goods.x]
            bid_cost_list.append((berth_id, cost))
            # berth = berths[berth_id]
            # cost = goods.price / (cost_matrix_list[berth_id][goods.pos.y][goods.pos.x] + berth.loading_speed + berth.transport_time)
            # id_cost_list.append((berth_id, -cost))
        # 将货物放入当前最近的3个港口中
        bid_cost_list.sort(key=lambda x: x[1])
        for order in range(2):
            berth_id = bid_cost_list[order][0]
            self.env.berths[berth_id].add_goods(goods)

    def schedule_robots(self):
        robots = self.env.robots
        berths = self.env.berths
        move_matrix_list = self.env.move_matrix_list
        arrive_rate_list = [(i, berth.get_estimated_rate()) for i, berth in enumerate(berths)]
        arrive_rate_list.sort(key = lambda x : x[1], reverse=True)
        logger.info("from %s to %s", robots[arrive_rate_list[9][0]].berth_id, arrive_rate_list[0][0])
        robots[arrive_rate_list[9][0]].berth_id = arrive_rate_list[0][0]
        robots[arrive_rate_list[9][0]].extended_status = Robot_Extended_Status.BackBerthAndPull
        robots[arrive_rate_list[9][0]].paths_stk = LifoQueue()
        robots[arrive_rate_list[9][0]].path_planing(move_matrix_list[robots[arrive_rate_list[9][0]].berth_id])

        logger.info("from %s to %s", robots[arrive_rate_list[8][0]].berth_id, arrive_rate_list[1][0])
        robots[arrive_rate_list[8][0]].berth_id = arrive_rate_list[1][0]
        robots[arrive_rate_list[8][0]].extended_status = Robot_Extended_Status.BackBerthAndPull
        robots[arrive_rate_list[8][0]].paths_stk = LifoQueue()
        robots[arrive_rate_list[8][0]].path_planing(move_matrix_list[robots[arrive_rate_list[1][0]].berth_id])

        logger.info("from %s to %s", robots[arrive_rate_list[7][0]].berth_id, arrive_rate_list[2][0])
        robots[arrive_rate_list[7][0]].berth_id = arrive_rate_list[2][0]
        robots[arrive_rate_list[7][0]].extended_status = Robot_Extended_Status.BackBerthAndPull
        robots[arrive_rate_list[7][0]].paths_stk = LifoQueue()
        robots[arrive_rate_list[7][0]].path_planing(move_matrix_list[robots[arrive_rate_list[2][0]].berth_id])