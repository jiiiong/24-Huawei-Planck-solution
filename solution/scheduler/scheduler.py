from typing import List, Tuple, Set
from queue import LifoQueue, Queue, PriorityQueue

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
        # berths = self.env.berths
        # robots = self.env.robots
                
        # # 分配robot到港口
        # for i, robot in enumerate(robots):
        #     robot.robot_id = i
        #     robot.berth_id = i
        #     robot.env = self.env
        #     robot.suppose_pos = robot.pos
        #     robot.last_pos = robot.pos
            
        berths = self.env.berths
        robots = self.env.robots
        cost_matrix_list = self.env.cost_matrix_list
        # 初始化robot

        for i, robot in enumerate(robots):
            robot.robot_id = i
            robot.env = self.env
            robot.suppose_pos = robot.pos
            robot.last_pos = robot.pos

        # 计算robot最近的港口
        distance: List[Tuple[int, int, int]] = []
        for robot in robots:
            for berth in berths:
                distance.append((cost_matrix_list[berth.berth_id][robot.y][robot.x], robot.robot_id, berth.berth_id))
        distance.sort()

        allocated_berth = set()
        robot_berth_init_map = {}
        for item in distance:
            (_, robot_id, berth_id) = item
            if robot_id not in robot_berth_init_map and berth_id not in allocated_berth: 
                robot_berth_init_map[robot_id] = berth_id
                allocated_berth.add(berth_id)

        # 分配robot到港口
        for i, robot in enumerate(robots):
            robot.berth_id = robot_berth_init_map[robot.robot_id]

    
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
        # if goods.price < 180:
        #     return
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
        for order in range(3):
            berth_id = bid_cost_list[order][0]
            self.env.berths[berth_id].add_goods(goods)

    def schedule_robots(self):
        robots = self.env.robots
        berths = self.env.berths
        move_matrix_list = self.env.move_matrix_list
        avg_earn = 0
        for berth in berths:
            avg_earn += berth.earn_when_n[0]
        avg_earn = avg_earn/10

        arrive_rate_list = [(berth.earn_when_n[0], berth) for berth in berths]
        arrive_rate_list.sort(key = lambda x : x[0])

        n = 5
        for lowest_i in range(n):
            cur_berth = arrive_rate_list[lowest_i][1]

            # # 如果不是太低
            # if (cur_berth.earn_when_n[0] >= avg_earn / 2):
            #     return
            
            robot = None
            for _robot in robots:
                if _robot.berth_id == cur_berth.berth_id:
                    robot = _robot
                    # 假设只有一个机器人

            closest_berth = arrive_rate_list[n][1]
            for i, (_, next_berth) in enumerate(arrive_rate_list): 
                if i >= n:
                    if (cur_berth.pos.distance(next_berth.pos) < cur_berth.pos.distance(closest_berth.pos)):
                        closest_berth = next_berth

            if robot is not None:
                robot.change_berth(closest_berth.berth_id)
        # logger.info("from %s to %s", robots[arrive_rate_list[0][0]].berth_id, arrive_rate_list[0][0])
        # robots[arrive_rate_list[9][0]].change_berth(arrive_rate_list[0][0])

        # logger.info("from %s to %s", robots[arrive_rate_list[8][0]].berth_id, arrive_rate_list[1][0])
        # robots[arrive_rate_list[8][0]].change_berth(arrive_rate_list[1][0])

        # logger.info("from %s to %s", robots[arrive_rate_list[7][0]].berth_id, arrive_rate_list[2][0])
        # robots[arrive_rate_list[7][0]].change_berth(arrive_rate_list[2][0])

        # logger.info("from %s to %s", robots[arrive_rate_list[6][0]].berth_id, arrive_rate_list[3][0])
        # robots[arrive_rate_list[6][0]].change_berth(arrive_rate_list[3][0])

        # logger.info("from %s to %s", robots[arrive_rate_list[5][0]].berth_id, arrive_rate_list[4][0])
        # robots[arrive_rate_list[5][0]].change_berth(arrive_rate_list[4][0])