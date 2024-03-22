from __future__ import annotations
from typing import List, Tuple, Set
from queue import PriorityQueue, Queue, LifoQueue

from core import Env
from path_planing import Point
from log import logger, error_logger

class Berth:
    def __init__(self, x=0, y=0, transport_time=0, loading_speed=0):
        self.pos = Point(x = x, y = y)
        # self.x = x
        # self.y = y
        self.transport_time = transport_time
        self.loading_speed = loading_speed
        self.gds_priority_queue = PriorityQueue()
        self.cur_num_gds = 0
        self.total_earn = 0
        self.total_value_of_allocated_goods = 0

        self.berth_id: int = 0
        self.env: Env = Env()

        self.num_available_goods: int = 0
        self.total_cost_available_goods: int = 0

        # 基于排队论模型的优先级计算
        self.earn_when_n: List[float] = [0.0, 0.0, 0.0]
        self.total_num_gds = 0
        self.increase_rate = 0
        self.boats_id_set: Set = set()
    
    @property
    def have_boats(self):
        return False if len(self.boats_id_set) == 0 else True
    
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
    
    def clear_queue(self):
        pq = self.gds_priority_queue
        elements = []
        total_cost_available_goods = 0
        while not pq.empty():
            item: Tuple[float, Goods] = pq.get()
            if (item[1].elapsed_zhen < 1000):
                elements.append(item)
                total_cost_available_goods += item[1].cost
        self.total_cost_available_goods = total_cost_available_goods
        for item in elements:
            pq.put(item)
        # 计算该队列在只有n个机器人时能够赚多少
        for i, _ in enumerate(self.earn_when_n):
            self.earn_when_n[i] = self.cal_earn_berfore_end_when_n_robots(elements, i+1)
        

    def add_goods(self, goods: Goods):
            
            cost = self.env.cost_matrix_list[self.berth_id][goods.pos.y][goods.pos.x]
            goods.cost = cost
            self.gds_priority_queue.put((-goods.price/(2 * cost), goods))
            self.total_value_of_allocated_goods += goods.price
            self.total_cost_available_goods +=  cost

    def fetch_goods(self) -> Tuple[bool, Goods] : 
        success = False
        goods: Goods = Goods(-1, [0]) # 无效的，只是为了注释正确
        cost_matrix = self.env.cost_matrix_list[self.berth_id]
        if not self.gds_priority_queue.empty():
            goods= self.gds_priority_queue.get(False)[1]
            while ( (goods.fetched == True or (1000 - (goods.elapsed_zhen) < cost_matrix[goods.y][goods.x] + 5))
                   and not self.gds_priority_queue.empty()):
                goods = self.gds_priority_queue.get(False)[1]
            # 如果能取到还未被取的
            if goods.fetched == False:
                success = True

        # logger.info("fetched goods: %s", goods)
        return success, goods

    def cal_earn_berfore_end_when_n_robots(self, pq: List[Tuple[float, Goods]], n:int):
        earn = 0
        limit_time = 15000 - self.env.global_zhen
        ordered_n_elapsed_time = [0 for _ in range(n)]
        for item in pq:
            ordered_n_elapsed_time.sort()
            goods = item[1]
            if (goods.remaining_zhen > (ordered_n_elapsed_time[0] + goods.cost + 5)
                and goods.fetched == False): # 表示货物在经过elapsed_time后，还能被取到
                ordered_n_elapsed_time[0] += 2*goods.cost + 5    # 表示取货需要花费2*goods.cost + 5长的时间
                if ordered_n_elapsed_time[0] > limit_time:   # 如果超时
                    break
                earn += item[1].price
        alpha = 0.5
        earn = self.earn_when_n[n-1] * alpha + earn * (1-alpha)
        return earn

    def cal_increase_rate(self): # 最开始几帧
        return self.total_num_gds / self.env.global_zhen
    
    def predict_num_of_goods_after_n(self, n):
        return self.cur_num_gds + self.cal_increase_rate() * n

class Goods:
    def __init__(self, gen_zhen:int, global_zhen_ref: List[int], pos: Point = Point(-1, -1), price: int = 0):
        self.pos = pos
        self.price = price
        self.cost = -1

        self.fetched = False

        self.gen_zhen = gen_zhen
        self.global_zhen_ref = global_zhen_ref
    @property
    def elapsed_zhen(self):
        return self.global_zhen_ref[0] - self.gen_zhen
    
    @property
    def remaining_zhen(self):
        return 1000 - self.global_zhen_ref[0] + self.gen_zhen

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

    def __repr__(self) -> str:
        return "cost: " + str(self.cost) + ", price: " + str(self.price)
    
    def __lt__(self, b):
        return True
