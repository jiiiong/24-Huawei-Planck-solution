from __future__ import annotations
from typing import List, Tuple
from queue import PriorityQueue, Queue

from path_planing import Point
from log import logger

class Berth:
    def __init__(self, x=0, y=0, transport_time=0, loading_speed=0):
        self.pos = Point(x = x, y = y)
        # self.x = x
        # self.y = y
        self.transport_time = transport_time
        self.loading_speed = loading_speed
        self.gds_priority_queue = PriorityQueue()
        self.overload: int = 0
        self.total_val: int = 0
        self.cost_time: int = 0
        self.num_gds = 0
        self.total_earn = 0
        self.total_value_of_allocated_goods = 0

        self.cost_matrix: List[List[int]] = []

        # 基于排队论模型的优先级计算
        self.arrival_rate_estimator = ArrivalRateEstimator()

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
    
    def get_estimated_rate(self) -> float:
        rate = self.arrival_rate_estimator.estimated_rate
        if rate is None: return 0
        else: return rate

    def add_goods(self, goods: Goods):
            
            self.gds_priority_queue.put((self.cost_matrix[goods.pos.y][goods.pos.x], goods))
            self.total_value_of_allocated_goods += goods.price
            self.arrival_rate_estimator.update(goods.gen_zhen)
    
    def fetch_goods(self) -> Tuple[bool, Goods] : 
        success = False
        goods: Goods = Goods(-1) # 无效的，只是为了注释正确
        if not self.gds_priority_queue.empty():
            goods= self.gds_priority_queue.get(False)[1]
            while (goods.fetched == True and not self.gds_priority_queue.empty()):
                goods = self.gds_priority_queue.get(False)[1]
            # 如果能取到还未被取的
            if goods.fetched == False:
                success = True
        return success, goods

class Goods:
    def __init__(self, gen_zhen:int, pos: Point = Point(-1, -1), price: int = 0):
        self.pos = pos
        self.price = price
        
        self.fetched = False

        self.gen_zhen = gen_zhen
        self.left_zhen = 200
    # def __repr__(self) -> str:
    #     return repr(self.pos)
    def __lt__(self, b):
        return True

class ArrivalRateEstimator:
    def __init__(self, alpha: float=0.99):
        self.alpha = alpha
        self.estimated_rate = 0
        self.prev_arrival_time = 0
        self.cur_rate = 0

    def update(self, arrival_time: float):
        inter_arrival_time = arrival_time - self.prev_arrival_time
        if inter_arrival_time == 0: # 同一帧生成多个
            self.estimated_rate += (1 - self.alpha) * self.cur_rate
        else:
            self.cur_rate = 1 / inter_arrival_time
            self.estimated_rate = self.alpha * self.estimated_rate + (1 - self.alpha) * (1 / inter_arrival_time)
            self.prev_arrival_time = arrival_time

        # if self.prev_arrival_time is None:
        #     self.prev_arrival_time = arrival_time
        #     self.estimated_rate = 0
        # else:
        #     inter_arrival_time = arrival_time - self.prev_arrival_time
        #     if self.estimated_rate is None:
        #         self.estimated_rate = 1 / inter_arrival_time
        #     else:
        #         self.estimated_rate = self.alpha * self.estimated_rate + (1 - self.alpha) * (1 / inter_arrival_time)
        #     self.prev_arrival_time = arrival_time

