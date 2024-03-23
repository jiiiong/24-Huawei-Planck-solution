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
        self.total_num_gds = 1
        self.increase_rate = 0
        self.boats_id_set: Set = set()

        # 可支援的港口
        self.friend_berths: List[Berth] = []
    
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

        # # 计算该队列在只有n个机器人时能够赚多少
        # for i, _ in enumerate(self.earn_when_n):
        #     self.earn_when_n[i] = self.cal_earn_berfore_end_when_n_robots(elements, i+1)
        
    def add_goods(self, goods: Goods):
            
            cost = self.env.cost_matrix_list[self.berth_id][goods.pos.y][goods.pos.x]
            goods.cost = cost
            self.gds_priority_queue.put((-goods.price/(2 * cost), goods))
            self.total_value_of_allocated_goods += goods.price
            self.total_cost_available_goods +=  cost

    def fetch_goods(self) -> Tuple[bool, Goods] : 
        success = False
        fetched_goods: Goods = Goods(-1, [0]) # 无效的，只是为了注释正确
        cost_matrix = self.env.cost_matrix_list[self.berth_id]
        
        # 尝试在自己的优先级队列中取物品
        if not self.gds_priority_queue.empty():
            fetched_goods= self.gds_priority_queue.get(False)[1]
            while ( (fetched_goods.fetched == True or (1000 - (fetched_goods.elapsed_zhen) < cost_matrix[fetched_goods.y][fetched_goods.x] + 5))
                   and not self.gds_priority_queue.empty()):
                fetched_goods = self.gds_priority_queue.get(False)[1]
            # 如果能取到还未被取的
            if fetched_goods.fetched == False:
                success = True

        # 如果没有在当前队列取到货，则尝试帮朋友港口取会过期的货
        if success == False:
            # 用于筛选最优的friend berths会丢失的货
            best_losing_gds_queue: PriorityQueue = PriorityQueue()
            
            # 用于暂存所有friend berths的优先级队列的队列
            pq_list_list: List[List[Tuple[float, Goods]]] = []            
            for i, friend_berth in enumerate(self.friend_berths):
                # 当前friend berth的优先级队列
                pq_list: List[Tuple[float, Goods]] = []
                pq = friend_berth.gds_priority_queue
                while not pq.empty():
                    item: Tuple[float, Goods] = pq.get()
                    if item[1].elapsed_zhen < 1000: # 丢弃超时的
                        pq_list.append(item)
                # 加入pq_list_list用于恢复
                pq_list_list.append(pq_list)

                # 寻找friend berth无法拿到的货物
                # elapsed_time = 0 # 受到friend berth当前小车取货状态的影响，一般来说大于0
                elapsed_time = int(1/friend_berth.cal_increase_rate()) # 受到friend berth当前小车取货状态的影响，一般来说大于0
                for j, item in enumerate(pq_list):
                    tmp_gds = item[1]
                    # 哪些货物能够被friend berth取到
                    go_time = elapsed_time + tmp_gds.cost + 2
                    go_back_time =  elapsed_time + 2 * tmp_gds.cost + 4
                    if (go_time < tmp_gds.remaining_zhen # 物品消失前能取到
                        and go_back_time < self.env.left_zhen - friend_berth.transport_time - 5): # 能赶上最后一趟
                        elapsed_time = go_back_time # 取货来回需要两倍，额外考虑避让的时间
                    # 对于无法被取到的物品，如果没被取过，并且自己能够取到
                    else: 
                        cost = cost_matrix[tmp_gds.pos.y][tmp_gds.pos.x]
                        if (tmp_gds.fetched == False 
                            and (cost + 5 < tmp_gds.remaining_zhen)
                            and cost < (248 - (1/(8 * friend_berth.cal_increase_rate())))):
                            losing_gds = (-tmp_gds.price/(2 * cost), tmp_gds, i, j)
                            best_losing_gds_queue.put(losing_gds) # 需要保存对应的friend_berth和item的索引
            
            # 如果可以拿到一个别人拿不到的货物
            if not best_losing_gds_queue.empty():
                (_, fetched_goods, i_friend_berth, j_item) = best_losing_gds_queue.get()
                pq_list_list[i_friend_berth][j_item][1].fetched = True
                success = True
            
            # 恢复friend_berth的队列
            for i, friend_berth in enumerate(self.friend_berths):
                pq_list = pq_list_list[i]
                for item in pq_list:
                    friend_berth.gds_priority_queue.put(item)
            
        return success, fetched_goods

    # def fetch_goods(self) -> Tuple[bool, Goods] : 
    #     success = False
    #     goods: Goods = Goods(-1, [0]) # 无效的，只是为了注释正确
    #     cost_matrix = self.env.cost_matrix_list[self.berth_id]
    #     if not self.gds_priority_queue.empty():
    #         goods= self.gds_priority_queue.get(False)[1]
    #         while ( (goods.fetched == True or (1000 - (goods.elapsed_zhen) < cost_matrix[goods.y][goods.x] + 5))
    #                and not self.gds_priority_queue.empty()):
    #             goods = self.gds_priority_queue.get(False)[1]
    #         # 如果能取到还未被取的
    #         if goods.fetched == False:
    #             success = True

    #     # logger.info("fetched goods: %s", goods)
    #     return success, goods


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
        return (self.total_num_gds / self.env.global_zhen)
    
    def predict_num_of_goods_after_n(self, n):
        return self.cur_num_gds + self.cal_increase_rate() * n

class Goods:
    def __init__(self, gen_zhen:int, global_zhen_ref: List[int], pos: Point = Point(-2, -2), price: int = 0):
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
