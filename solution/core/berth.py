from typing import List, Tuple
from queue import PriorityQueue
from path_planing import Point

class Berth:
    def __init__(self, x=0, y=0, transport_time=0, loading_speed=0):
        self.pos = Point(x = x, y = y)
        # self.x = x
        # self.y = y
        self.transport_time = transport_time
        self.loading_speed = loading_speed
        self.gds_priority_queue: PriorityQueue = PriorityQueue()
        self.overload: int = 0
        self.total_val: int = 0
        self.cost_time: int = 0
        self.num_gds = 0
        
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


class Goods:
    def __init__(self, pos: Point = Point(-1, -1), price: int = 0):
        self.pos = pos
        self.price = price
        self.left_lift = 200
        self.fetched = False
        
    def __lt__(self, b):
        return True
