from __future__ import annotations
from typing import List, Tuple, Set
import typing
if typing.TYPE_CHECKING:
    from .robot import Robot
    from .berth import Berth, Goods
    from .boat import Boat
from queue import PriorityQueue
from dataclasses import dataclass

from path_planing import Point


N:int = 210

@dataclass
class Env:
    n:int = 200
    robot_num:int  = 10
    berth_num:int = 10

    money:int = 0
    boat_capacity:int = 0
    id:int = 0

    total_map_gds_value:int = 0
    
    def __init__(self):
        self.ch: List[List[str]] = []#每行只有一个元素
        self.gds:List[List[int]] = [[0 for _ in range(N)] for _ in range(N)]

        self.global_zhen_ref:List[int] = [0]
        # for path_planing 
        self.value_matrix:  List[List[int]]   = [[] for _ in range(10)] # 用来表示每个位置的开销，0代表不可通行
        self.divide_matrix: List[List[int]] = []
        self.cost_matrix_list:List[ (List[List[int]]) ]   = []
        self.move_matrix_list:  List[ (List[List[Point]]) ] = []
        # global queue for goods
        # List[PriorityQueue[Tuple[int, Point]]]
        self.berth_gds_priority_queue_list:List[PriorityQueue] = [PriorityQueue() for _ in range(self.berth_num)]


    @property
    def global_zhen(self):
        return self.global_zhen_ref[0]
    @global_zhen.setter
    def global_zhen(self, value):
        self.global_zhen_ref[0] = value
    
    @property
    def left_zhen(self):
        return 15000 - self.global_zhen_ref[0]

    def init_env(self, robots, berths, boats):
        self.robots:List[Robot] = robots
        self.berths:List[Berth] = berths
        self.boats:List[Boat] = boats