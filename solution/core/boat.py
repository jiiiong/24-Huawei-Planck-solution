from typing import List
from .env import Env
from .berth import Berth

class Boat:
    def __init__(self, num=0, pos=0, status=0, capacity=0):
        self.num = num
        self.pos = pos
        self.status = status

        self.boat_id = -1
        self.env = Env()
        self.capacity = capacity
        self.total_capacity = 0

        # 所属港口
        self.current_berth_id = -1
        # 自虚拟点出发后已过时间
        self.last_run = False

        # 周期算法
        self.associated_berths_list: List[Berth] = []
        self.phase_limited_time_list: List[int] = []
        self.phase_start_time: int = 0
        self.current_elapsed_time: int = 0
        self.num_available_rounds: int = -1
        self.cost_per_round: int = -1
        
    @property
    def used_capacity(self):
        return self.total_capacity - self.capacity

    def init_boat_at_vp(self):
        self.current_elapsed_time = 0
        self.capacity = self.total_capacity

    def ship_boat_from_vp(self, berth_id: int):
        self.init_boat_at_vp()

        self.current_berth_id = berth_id
        # 新增耗时
        self.current_elapsed_time += self.env.berths[berth_id].transport_time
        # 标志港口有船
        self.env.berths[berth_id].boats_id_set.add(self.boat_id)

        print("ship", self.boat_id, berth_id)

    def ship_boat_from_berth(self, new_berth_id: int):
        # 原来港口制空，
        self.env.berths[self.current_berth_id].boats_id_set.remove(self.boat_id)

        # 新增耗时
        self.current_elapsed_time += 500
        # 新港口加入
        self.current_berth_id = new_berth_id
        self.env.berths[new_berth_id].boats_id_set.add(self.boat_id)
        
        print("ship", self.boat_id, new_berth_id)

    def back_vp(self):
        # 原来的港口置空
        self.env.berths[self.current_berth_id].boats_id_set.remove(self.boat_id)
        print("go", self.boat_id)

    def stay_and_fetch_at_berth(self):
        self.current_elapsed_time += 1
        berth = self.env.berths[self.current_berth_id]
        # 装载的货物数量
        num_loaded_gds = min(berth.cur_num_gds, berth.loading_speed)
        num_loaded_gds = min(num_loaded_gds, self.capacity)
        # 剩余的容量
        self.capacity -= num_loaded_gds
        # 港口剩余货物
        berth.cur_num_gds -= num_loaded_gds