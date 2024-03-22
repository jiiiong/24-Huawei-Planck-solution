from typing import List, Tuple, Set, Dict
from queue import LifoQueue, Queue, PriorityQueue

from log import logger, error_logger
from core import Env
from core import Robot, Robot_Extended_Status
from core import Berth, Goods, Boat
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
        #     robot.berth_id = 0
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

    def init_boats(self):
        boats = self.env.boats

        # 初始化boat
        for i, boat in enumerate(boats):
            boat.env = self.env
            boat.boat_id = i
            boat.capacity = self.env.boat_capacity
            boat.total_capacity = self.env.boat_capacity
            boat.pos = -1
            boat.last_run = False
        
        # 初始化最近的港口id列表
        self.tran_time_ordered_list = [(berth.berth_id, berth.transport_time) for berth in self.env.berths]
        self.tran_time_ordered_list.sort(key=lambda x:x[1])
        self.tran_time_ordered_berths_id_list = [tup[0] for tup in self.tran_time_ordered_list]

        # self.boat_berths_map: Dict[int, List[int]]  = {}
        # for i in range(10):
        #     berths_id_list = []
        #     for j in range(10):
        #         if j != i: berths_id_list.append(j)
        #     self.boat_berths_map[i] = berths_id_list
        
        berths_cost_time = [(berth_id, berth.transport_time + int(self.env.boat_capacity/berth.loading_speed)+1 )
                            for berth_id, berth in enumerate(self.env.berths)]
        
        berths_cost_time.sort(key=lambda x:x[1])


        self.boat_berths_map = {
            0: [berths_cost_time[0][0], berths_cost_time[9][0]],
            1: [berths_cost_time[1][0], berths_cost_time[8][0]],
            2: [berths_cost_time[2][0], berths_cost_time[7][0]],
            3: [berths_cost_time[3][0], berths_cost_time[6][0]],
            4: [berths_cost_time[4][0], berths_cost_time[5][0]],
        }

        

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
        
        # delegated_berth_id = self.env.divide_matrix[goods.y][goods.x]
        # if (delegated_berth_id != -1):
        #     self.env.berths[delegated_berth_id].add_goods(goods)

        cost_matrix_list = self.env.cost_matrix_list
        bid_cost_list: List[Tuple[int, float]] = []
        for berth_id in range(self.env.berth_num):
            cost = cost_matrix_list[berth_id][goods.y][goods.x]
            bid_cost_list.append((berth_id, cost))
        bid_cost_list.sort(key=lambda x: x[1])
        # 将货物放入当前最近的3个港口中
        for order in range(3):
            berth_id = bid_cost_list[order][0]
            self.env.berths[berth_id].add_goods(goods)

    def best_move_for_boat_at_vp(self, cur_boat: Boat):
        # 转移港口分数
        max_next_berth_point = -1
        max_next_berth_id = -1
        berths = self.env.berths
        for next_berth in berths:
            # 目标港口没有boat，考虑到增长率，不限制现存货物大于0
            if (next_berth.have_boats == False):
                # 到达时的货物量，现有的+tran_time*增长率
                num_of_gds_when_arrive = next_berth.cur_num_gds + next_berth.transport_time * next_berth.increase_rate
                # 已过去的时间，到达，取货，并会vp所花时间
                cosuming_time = 0 + 2 * next_berth.transport_time + (num_of_gds_when_arrive / next_berth.loading_speed)
                cur_point = max(num_of_gds_when_arrive, cur_boat.total_capacity) / cosuming_time
                if cur_point > max_next_berth_point:
                    max_next_berth_point = cur_point
                    max_next_berth_id = next_berth.berth_id
        cur_boat.ship_boat_from_vp(max_next_berth_id)

    def best_move_for_boat_at_berth(self, cur_boat: Boat):
        # 满载了
        if cur_boat.capacity == 0:
            cur_boat.back_vp()
        # 还有容量
        else:
            berths = self.env.berths
            cur_berth = berths[cur_boat.current_berth_id]
            num_of_gds_when_stay = min((cur_boat.used_capacity + cur_berth.cur_num_gds),cur_boat.total_capacity)
            # stay_point =  num_of_gds_when_stay / (num_of_gds_when_stay/cur_berth.loading_speed)
            stay_point =  num_of_gds_when_stay / (cur_boat.current_elapsed_time + num_of_gds_when_stay/cur_berth.loading_speed + cur_berth.transport_time)
                            #/ (cur_boat.current_elapsed_time + cur_berth.transport_time)
                
            # 回虚拟点的分数： 现有货量 / 已经过去的时间+回去所需时间
            back_point = (cur_boat.used_capacity) / (cur_boat.current_elapsed_time + cur_berth.transport_time)
            # back_point = 0
            # 尝试转移到其他港口，要求：
            # 没有其他船只
            # 能扩大收益
            # 转移港口分数
            max_next_berth_point = -1
            max_next_berth_id = -1
            for next_berth in berths:
                # 目标港口没有boat，并且有货
                if (next_berth.have_boats == False and next_berth.cur_num_gds > 0):
                    # 到达时的货物量，现有的+500*增长率
                    num_of_gds_when_arrive = next_berth.cur_num_gds + 500 * next_berth.cal_increase_rate() * 0.9
                    # 已过去的时间，到达，取货，并会vp所花时间
                    # cosuming_time = 500 + (num_of_gds_when_arrive / next_berth.loading_speed)
                    cosuming_time = cur_boat.current_elapsed_time + 500 + (num_of_gds_when_arrive / next_berth.loading_speed) + next_berth.transport_time
                    cur_point = min((cur_boat.used_capacity + num_of_gds_when_arrive), cur_boat.total_capacity) / cosuming_time
                    if cur_point > max_next_berth_point:
                        max_next_berth_point = cur_point
                        max_next_berth_id = next_berth.berth_id

            # 呆在原地的情况？？？估计不行
            error_logger.error("zhen: %s, boat_id: %s, berth_id: %s, stay: %s back: %s, move %s: %s, ",
                               self.env.global_zhen, cur_boat.boat_id, cur_berth.berth_id, stay_point, back_point, max_next_berth_id, max_next_berth_point)
            if back_point > max(max_next_berth_point, stay_point) :
                cur_boat.back_vp()
            elif max_next_berth_point > max(back_point, stay_point):
                cur_boat.ship_boat_from_berth(max_next_berth_id)
            else: 
                cur_boat.stay_and_fetch_at_berth()

    def schedule_boats(self):
        env = self.env
        boats = self.env.boats
        berths = self.env.berths
        global_zhen = env.global_zhen
        # 遍历调度每一艘轮船
        for cur_boat in (boats):
            # 前800帧，不做处理，因为每个港口的increase_rate还不确定
            if (global_zhen <= 800): #???????????????????????????????????????
                if (global_zhen == 1000):
                    # 分配到最近的几个港口
                    cur_boat.ship_boat_from_vp(self.tran_time_ordered_berths_id_list[cur_boat.boat_id])
                return
            
            # 当boat在某个港口时
            if (cur_boat.status == 1):
                if cur_boat.pos == -1: # 在虚拟点
                    self.best_move_for_boat_at_vp(cur_boat)
                else:
                    self.best_move_for_boat_at_berth(cur_boat)
 
    def schedule_boats_2(self):
        boats = self.env.boats
        berths = self.env.berths
        # 遍历调度每一艘轮船
        for cur_boat_id, berths_id_list in self.boat_berths_map.items():
            cur_boat = boats[cur_boat_id]
            # 如果已经到达虚拟点
            if cur_boat.status == 1 and cur_boat.pos == -1: 
                # 将轮船调度到第一个目标点
                cur_boat.capacity = self.env.boat_capacity
                print("ship", cur_boat_id, berths_id_list[0])

            # 已经到达第一个目标港口
            elif cur_boat.status == 1 and cur_boat.pos == berths_id_list[0]: 
                berth = berths[berths_id_list[0]]
                # 计算港口这 一帧 能装载的货物数量
                num_loaded_gds = min(berth.cur_num_gds, berth.loading_speed)
                # 但考虑床的容量，不一定能装那么多
                num_loaded_gds = min(num_loaded_gds, cur_boat.capacity)
                ##########################？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？装货在每一帧的最后结算
                #》》》》》》》》》》？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？所以实际上下一帧才可以 减少，暂时不管
                # 计算港口剩余货量
                berth.cur_num_gds -= num_loaded_gds
                # 计算boat的剩余容量
                cur_boat.capacity -= num_loaded_gds
                
                # 调度部分***************
                # 如果满货了, 或快没时间了
                if (cur_boat.last_run == True):
                    if self.env.left_zhen < berths[berths_id_list[0]].transport_time + 2:
                        print("go", cur_boat_id)
                elif cur_boat.capacity == 0 or self.env.left_zhen < berths[berths_id_list[0]].transport_time + 2: 
                    print("go", cur_boat_id)
                # 如果没货了，则调度船到第二个港口
                elif berth.cur_num_gds == 0:
                    print("ship", cur_boat_id, berths_id_list[1])
            # 到达第二个目标港口
            elif cur_boat.status == 1 and cur_boat.pos == berths_id_list[1]: 
                berth = berths[berths_id_list[1]]
                # 计算港口这 一帧 能装载的货物数量
                num_loaded_gds = min(berth.cur_num_gds, berth.loading_speed)
                # 但考虑床的容量，不一定能装那么多
                num_loaded_gds = min(num_loaded_gds, cur_boat.capacity)
                
                ##########################？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？装货在每一帧的最后结算
                #》》》》》》》》》》？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？所以实际上下一帧才可以 减少，暂时不管
                # 计算港口剩余货量
                berth.cur_num_gds -= num_loaded_gds
                # 计算boat的剩余容量
                cur_boat.capacity -= num_loaded_gds
                
                # 调度部分***************
                # 如果满货了,或者 港口无货了
                if cur_boat.capacity == 0 or self.env.left_zhen < berths[berths_id_list[0]].transport_time + 2: 
                    print("go", cur_boat_id)
                # 如果不够 1->vp->0->vp
                elif (berth.cur_num_gds == 0):
                    if (self.env.left_zhen < berths[berths_id_list[1]].transport_time + berths[berths_id_list[0]].transport_time * 2 + 73):
                        for robot in self.env.robots:
                            if robot.berth_id == berths_id_list[1]:
                                robot.change_berth(berths_id_list[0])
                        cur_boat.last_run = True
                        print("ship", cur_boat_id, berths_id_list[0])
                    elif(self.env.left_zhen < berths[berths_id_list[1]].transport_time\
                                                + berths[berths_id_list[0]].transport_time \
                                                + 73 + 500 + berths[berths_id_list[1]].transport_time + 73):
                        for robot in self.env.robots:
                            if robot.berth_id == berths_id_list[1]:
                                robot.change_berth(berths_id_list[0])
                        cur_boat.last_run = True
                        print("go", cur_boat_id)
                    else:
                        print("go", cur_boat_id)

    def schedule_boats_3(self):
        boats = self.env.boats
        berths = self.env.berths
        # 遍历调度每一艘轮船
        for cur_boat_id, berths_id_list in self.boat_berths_map.items():
            cur_boat = boats[cur_boat_id]
            # 如果已经到达虚拟点

            if ((cur_boat.pos ==  berths_id_list[1]
                and self.env.left_zhen == berths[berths_id_list[1]].transport_time + berths[berths_id_list[0]].transport_time \
                                     + 500 + berths[berths_id_list[1]].transport_time + 72 + 72) 
                or
                (cur_boat.pos ==  berths_id_list[0]
                and self.env.left_zhen == berths[berths_id_list[0]].transport_time + berths[berths_id_list[0]].transport_time \
                                     + 500 + berths[berths_id_list[1]].transport_time + 72 + 72)):
                
                print("go", cur_boat_id)
                error_logger.error("zhen: %s, boat_id: %s, go ",
                        self.env.global_zhen, cur_boat.boat_id )
                # for robot in self.env.robots:
                #     if robot.berth_id == berths_id_list[1]:
                #         robot.change_berth(berths_id_list[0])

            elif cur_boat.status == 1 and cur_boat.pos == -1: 
                # 将轮船调度到第一个目标点
                cur_boat.capacity = self.env.boat_capacity
                print("ship", cur_boat_id, berths_id_list[0])
                error_logger.error("zhen: %s, boat_id: %s, ship from %s to %s ",
                        self.env.global_zhen, cur_boat.boat_id, -1, berths_id_list[0])
            # 已经到达第一个目标港口
            elif cur_boat.status == 1 and cur_boat.pos == berths_id_list[0]: 
                berth = berths[berths_id_list[0]]
                # 计算港口这 一帧 能装载的货物数量
                num_loaded_gds = min(berth.cur_num_gds, berth.loading_speed)
                # 但考虑床的容量，不一定能装那么多
                num_loaded_gds = min(num_loaded_gds, cur_boat.capacity)
                ##########################？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？装货在每一帧的最后结算
                #》》》》》》》》》》？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？所以实际上下一帧才可以 减少，暂时不管
                # 计算港口剩余货量
                berth.cur_num_gds -= num_loaded_gds
                # 计算boat的剩余容量
                cur_boat.capacity -= num_loaded_gds
                
                # 调度部分***************
                # 如果满货了, 或快没时间了
                if cur_boat.capacity == 0 or self.env.left_zhen < berths[berths_id_list[0]].transport_time + 2: 
                    print("go", cur_boat_id)
                    error_logger.error("zhen: %s, boat_id: %s, go ",
                        self.env.global_zhen, cur_boat.boat_id )
                # 如果没货了，则调度船到第二个港口
                elif berth.cur_num_gds == 0:
                    print("ship", cur_boat_id, berths_id_list[1])
                    error_logger.error("zhen: %s, boat_id: %s, ship from %s to %s ",
                        self.env.global_zhen, cur_boat.boat_id, berths_id_list[0], berths_id_list[1])
            # 到达第二个目标港口
            elif cur_boat.status == 1 and cur_boat.pos == berths_id_list[1]: 
                berth = berths[berths_id_list[1]]
                # 计算港口这 一帧 能装载的货物数量
                num_loaded_gds = min(berth.cur_num_gds, berth.loading_speed)
                # 但考虑床的容量，不一定能装那么多
                num_loaded_gds = min(num_loaded_gds, cur_boat.capacity)
                
                ##########################？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？装货在每一帧的最后结算
                #》》》》》》》》》》？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？所以实际上下一帧才可以 减少，暂时不管
                # 计算港口剩余货量
                berth.cur_num_gds -= num_loaded_gds
                # 计算boat的剩余容量
                cur_boat.capacity -= num_loaded_gds
                
                # 调度部分***************
                # 如果满货了,或者 港口无货了
                if cur_boat.capacity == 0 or berth.cur_num_gds == 0 or self.env.left_zhen < berths[berths_id_list[0]].transport_time + 2: 
                    print("go", cur_boat_id)
                    error_logger.error("zhen: %s, boat_id: %s, go ",
                        self.env.global_zhen, cur_boat.boat_id )

    def schedule_boats_5(self):
        boats = self.env.boats
        berths = self.env.berths
        boat_capacity = self.env.boat_capacity
        # 遍历调度每一艘轮船
        for cur_boat_id, berths_id_list in self.boat_berths_map.items():
            cur_boat = boats[cur_boat_id]
            # 如果已经到达虚拟点
            if (self.env.global_zhen <= 15000 
                                        - (berths[berths_id_list[0]].transport_time + berths[berths_id_list[1]].transport_time 
                                            + 500
                                            + int(boat_capacity/berths[berths_id_list[0]].loading_speed) 
                                            + int(boat_capacity/berths[berths_id_list[1]].loading_speed)
                                        ) *5 ):
                continue
            
            if (self.env.left_zhen == berths[berths_id_list[0]].transport_time + berths[berths_id_list[1]].transport_time \
                                     + 500 + berths[berths_id_list[1]].transport_time + 
                                     + int(boat_capacity/berths[berths_id_list[0]].loading_speed) + int(boat_capacity/berths[berths_id_list[1]].loading_speed)
                and not cur_boat.last_run == True):

                # print("go", cur_boat_id)
                cur_boat.last_run = True
                error_logger.error("!!zhen: %s, boat_id: %s, go ",
                        self.env.global_zhen, cur_boat.boat_id )
                # for robot in self.env.robots:
                #     if robot.berth_id == berths_id_list[1]:
                #         robot.change_berth(berths_id_list[0])

            elif cur_boat.status == 1 and cur_boat.pos == -1: 
                # 将轮船调度到第一个目标点
                cur_boat.capacity = self.env.boat_capacity
                print("ship", cur_boat_id, berths_id_list[0])
                error_logger.error("zhen: %s, boat_id: %s, ship from %s to %s ",
                        self.env.global_zhen, cur_boat.boat_id, -1, berths_id_list[0])
            # 已经到达第一个目标港口
            elif cur_boat.status == 1 and cur_boat.pos == berths_id_list[0]: 
                berth = berths[berths_id_list[0]]
                # 计算港口这 一帧 能装载的货物数量
                num_loaded_gds = min(berth.cur_num_gds, berth.loading_speed)
                # 但考虑床的容量，不一定能装那么多
                num_loaded_gds = min(num_loaded_gds, cur_boat.capacity)
                ##########################？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？装货在每一帧的最后结算
                #》》》》》》》》》》？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？所以实际上下一帧才可以 减少，暂时不管
                # 计算港口剩余货量
                berth.cur_num_gds -= num_loaded_gds
                # 计算boat的剩余容量
                cur_boat.capacity -= num_loaded_gds
                

                if cur_boat.last_run == True:
                    if self.env.left_zhen < berths[berths_id_list[1]].transport_time + self.env.boat_capacity + 500:
                        print("ship", cur_boat_id, berths_id_list[1])
                        # # 调度机器人
                        # if (cur_boat.last_run == True):
                        #     for robot in self.env.robots:
                        #         if robot.berth_id == berths_id_list[0]:
                        #             robot.change_berth(berths_id_list[1])
                        #             error_logger.error("zhen: %s, boat_id: %s, ship from %s to %s ",
                        #                                 self.env.global_zhen, cur_boat.boat_id, berths_id_list[0], berths_id_list[1])
                # 调度部分***************
                # 如果满货了, 或快没时间了
                elif cur_boat.capacity == 0 or self.env.left_zhen < berths[berths_id_list[0]].transport_time: 
                    # print("go", cur_boat_id)
                    error_logger.error("zhen: %s, boat_id: %s, go ",
                        self.env.global_zhen, cur_boat.boat_id )
                # 如果没货了，则调度船到第二个港口
                elif berth.cur_num_gds == 0:
                    print("ship", cur_boat_id, berths_id_list[1])


            # 到达第二个目标港口
            elif cur_boat.status == 1 and cur_boat.pos == berths_id_list[1]: 
                berth = berths[berths_id_list[1]]
                # 计算港口这 一帧 能装载的货物数量
                num_loaded_gds = min(berth.cur_num_gds, berth.loading_speed)
                # 但考虑床的容量，不一定能装那么多
                num_loaded_gds = min(num_loaded_gds, cur_boat.capacity)
                
                ##########################？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？装货在每一帧的最后结算
                #》》》》》》》》》》？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？所以实际上下一帧才可以 减少，暂时不管
                # 计算港口剩余货量
                berth.cur_num_gds -= num_loaded_gds
                # 计算boat的剩余容量
                cur_boat.capacity -= num_loaded_gds
                
                # 调度部分***************
                # 如果满货了,或者 港口无货了
                if cur_boat.last_run == True:
                    if self.env.left_zhen < berths[berths_id_list[1]].transport_time + 2:
                        print("go", cur_boat_id)
                        
                elif cur_boat.capacity == 0 or berth.cur_num_gds == 0 or self.env.left_zhen < berths[berths_id_list[1]].transport_time + 2: 
                    print("go", cur_boat_id)
                    error_logger.error("zhen: %s, boat_id: %s, go ",
                        self.env.global_zhen, cur_boat.boat_id )

    def schedule_boats_4(self):
        boats = self.env.boats
        berths = self.env.berths
        boat_capacity = self.env.boat_capacity
        # 遍历调度每一艘轮船
        for cur_boat_id, berths_id_list in self.boat_berths_map.items():
            cur_boat = boats[cur_boat_id]
            # 如果已经到达虚拟点
            if (self.env.global_zhen <= 15000 
                                        - (berths[berths_id_list[0]].transport_time + berths[berths_id_list[1]].transport_time 
                                            + 500
                                            + int(boat_capacity/berths[berths_id_list[0]].loading_speed) 
                                            + int(boat_capacity/berths[berths_id_list[1]].loading_speed)
                                        ) *5 ):
                continue
            
            if (self.env.left_zhen == berths[berths_id_list[0]].transport_time + berths[berths_id_list[1]].transport_time \
                                     + 500 + berths[berths_id_list[1]].transport_time + 
                                     + int(boat_capacity/berths[berths_id_list[0]].loading_speed) + int(boat_capacity/berths[berths_id_list[1]].loading_speed)
                and not cur_boat.last_run == True):

                # print("go", cur_boat_id)
                cur_boat.last_run = True
                error_logger.error("!!zhen: %s, boat_id: %s, go ",
                        self.env.global_zhen, cur_boat.boat_id )
                # for robot in self.env.robots:
                #     if robot.berth_id == berths_id_list[1]:
                #         robot.change_berth(berths_id_list[0])

            elif cur_boat.status == 1 and cur_boat.pos == -1: 
                # 将轮船调度到第一个目标点
                cur_boat.capacity = self.env.boat_capacity
                print("ship", cur_boat_id, berths_id_list[0])
                error_logger.error("zhen: %s, boat_id: %s, ship from %s to %s ",
                        self.env.global_zhen, cur_boat.boat_id, -1, berths_id_list[0])
            # 已经到达第一个目标港口
            elif cur_boat.status == 1 and cur_boat.pos == berths_id_list[0]: 
                berth = berths[berths_id_list[0]]
                # 计算港口这 一帧 能装载的货物数量
                num_loaded_gds = min(berth.cur_num_gds, berth.loading_speed)
                # 但考虑床的容量，不一定能装那么多
                num_loaded_gds = min(num_loaded_gds, cur_boat.capacity)
                ##########################？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？装货在每一帧的最后结算
                #》》》》》》》》》》？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？所以实际上下一帧才可以 减少，暂时不管
                # 计算港口剩余货量
                berth.cur_num_gds -= num_loaded_gds
                # 计算boat的剩余容量
                cur_boat.capacity -= num_loaded_gds
                

                if cur_boat.last_run == True:
                    if self.env.left_zhen < berths[berths_id_list[1]].transport_time + self.env.boat_capacity + 500:
                        print("ship", cur_boat_id, berths_id_list[1])
                        # # 调度机器人
                        # if (cur_boat.last_run == True):
                        #     for robot in self.env.robots:
                        #         if robot.berth_id == berths_id_list[0]:
                        #             robot.change_berth(berths_id_list[1])
                        #             error_logger.error("zhen: %s, boat_id: %s, ship from %s to %s ",
                        #                                 self.env.global_zhen, cur_boat.boat_id, berths_id_list[0], berths_id_list[1])
                # 调度部分***************
                # 如果满货了, 或快没时间了
                elif cur_boat.capacity == 0 or self.env.left_zhen < berths[berths_id_list[0]].transport_time: 
                    # print("go", cur_boat_id)
                    error_logger.error("zhen: %s, boat_id: %s, go ",
                        self.env.global_zhen, cur_boat.boat_id )
                # 如果没货了，则调度船到第二个港口
                elif berth.cur_num_gds == 0:
                    print("ship", cur_boat_id, berths_id_list[1])


            # 到达第二个目标港口
            elif cur_boat.status == 1 and cur_boat.pos == berths_id_list[1]: 
                berth = berths[berths_id_list[1]]
                # 计算港口这 一帧 能装载的货物数量
                num_loaded_gds = min(berth.cur_num_gds, berth.loading_speed)
                # 但考虑床的容量，不一定能装那么多
                num_loaded_gds = min(num_loaded_gds, cur_boat.capacity)
                
                ##########################？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？装货在每一帧的最后结算
                #》》》》》》》》》》？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？所以实际上下一帧才可以 减少，暂时不管
                # 计算港口剩余货量
                berth.cur_num_gds -= num_loaded_gds
                # 计算boat的剩余容量
                cur_boat.capacity -= num_loaded_gds
                
                # 调度部分***************
                # 如果满货了,或者 港口无货了
                if cur_boat.last_run == True:
                    if self.env.left_zhen < berths[berths_id_list[1]].transport_time + 2:
                        print("go", cur_boat_id)
                        
                elif cur_boat.capacity == 0 or berth.cur_num_gds == 0 or self.env.left_zhen < berths[berths_id_list[1]].transport_time + 2: 
                    print("go", cur_boat_id)
                    error_logger.error("zhen: %s, boat_id: %s, go ",
                        self.env.global_zhen, cur_boat.boat_id )

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