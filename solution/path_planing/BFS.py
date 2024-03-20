import queue
from typing import List, Tuple

from .utils import fileMap2Matrix, UNREACHABLE
from .point import Point

def BFS(value_matrix: List[List[int]], source_point: Point) -> Tuple[List[List[Point]], List[List[int]]]:
    '''
    输入：
    value_matrix: 表示是否为障碍物，0表示障碍物，1表示非障碍物
    输出：
    cost_matrix: 地图上任一点到source point需要移动的步数, 若不可达, 则为-1
    move_matrix: 表示从该点回source point需要移动的方向, -2,-2表示不可达
    '''
    frontier = queue.Queue()
    came_from = dict()
    cost_matrix = [[2147483647] * 200 for _ in range(200)]
    move_matrix = [ [UNREACHABLE] * 200 for _ in range(200)]

    frontier.put(source_point)
    came_from[source_point] = None
    cost_matrix[source_point.y][source_point.x] = 0
    move_matrix[source_point.y][source_point.x] = Point(0, 0)
    
    # 未考虑初始点为障碍物时的情况
    while frontier.empty() is False:
        cur = frontier.get()
        for offset in [(-1, 0), (1, 0), (0 , -1), (0, 1)]:
            offset = Point(offset[0], offset[1])
            next:Point = cur + offset
            if (0<=next.y and next.y<200 and 0<=next.x and next.x<200
                and value_matrix[next.y][next.x] != 0
                and next not in came_from):
                frontier.put(next)
                came_from[next] = cur
                cost_matrix[next.y][next.x] = cost_matrix[cur.y][cur.x] + 1
                move_matrix[next.y][next.x] = -offset

    return move_matrix, cost_matrix

def one_move_avoidance(value_matrix: List[List[int]], source_point: Point) -> Tuple[List[Point], bool]:
    '''
    输入：
    value_matrix: 表示是否为障碍物，0表示障碍物，1表示非障碍物
    原点必为障碍物？？？
    输出：
    
    '''
    success = False
    avoidance_paths: List[Point] = []
    
    # 未考虑初始点为障碍物时的情况
    y_len = len(value_matrix)
    x_len = len(value_matrix[0])

    for offset in [(0, 0), (-1, 0), (1, 0), (0 , -1), (0, 1)]:
        offset = Point(offset[0], offset[1])
        next:Point = source_point + offset
        if (0<=next.y and next.y < y_len 
            and 0<=next.x and next.x < x_len
            and value_matrix[next.y][next.x] != 0
            ): # 下一个位置不会碰撞
            avoidance_paths.append(next)
            success = True
            break

    return (avoidance_paths, success)

def BFS_file(file, source_point: Point) -> Tuple[List[List[Point]], List[List[int]]]:
    matrix = fileMap2Matrix(file)
 
    frontier = queue.Queue()
    came_from = dict()
    cost_matrix = [[-1] * 200 for _ in range(200)]
    move_matrix = [ [Point()] * 200 for _ in range(200)]

    frontier.put(source_point)
    came_from[source_point] = None
    cost_matrix[source_point.y][source_point.x] = 0
    move_matrix[source_point.y][source_point.x] = Point(0, 0)
    
    # 未考虑初始点为障碍物时的情况
    while frontier.empty() is False:
        cur = frontier.get()
        for offset in [(-1, 0), (1, 0), (0 , -1), (0, 1)]:
            offset = Point(offset[0], offset[1])
            next:Point = cur + offset
            if (0<=next.y and next.y<200 and 0<=next.x and next.x<200
                and matrix[next.y][next.x] != 0
                and next not in came_from):
                frontier.put(next)
                came_from[next] = cur
                cost_matrix[next.y][next.x] = cost_matrix[cur.y][cur.x] + 1
                move_matrix[next.y][next.x] = -offset

    return move_matrix, cost_matrix
        


