from path_planing.BFS import fileMap2Matrix, one_move_avoidance
from path_planing import Point
from typing import List
value_matrix = fileMap2Matrix("./maps/map2.txt")


avoidance_matrix_len = 5 # 7/2
pos = Point(55,5)

l = pos.x - avoidance_matrix_len
r = pos.x + avoidance_matrix_len
t = pos.y - avoidance_matrix_len
b = pos.y + avoidance_matrix_len

if l < 0: l = 0
if r > 199:  r = 199
if t < 0: t = 0
if b > 199: b = 199

print(l+1, t+1, r+1, b+1)

avoidance_matrix: List[List[int]] = []
col = -1
for y in range(t, b+1):
    avoidance_matrix.append([])
    col += 1
    for x in range(l, r+1):
        avoidance_matrix[col].append(value_matrix[y][x])
    print("".join([('.' if value == 1 else '*') for value in avoidance_matrix[col]]))

print("\n")
surrounding_robots = {0:Point(55,4), 1:Point(54,5)}

for robot_id in surrounding_robots:
    robot: Point = surrounding_robots[robot_id]
    poses = []
    # 当前位置
    poses.append(Point(x = robot.x, y = robot.y))
    # 下一步位置
    poses.append(Point(x = robot.x, y = robot.y+1))
    for _pos in poses:
        avoidance_matrix[_pos.y-t][_pos.x-l] = 0

avoidance_paths, success = one_move_avoidance(avoidance_matrix, 
                                              Point(pos.x-l,pos.y-t))

for y, line in enumerate(avoidance_matrix):
    line_str = []
    for x, item in enumerate(line):
        if (success and avoidance_paths[0] == Point(x,y)):
            line_str.append("$")
        else:
            line_str.append("." if item == 1 else "*")
        
            
    print("".join(line_str))