import time
from path_planing.BFS import BFS_file
from core.base import Point
from path_planing.utils import fileMap2Matrix

t = time.time()
mapFile = "/home/jiiiong/LinuxRelease/maps/map1.txt"
source_point = Point(173, 36)
move_matrix, cost_matrix = BFS_file(mapFile, source_point)
t = (time.time() - t) * 1000

matrix = fileMap2Matrix(mapFile)
up = Point(0, -1)
down = Point(0, 1)
left = Point(-1, 0)
right = Point(1, 0)

with open("solution/test/cost map", "w+") as fd:
    output_matrix = []
    original = open(mapFile, "r")

    lines = original.readlines()
    for y in range(200):
        line = lines[y]
        output_matrix.append([])
        for (x, placeholder) in enumerate(line):
            if (x >= 200):
                continue
            output_matrix[y].append(cost_matrix[y][x])
            continue
            char = move_matrix[y][x]
            if (char == up):
                output_matrix[y].append('↑')
            elif (char == down):
                output_matrix[y].append('↓')
            elif (char == left):
                output_matrix[y].append('←')
            elif (char == right):
                output_matrix[y].append('→')
            else:
                output_matrix[y].append(placeholder)
        fd.write(str(output_matrix[y]) + "\n")

    print('running time: ', t, file=fd)
