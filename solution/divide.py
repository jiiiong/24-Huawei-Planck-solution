from path_planing.BFS import fileMap2Matrix

value_matrix = fileMap2Matrix("/home/jiiiong/LinuxRelease/maps/map2.txt")

from path_planing.BFS import BFS_divide
from path_planing import Point

divide_matrix =  BFS_divide(value_matrix, [Point(138, 89), Point(138, 71),Point(64, 84) ])

from path_planing.utils import applyDivideMatrix2ChMap, saveMatrix2File


saveMatrix2File(applyDivideMatrix2ChMap(value_matrix, divide_matrix))