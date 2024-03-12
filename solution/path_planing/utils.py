from pathlib import Path
from typing import List, Any, Iterable
from solution.core import Point

direction2Point = {
    "UP" :Point(0, -1),
    "DOWN" : Point(0, 1),
    "LEFT" : Point(-1, 0),
    "RIGHT" : Point(1, 0),
}

UP = Point(0, -1)
DOWN = Point(0, 1)
LEFT = Point(-1, 0)
RIGHT = Point(1, 0)

def evalLine(line: Iterable):
    newLine = []
    for c in line:
        if (c == '.'):
            val = 1
        elif (c == 'A'):
            val = 1
        elif (c == 'B'):
            val = 1
        elif (c == '#'):
            val = 0
        elif (c == '*'):
            val = 0
        elif (c == '\n'):
            break
        else: # syntax fix
            val = 0
        newLine.append(val)
    return newLine

def fileMap2Matrix(mapFile) -> List[List[int]]:
    with open(mapFile,"r") as fd:
        lines = fd.readlines()
        value_matrix = []
        for line in lines:
            value_matrix.append(evalLine(line))
        return value_matrix
    
def chMap2ValueMatrix(ch: List[List[str]]) -> List[List[int]]:
    matrix = []
    for line in ch:
        matrix.append(evalLine(line[0]))
    return matrix

def applyMoveMatrix2ChMap(ch: List[List[str]], move_matrix: List[List[Point]]) -> List[List[str]]:
    matrix = []
    for line in ch:
        new_line = []
        for c in line[0]:
            new_line.append(c)
        matrix.append(new_line)

    for y, line in enumerate(move_matrix):
        for x, c in enumerate(line):
            if (c == UP):
                matrix[y][x] = '↑'
            elif (c == DOWN):
                matrix[y][x] = '↓'
            elif (c == LEFT):
                matrix[y][x] = '←'
            elif (c == RIGHT):
                matrix[y][x] = '→'
    return matrix

def saveMatrix2File(matrix: List[List[Any]]) -> None:
    '''将矩阵[[]]存到文件中'''
    index = 0
    file_name = Path("solution/test/move_" + str(index))
    while file_name.exists():
        index += 1
        file_name = Path("solution/test/move_" + str(index))

    with open(file_name, "w") as fd:
        for y in range(len(matrix)):
            line = str().join(matrix[y])
            fd.write(line + "\n")