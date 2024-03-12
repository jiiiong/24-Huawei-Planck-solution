class Point():
    def __init__(self, x = 0, y = 0) -> None:
        self.x = x
        self.y = y

    def __add__(self, b):
        return Point(self.x + b.x, self.y + b.y)
    def __sub__(self, b):
        return Point(self.x - b.x, self.y - b.y)
    def __eq__(self, b):
        return True if (self.x == b.x and self.y == b.y) else False
    def __hash__(self) -> int:
        return hash((self.x, self.y))
    def __repr__(self) -> str:
        return "x = " + str(self.x) + ", y = " + str(self.y)
    def __neg__(self):
        return Point(-self.x, -self.y)
    def __lt__(self, b):
        return True