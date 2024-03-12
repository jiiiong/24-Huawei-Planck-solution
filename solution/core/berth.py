from solution.core.base import Point

class Berth:
    def __init__(self, x=0, y=0, transport_time=0, loading_speed=0):
        self.pos = Point(x = x, y = y)
        # self.x = x
        # self.y = y
        self.transport_time = transport_time
        self.loading_speed = loading_speed
        
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
