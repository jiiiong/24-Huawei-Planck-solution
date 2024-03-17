from .point import Point
# hui 修改 新增一个mission任务类

class Mission():
    def __init__(self, pos: Point, car_id = -1, berth_id = -1):
        
        self.car_id = car_id
        self.berth_id = berth_id
        self.pos = pos
    
    def __repr__(self):
        return f"Mission(car_id={self.car_id}, berth_id={self.berth_id}, pos={self.pos})"
    
    def __eq__(self, other):
        if (isinstance(other, Point)):
            return self.pos == other
        
        return (self.car_id == other.car_id and
                self.berth_id == other.berth_id and
                self.pos == other.pos)
    
    def __hash__(self):
        return hash((self.pos))
        #return hash((self.car_id, self.berth_id, self.pos))