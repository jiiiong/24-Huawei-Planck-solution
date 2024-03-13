import logging

logger = logging.getLogger('info')
logger.setLevel(logging.INFO)

fh = logging.FileHandler("log_info.txt", mode="w+")
fh.setLevel(logging.INFO)
logger.addHandler(fh)

import time
class My_Timer():
    def __init__(self) -> None:
        self.t = time.time()
    def click(self):
        return time.time() - self.t


