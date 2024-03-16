import logging

logger = logging.getLogger('info')
logger.setLevel(logging.INFO)

fh = logging.FileHandler("log_info.txt", mode="w+")
fh.setLevel(logging.INFO)
logger.addHandler(fh)

error_logger = logging.getLogger('error')
error_logger.setLevel(logging.ERROR)

fh2 = logging.FileHandler("log_error.txt", mode="w+")
fh2.setLevel(logging.ERROR)
error_logger.addHandler(fh2)

import time
class My_Timer():
    def __init__(self) -> None:
        self.t = time.time()
    def click(self):
        return time.time() - self.t


