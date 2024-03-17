import logging

RELEASE = False

null_handler = logging.NullHandler()

logger = logging.getLogger('info')
logger.setLevel(logging.INFO)

error_logger = logging.getLogger('error')
error_logger.setLevel(logging.ERROR)

if RELEASE == False:
    fh = logging.FileHandler("solution/log_info.txt", mode="w+")
    fh.setLevel(logging.INFO)
    logger.addHandler(fh)
    fh2 = logging.FileHandler("solution/log_error.txt", mode="w+")
    fh2.setLevel(logging.ERROR)
    error_logger.addHandler(fh2)
else:
    logger.addHandler(null_handler)
    error_logger.addHandler(null_handler)

import time
class My_Timer():
    def __init__(self) -> None:
        self.t = time.time()
    def click(self):
        return time.time() - self.t


