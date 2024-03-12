import logging

logger = logging.getLogger('test')
logger.setLevel(logging.INFO)

fh = logging.FileHandler("solution/log/log1.txt", mode="w+")
fh.setLevel(logging.INFO)
logger.addHandler(fh)