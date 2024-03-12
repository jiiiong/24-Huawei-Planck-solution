import logging

logger = logging.getLogger('test')
logger.setLevel(logging.INFO)

fh = logging.FileHandler("log1.txt", mode="w+")
fh.setLevel(logging.INFO)
logger.addHandler(fh)