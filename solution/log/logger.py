import logging

logger = logging.getLogger('info')
logger.setLevel(logging.INFO)

fh = logging.FileHandler("log_info.txt", mode="w+")
fh.setLevel(logging.INFO)
logger.addHandler(fh)



