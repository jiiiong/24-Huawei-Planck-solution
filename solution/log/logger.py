import logging

RELEASE = False

null_handler = logging.NullHandler()

fmt = logging.Formatter('%(relativeCreated)d - %(name)s - %(levelname)s - %(message)s')

# root 用于保存所有日志中的ERROR
root_logger = logging.getLogger()

# main 用于记录整个程序的运行进度
main_logger = logging.getLogger('main')

# scheduler 用于记录调度器配置的各种信息已经调度过程
scheduler_logger = logging.getLogger('scheduler')

# robot 用于记录小车的运动状态变化
robot_logger = logging.getLogger('robot')

if RELEASE == False:
    fh_for_root_logger = logging.FileHandler('solution/log/root.log', mode="w+")
    fh_for_root_logger.setLevel(logging.ERROR)
    fh_for_root_logger.setFormatter(fmt)
    root_logger.addHandler(fh_for_root_logger)

    main_logger.setLevel(logging.INFO)
    fh_for_global_info_logger = logging.FileHandler("solution/log/main.log", mode="w+")
    fh_for_global_info_logger.setFormatter(fmt)
    main_logger.addHandler(fh_for_global_info_logger)

    scheduler_logger.setLevel(logging.INFO)
    fh_for_scheduler_logger = logging.FileHandler("solution/log/scheduler.log", mode="w+")
    fh_for_scheduler_logger.setFormatter(fmt)
    scheduler_logger.addHandler(fh_for_scheduler_logger)

    robot_logger.setLevel(logging.DEBUG)
    fh_for_robot_logger = logging.FileHandler("solution/log/robot.log", mode="w+")
    fh_for_robot_logger.setFormatter(fmt)
    robot_logger.addHandler(fh_for_robot_logger)
else:
    root_logger.addHandler(null_handler)
    main_logger.addHandler(null_handler)
    scheduler_logger.addHandler(null_handler)
    root_logger.addHandler(null_handler)


import time
class My_Timer():
    def __init__(self) -> None:
        self.t = time.time()
    def click(self):
        return time.time() - self.t


