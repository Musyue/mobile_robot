import sys
print sys.path.append("..")
from logger_config.logger_set import *
logger=LoggerSetClass(0)
logger.loggererror("haha")