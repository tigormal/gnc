import logging

TRACE = 5

logging.basicConfig(
    # filename='log_file_name.log',
    level=logging.NOTSET, 
    # format= '[%(asctime)s] {%(pathname)s:%(lineno)d} %(levelname)s - %(message)s',
    format= '%(asctime)s [%(name)s] %(levelname)s - %(message)s',
    datefmt='%H:%M:%S'
)

logging.addLevelName(5, 'TRACE')

logger = logging.getLogger(__name__)

logger.log(logging.NOTSET, "hi, notset test")
logger.log(TRACE, "hi, trace test")

