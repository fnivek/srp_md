import logging as log
import os
import datetime

# Setup logging
file_loc = os.path.dirname(os.path.abspath(__file__))
now = str(datetime.datetime.now().strftime('%y%m%d'))
log_file = file_loc + '/../../../log/log_' + now + '.log'

file_format = log.Formatter(
    '%(asctime)s [%(module)s:%(funcName)s():%(lineno)d] [%(levelname)-5.5s] '
    '%(message)s'
)
stream_format = log.Formatter(
    '[%(module)s:%(funcName)s():%(lineno)d] [%(levelname)-5.5s] %(message)s'
)

logger = log.getLogger('srp_md')
logger.setLevel(log.DEBUG)

file_handler = log.FileHandler(log_file)
file_handler.setFormatter(file_format)
file_handler.setLevel(log.DEBUG)
logger.addHandler(file_handler)

stream_handler = log.StreamHandler()
stream_handler.setFormatter(stream_format)
stream_handler.setLevel(log.DEBUG)
logger.addHandler(stream_handler)

logger.debug('Logger Started')
