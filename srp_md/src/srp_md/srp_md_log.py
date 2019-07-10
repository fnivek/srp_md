import logging as log
import os
import datetime

# Constants defined for console coloring
LOG_COLORS = {
    'RESET': '\033[0m',
    'BOLD': '\033[01m',
    'DISABLE': '\033[02m',
    'UNDERLINE': '\033[04m',
    'REVERSE': '\033[07m',
    'STRIKE': '\033[09m',
    'INVISIBLE': '\033[08m',
    'BLACK': '\033[30m',
    'RED': '\033[31m',
    'GREEN': '\033[32m',
    'ORANGE': '\033[33m',
    'BLUE': '\033[34m',
    'PURPLE': '\033[35m',
    'CYAN': '\033[36m',
    'LIGHTGREY': '\033[37m',
    'DARKGREY': '\033[90m',
    'LIGHTRED': '\033[91m',
    'LIGHTGREEN': '\033[92m',
    'YELLOW': '\033[93m',
    'LIGHTBLUE': '\033[94m',
    'PINK': '\033[95m',
    'LIGHTCYAN': '\033[96m',
    'BG_BLACK': '\033[40m',
    'BG_RED': '\033[41m',
    'BG_GREEN': '\033[42m',
    'BG_ORANGE': '\033[43m',
    'BG_BLUE': '\033[44m',
    'BG_PURPLE': '\033[45m',
    'BG_CYAN': '\033[46m',
    'BG_LIGHTGREY': '\033[47m'
}

# Setup log file creation
file_loc = os.path.dirname(os.path.abspath(__file__))
now = str(datetime.datetime.now().strftime('%y%m%d'))
log_file = file_loc + '/../../../log/log_' + now + '.log'

# Custom formatter


class CustomFormat(log.Formatter):
    def __init__(self, fmt='[%(module)s:%(lineno)d] [%(levelname)s] %(message)s'):
        log.Formatter.__init__(self, fmt)

    def format(self, record):
        format_origin = self._fmt
        # Get the index of message
        msg_index = self._fmt.find('%(message)s')

        # Change color of message if indicated
        message = '%(message)s'
        if '#' in record.message:
            split_index = record.message.find('#')
            str_format = '{:.' + str(split_index) + '}'
            message = LOG_COLORS[record.message[split_index + 1:]] + \
                str_format.format(record.message) + LOG_COLORS['RESET']
        # Construct general message
        self._fmt = self._fmt[:msg_index] + LOG_COLORS['RESET'] + message

        # Change the color of prefix message depending on their level
        if record.levelno == log.DEBUG:
            self._fmt = LOG_COLORS['GREEN'] + self._fmt
        elif record.levelno == log.WARNING:
            self._fmt = LOG_COLORS['YELLOW'] + self._fmt
        elif record.levelno == log.ERROR:
            self._fmt = LOG_COLORS['ORANGE'] + self._fmt
        elif record.levelno == log.CRITICAL:
            self._fmt = LOG_COLORS['RED'] + self._fmt

        # Call the original formatter class to do the grunt work
        result = log.Formatter.format(self, record)
        self._fmt = format_origin

        return result


# Setup formats
file_format = log.Formatter(
    '%(asctime)s [%(module)s:%(funcName)s():%(lineno)d] [%(levelname)s] '
    '%(message)s'
)
stream_format = log.Formatter(
    '[%(module)s:%(lineno)d] [%(levelname)s] %(message)s'
)
stream_format = CustomFormat()

# Make loggers
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
