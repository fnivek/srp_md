""" Plan

Contains the planning utilities for srp-md

"""
import logging


class Planner(object):
    def __init__(self):
        self._logger = logging.getLogger(__name__)

    def plan(self):
        self._logger.debug('Starting to plan')
