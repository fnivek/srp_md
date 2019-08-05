#!/usr/bin/env python

""" PyQt5 Viewer
"""
from __future__ import print_function
from __future__ import absolute_import
# SRP_MD imports
from . import view
from srp_md import learn
from srp_md import sense

# Python imports
import os
import logging
from logging.handlers import QueueHandler
import queue
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.uic import *


class PyQtView(view.BaseView):
    def __init__(self, model, ctrl):
        # Initialize the view class
        super(PyQtView, self).__init__(model, ctrl)
        self._logger = logging.getLogger(__name__)

        self._global_logger = logging.getLogger("srp_md")
        self.queue = queue.Queue(-1)
        queue_handler = QueueHandler(self.queue)
        queue_handler.setLevel(logging.DEBUG)
        self._global_logger.addHandler(queue_handler)

        # Initialize the GUI
        self.ImportGUI()

        self._state = 'idle'
        self._learners = {n: s for n, s in enumerate(learn.learners.keys())}
        self._sensors = {n: s for n, s in enumerate(sense.sensors.keys())}
        self.run()

    def ImportGUI(self):
        # Import pre-defined GUI layout from pyqt_format.ui
        self._gui = QMainWindow()
        ui_path = os.path.dirname(os.path.abspath(__file__))
        loadUi(os.path.join(ui_path, "pyqt_format.ui"), self._gui)

        # Set up displays on GUI
        self._gui.setWindowTitle("Semantic Robot Programing with Multiple Demonstrations")
        self._gui.sensorComboBox.addItems(list(sense.sensors.keys()))
        self._gui.learnerComboBox.addItems(list(learn.learners.keys()))

        # Set initial values for comboboxes
        self._ctrl.set_sensor(self._gui.sensorComboBox.currentText())
        self._ctrl.set_learner(self._gui.learnerComboBox.currentText())

        # Connect the buttons to actions
        self._gui.get_demo.pressed.connect(self._ctrl.generate_demo)
        self._gui.write_demos.pressed.connect(self._ctrl.write_demos)
        self._gui.load_demos.pressed.connect(self._ctrl.load_demos)
        self._gui.clear_demos.pressed.connect(self._ctrl.clear_demos)
        self._gui.undo_demo.pressed.connect(self._ctrl.undo_demo)
        self._gui.redo_demo.pressed.connect(self._ctrl.redo_demo)
        self._gui.learnButton.pressed.connect(self._ctrl.learn)
        self._gui.sensorComboBox.currentIndexChanged.connect(self.update_sensor)
        self._gui.learnerComboBox.currentIndexChanged.connect(self.update_learner)

        # Display the GUI
        self._gui.show()
        # self._gui.resize(self._gui.minimumSizeHint())

    def update_from_model(self):
        """ Update GUI and update model from GUI
        """

        # Update model from GUI
        self._srp_state = [self._model.get_learner(),
                           self._model.get_sensor(),
                           self._model.get_num_demos()]

        # Print to QTextEdit in GUI
        while not self.queue.empty():
            record = self.queue.get()
            message = record.message
            if '#' in record.message:
                split_index = record.message.find('#')
                message = record.message[:split_index]
            log_message = '[{}:{}] [{}] {}'.format(record.module, record.lineno, record.levelname, message)
            self._gui.loggerText.append(log_message)

    def update_sensor(self):
        self._ctrl.set_sensor(self._gui.sensorComboBox.currentText())

    def update_learner(self):
        self._ctrl.set_learner(self._gui.learnerComboBox.currentText())

    def run_once(self):
        self.update_from_model()

    def run(self):
        self.timer = QTimer(self._gui)
        self.timer.timeout.connect(self.run_once)
        self.timer.start(100)
