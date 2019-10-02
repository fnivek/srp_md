""" PyQt5 Viewer
"""
from __future__ import print_function
from __future__ import absolute_import
# SRP_MD imports
from . import view
from srp_md import learn
from srp_md import sense
import srp_md.goal

# Python imports
import os
import logging
# from logging.handlers import QueueHandler
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.uic import *
from StringIO import StringIO


class PyQtView(view.BaseView):
    def __init__(self, model, ctrl):
        # Initialize the view class
        super(PyQtView, self).__init__(model, ctrl)
        self._logger = logging.getLogger(__name__)

        self._global_logger = logging.getLogger("srp_md")
        self._stream = StringIO()
        self._qt_handler = logging.StreamHandler(self._stream)
        self._qt_handler.setLevel(logging.DEBUG)
        self._qt_handler.setFormatter(HtmlFormat())
        self._global_logger.addHandler(self._qt_handler)

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
        self._gui.sensorComboBox.addItems(['  --- Choose Sensor ---  '])
        self._gui.learnerComboBox.addItems(['  --- Choose Learner ---  '])
        self._gui.getDemoComboBox.addItems(['  --- Choose Type ---  '])
        self._gui.goalGeneratorComboBox.addItems(['  --- Choose Generator ---  '])

        self._gui.sensorComboBox.addItems(list(sense.sensors.keys()))
        self._gui.learnerComboBox.addItems(list(learn.learners.keys()))
        # self._gui.getDemoComboBox.addItems(list(sense.goal_types[self._model._sensor]))
        self._gui.goalGeneratorComboBox.addItems(list(srp_md.goal.goal_generators.keys()))

        # Connect the buttons to actions
        self._gui.get_demo.pressed.connect(self._ctrl.generate_demo)
        self._gui.write_demos.triggered.connect(self.write_demos)
        self._gui.load_demos.triggered.connect(self.load_demos)
        self._gui.clear_demos.triggered.connect(self._ctrl.clear_demos)
        self._gui.undo_demo.triggered.connect(self._ctrl.undo_demo)
        self._gui.redo_demo.triggered.connect(self._ctrl.redo_demo)
        self._gui.learn.pressed.connect(self._ctrl.learn)
        self._gui.show_graph.pressed.connect(self._ctrl.show_graph)
        self._gui.generate_goal.pressed.connect(self._ctrl.generate_goal)
        self._gui.evaluate_goal.pressed.connect(self._ctrl.evaluate_goal)

        self._gui.sensorComboBox.currentIndexChanged.connect(self.update_sensor)
        self._gui.sensorComboBox.currentIndexChanged.connect(self.update_goal_type_list)

        self._gui.learnerComboBox.currentIndexChanged.connect(self.update_learner)
        self._gui.learnerComboBox.setContextMenuPolicy(Qt.CustomContextMenu)
        self._gui.learnerComboBox.customContextMenuRequested.connect(self.configure_learner)

        self._gui.getDemoComboBox.currentIndexChanged.connect(self.update_goal_type)
        self._gui.getDemoComboBox.setContextMenuPolicy(Qt.CustomContextMenu)
        self._gui.getDemoComboBox.customContextMenuRequested.connect(self.configure_demo_type)

        self._gui.goalGeneratorComboBox.currentIndexChanged.connect(self.update_goal_generator)

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

        # Get the errors
        errors = self._ctrl.pop_errors()
        for error in errors:
            QMessageBox.question(self._gui, 'Errors', error, QMessageBox.Ok)

        # Print to QTextEdit in GUI
        self._qt_handler.flush()
        if self._stream.len != 0:
            new_text = self._stream.getvalue()
            self._stream.truncate(0)
            for line in new_text.splitlines():
                self._gui.loggerText.append(line)
        self._gui.numLabel.setText('{}'.format(self._model.get_num_demos()))

    def update_sensor(self):
        if self._gui.sensorComboBox.currentText()[0] == " ":
            self._ctrl.set_sensor(None)
        else:
            self._ctrl.set_sensor(self._gui.sensorComboBox.currentText())

    def update_goal_type_list(self):
        self._gui.getDemoComboBox.setCurrentIndex(0)
        for item in range(self._gui.getDemoComboBox.count() - 1):
            self._gui.getDemoComboBox.removeItem(1)
        if self._model._sensor_name in sense.goal_types.keys():
            self._gui.getDemoComboBox.addItems(list(sense.goal_types[self._model._sensor_name]))
        else:
            pass

    def update_learner(self):
        if self._gui.learnerComboBox.currentText()[0] == " ":
            self._ctrl.set_learner(None)
        else:
            self._ctrl.set_learner(self._gui.learnerComboBox.currentText())

    def update_goal_type(self):
        if self._gui.getDemoComboBox.currentText()[0] == " ":
            self._ctrl.update_sensor_config(goal_type='random')
        else:
            self._logger.debug('Setting goal type to {}'.format(self._gui.getDemoComboBox.currentText()))
            self._ctrl.update_sensor_config(goal_type=self._gui.getDemoComboBox.currentText())

    def update_goal_generator(self):
        if self._gui.goalGeneratorComboBox.currentText()[0] == " ":
            self._ctrl.set_goal_generator(None)
        else:
            self._ctrl.set_goal_generator(self._gui.goalGeneratorComboBox.currentText())

    def write_demos(self):
        demo_file = QFileDialog.getSaveFileName(self._gui, caption='Save File',
                                                filter='Demos (*.demo);;All files (*.*)')
        if demo_file[0] == '':
            pass
        else:
            self._ctrl.write_demos(demo_file[0])

    def load_demos(self):
        demo_file = QFileDialog.getOpenFileName(self._gui, caption='Open File',
                                                filter='Demos (*.demo);;All files (*.*)')
        if demo_file[0] == '':
            pass
        else:
            self._ctrl.load_demos(demo_file[0])

    def configure_learner(self, pos):
        # Currently only factor_graph_learner is configurable
        if self._gui.learnerComboBox.currentText() != 'factor_graph_learner':
            return

        main_menu = QMenu('Configure {}'.format(self._gui.learnerComboBox.currentText()))
        config_menu = QMenu('Factor Learner')
        actions = {config_menu.addAction(name): name for name, cls in learn.FACTOR_LEARNERS.iteritems()}
        main_menu.addMenu(config_menu)
        action = main_menu.exec_(self._gui.learnerComboBox.mapToGlobal(pos))
        if action in actions:
            self._logger.debug('Setting factor learner to {}'.format(actions[action]))
            self._ctrl.update_learner_config(factor_learner=actions[action])

    def configure_demo_type(self, pos):
        # Currently only factor_graph_learner is configurable
        main_menu = QMenu('Configure {}'.format(self._gui.getDemoComboBox.currentText()))
        config_menu = QMenu('Demo Type')
        actions = {config_menu.addAction(name): name for name in self._model.demo_types}
        main_menu.addMenu(config_menu)
        action = main_menu.exec_(self._gui.getDemoComboBox.mapToGlobal(pos))
        if self._model._sensor is None:
            self._logger.error('Please select sensor!')
        elif action in actions:
            self._logger.debug('Setting demo type to {}'.format(actions[action]))
            self._ctrl.update_sensor_config(demo_type=actions[action])

    def run_once(self):
        self.update_from_model()

    def run(self):
        self.timer = QTimer(self._gui)
        self.timer.timeout.connect(self.run_once)
        self.timer.start(100)


class HtmlFormat(logging.Formatter):
    def __init__(self, fmt='[%(module)s:%(lineno)d] [%(levelname)s] %(message)s'):
        logging.Formatter.__init__(self, fmt)
        bold_font = '<span style=\" font-weight:bold;\" >'
        color_font = '<span style=\" color:#{};\" >'
        plain_font = '<span style=\" color:#969696;\" >'
        end = '</span>'
        # TODO(Henry): Fill in more fonts and ensure it matches up to LOG_COLORS
        self._log_colors = {
            'RESET': end,
            'BOLD': bold_font,
            'DISABLE': plain_font,
            'UNDERLINE': plain_font,
            'REVERSE': plain_font,
            'STRIKE': plain_font,
            'INVISIBLE': plain_font,
            'BLACK': plain_font,
            'WHITE': color_font.format('b9b9b9'),
            'RED': color_font.format('f22e2e'),
            'GREEN': color_font.format('32a852'),
            'ORANGE': color_font.format('ff5a0d'),
            'BLUE': color_font.format('0d25ff'),
            'PURPLE': plain_font,
            'CYAN': plain_font,
            'LIGHTGREY': plain_font,
            'DARKGREY': plain_font,
            'LIGHTRED': plain_font,
            'LIGHTGREEN': plain_font,
            'YELLOW': color_font.format('fcd303'),
            'LIGHTBLUE': plain_font,
            'PINK': plain_font,
            'LIGHTCYAN': plain_font,
            'BG_BLACK': plain_font,
            'BG_RED': plain_font,
            'BG_GREEN': plain_font,
            'BG_ORANGE': plain_font,
            'BG_BLUE': plain_font,
            'BG_PURPLE': plain_font,
            'BG_CYAN': plain_font,
            'BG_LIGHTGREY': plain_font,
        }

    def format(self, record):
        format_origin = self._fmt
        # Get the index of message
        msg_index = self._fmt.find('%(message)s')

        # Change color of message if indicated
        message = '%(message)s'
        if '#' in record.message:
            split_index = record.message.find('#')
            str_format = '{:.' + str(split_index) + '}'
            message = self._log_colors[record.message[split_index + 1:]] + \
                str_format.format(record.message) + self._log_colors['RESET']
        # Construct general message
        self._fmt = self._fmt[:msg_index] + self._log_colors['RESET'] + message + '\r'

        # Change the color of prefix message depending on their level
        if record.levelno == logging.DEBUG:
            self._fmt = self._log_colors['GREEN'] + self._fmt
        elif record.levelno == logging.INFO:
            self._fmt = self._log_colors['WHITE'] + self._fmt
        elif record.levelno == logging.WARNING:
            self._fmt = self._log_colors['YELLOW'] + self._fmt
        elif record.levelno == logging.ERROR:
            self._fmt = self._log_colors['ORANGE'] + self._fmt
        elif record.levelno == logging.CRITICAL:
            self._fmt = self._log_colors['RED'] + self._fmt

        # Call the original formatter class to do the grunt work
        result = logging.Formatter.format(self, record)
        self._fmt = format_origin

        return result
