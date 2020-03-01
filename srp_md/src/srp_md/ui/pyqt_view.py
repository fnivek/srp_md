""" PyQt5 Viewer
"""
from __future__ import print_function
from __future__ import absolute_import
# SRP_MD imports
from . import view
from srp_md import learn
from srp_md import sense
from srp_md import act
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
import rospy
from sensor_msgs.msg import Image
import cv_bridge as bridge
import cv2
import py_trees, py_trees_ros


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

        script_path = os.path.dirname(os.path.realpath(__file__))
        self._data_folder = os.path.realpath(script_path + '/../../../../data')

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
        self._gui.guiLogFormatComboBox.addItems(['  --- Choose GUI Log Format Level---  '])
        self._gui.terminalLogFormatComboBox.addItems(['  --- Choose Terminal Log Format Level---  '])
        self._gui.fileLogFormatComboBox.addItems(['  --- Choose File Log Format Level---  '])

        self._gui.sensorComboBox.addItems(list(sense.sensors.keys()))
        self._gui.learnerComboBox.addItems(list(learn.learners.keys()))
        # self._gui.getDemoComboBox.addItems(list(sense.goal_types[self._model._sensor]))
        self._gui.goalGeneratorComboBox.addItems(list(srp_md.goal.goal_generators.keys()))

        # Set the combo boxes to the current value
        self._gui.sensorComboBox.setCurrentIndex(self._gui.sensorComboBox.findText(self._model.get_sensor()))
        self._gui.learnerComboBox.setCurrentIndex(self._gui.learnerComboBox.findText(self._model.get_learner()))
        self._gui.goalGeneratorComboBox.setCurrentIndex(
            self._gui.goalGeneratorComboBox.findText(self._model.get_goal_generator()))

        error_types = ['Critical', 'Error', 'Warning', 'Info', 'Debug']
        self._gui.guiLogFormatComboBox.addItems(error_types)
        self._gui.terminalLogFormatComboBox.addItems(error_types)
        self._gui.fileLogFormatComboBox.addItems(error_types)

        # Connect the buttons to actions
        self._gui.write_demos.triggered.connect(self.write_demos)
        self._gui.load_demos.triggered.connect(self.load_demos)
        self._gui.write_inits.triggered.connect(self.write_inits)
        self._gui.load_inits.triggered.connect(self.load_inits)
        self._gui.write_goals.triggered.connect(self.write_goals)
        self._gui.undo_demo.triggered.connect(self._ctrl.undo_demo)
        self._gui.redo_demo.triggered.connect(self._ctrl.redo_demo)
        self._gui.clear_demos.triggered.connect(self._ctrl.clear_demos)
        self._gui.clear_inits.triggered.connect(self._ctrl.clear_inits)
        self._gui.clear_goals.triggered.connect(self._ctrl.clear_goals)
        self._gui.get_demo.pressed.connect(self._ctrl.get_demo)
        self._gui.process_demos.pressed.connect(self._ctrl.process_data)
        self._gui.learn.pressed.connect(self._ctrl.learn)
        self._gui.get_init_scene.pressed.connect(self._ctrl.get_init_scene)
        self._gui.generate_goal.pressed.connect(self._ctrl.generate_goal)
        self._gui.evaluate_goal.pressed.connect(self._ctrl.evaluate_goal)
        self._gui.plan.pressed.connect(self._ctrl.plan)
        self._gui.act.pressed.connect(self._ctrl.act)
        self._gui.show_graph.pressed.connect(self._ctrl.show_graph)
        self._gui.grocery_experiment.pressed.connect(self.grocery_experiment)

        self._gui.sensorComboBox.currentIndexChanged.connect(self.update_sensor)
        self._gui.sensorComboBox.currentIndexChanged.connect(self.update_goal_type_list)

        self._gui.learnerComboBox.currentIndexChanged.connect(self.update_learner)
        self._gui.learnerComboBox.setContextMenuPolicy(Qt.CustomContextMenu)
        self._gui.learnerComboBox.customContextMenuRequested.connect(self.configure_learner)

        self._gui.getDemoComboBox.currentIndexChanged.connect(self.update_goal_type)
        self._gui.getDemoComboBox.setContextMenuPolicy(Qt.CustomContextMenu)
        self._gui.getDemoComboBox.customContextMenuRequested.connect(self.configure_demo_type)

        self._gui.goalGeneratorComboBox.currentIndexChanged.connect(self.update_goal_generator)

        self._gui.guiLogFormatComboBox.currentIndexChanged.connect(self.update_gui_log_format)
        self._gui.terminalLogFormatComboBox.currentIndexChanged.connect(self.update_terminal_log_format)
        self._gui.fileLogFormatComboBox.currentIndexChanged.connect(self.update_file_log_format)

        # Display the GUI
        dir(self._gui)
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
        self._gui.numDemos.setText('{}'.format(self._model.get_num_demos()))
        self._gui.numInits.setText('{}'.format(self._model.get_num_inits()))

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

    def text_level_to_logger_level(self, text):
        """

        input: text logger level ['Critical', 'Error', 'Warning', 'Info', 'Debug']
        output: Python logger level

        """
        level_map = {
            'Critical': logging.CRITICAL,
            'Error': logging.ERROR,
            'Warning': logging.WARNING,
            'Info': logging.INFO,
            'Debug': logging.DEBUG
        }
        return level_map[text]

    # get the logging handlers that aren't the QT handler
    def get_handlers(self):
        logger = logging.getLogger('srp_md')
        return [handler for handler in logger.handlers if handler != self._qt_handler]

    # Get the file handler
    def get_file_handler(self):
        handlers = self.get_handlers()
        fileHandlers = [handler for handler in handlers if type(handler) == logging.FileHandler]
        return fileHandlers[0]

    # Get the stream handler for the terminal
    def get_terminal_handler(self):
        handlers = self.get_handlers()
        streamHandlers = [handler for handler in handlers if type(handler) == logging.StreamHandler]
        return streamHandlers[0]

    def update_gui_log_format(self):
        level = self._gui.guiLogFormatComboBox.currentText()
        self._qt_handler.setLevel(logging.INFO)
        self._logger.info('Setting GUI format type to {}'.format(level))
        self._qt_handler.setLevel(self.text_level_to_logger_level(level))

    def update_terminal_log_format(self):
        level = self._gui.terminalLogFormatComboBox.currentText()
        terminal_handler = self.get_terminal_handler()
        terminal_handler.setLevel(logging.INFO)
        self._logger.info('Setting terminal format type to {}'.format(level))
        terminal_handler.setLevel(self.text_level_to_logger_level(level))

    def update_file_log_format(self):
        level = self._gui.fileLogFormatComboBox.currentText()
        file_handler = self.get_file_handler()
        file_handler.setLevel(logging.INFO)
        self._logger.info('Setting terminal format type to {}'.format(level))
        file_handler.setLevel(self.text_level_to_logger_level(level))

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
        demo_file = QFileDialog.getSaveFileName(self._gui, caption='Save File', directory=self._data_folder,
                                                filter='Demos (*.demo);;All files (*.*)')
        if demo_file[0] == '':
            pass
        else:
            self._ctrl.write_demos(demo_file[0])

    def load_demos(self):
        demo_file = QFileDialog.getOpenFileName(self._gui, caption='Open File', directory=self._data_folder,
                                                filter='Demos (*.demo);;All files (*.*)')
        if demo_file[0] == '':
            pass
        else:
            self._ctrl.load_demos(demo_file[0])

    def write_inits(self):
        init_file = QFileDialog.getSaveFileName(self._gui, caption='Save File', directory=self._data_folder,
                                                filter='Inits (*.init);;All files (*.*)')
        if init_file[0] == '':
            pass
        else:
            self._ctrl.write_inits(init_file[0])

    def load_inits(self):
        init_file = QFileDialog.getOpenFileName(self._gui, caption='Open File', directory=self._data_folder,
                                                filter='Inits (*.init);;All files (*.*)')
        if init_file[0] == '':
            pass
        else:
            self._ctrl.load_inits(init_file[0])

    def write_goals(self):
        goal_file = QFileDialog.getSaveFileName(self._gui, caption='Save File', directory=self._data_folder,
                                                filter='Goals (*.goal);;All files (*.*)')
        if goal_file[0] == '':
            pass
        else:
            self._ctrl.write_goals(goal_file[0])

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

    def grocery_experiment(self):
        # Use the video widget
        self._gui.vid = VidWidget(self._model, self._ctrl)
        self._gui.vid.run_main()

        # # Run the functionalities
        # self._ctrl.grocery_experiment()

    def run_once(self):
        self.update_from_model()

    def run(self):
        self.timer = QTimer(self._gui)
        self.timer.timeout.connect(self.run_once)
        self.timer.start(100)

class VidWidget(QWidget):
    def __init__(self, model, ctrl):
        super(VidWidget, self).__init__()
        self._model = model
        self._ctrl = ctrl
        self._demo_num = 0
        self._keyframe_num = 0

    @pyqtSlot(QImage)
    def setImage(self, image):
        self.video.setPixmap(QPixmap.fromImage(image))

    def run_main(self):
        # Import the video gui
        ui_path = os.path.dirname(os.path.abspath(__file__))
        loadUi(os.path.join(ui_path, "pyqt_format_vid.ui"), self)
        self.setWindowTitle("Video Capture")

        # create a label
        self.video.resize(910, 670)

        # Initialize video thread and connect the video
        self.vid_thread = VidThread()
        self.vid_thread.changePixmap.connect(self.setImage)
        self.vid_thread.start()

        # Initialize dope thread and run
        self.dope_thread = DopeThread()
        self.dope_thread.start()

        # Connect the buttons
        self.set_keyframe.pressed.connect(self.set_keyframe_func)
        self.undo_keyframe.pressed.connect(self.undo_keyframe_func)
        self.clear_keyframes.pressed.connect(self.clear_keyframes_func)
        self.next_demo.pressed.connect(self.next_demo_func)
        self.save_demos.pressed.connect(self.write_demos)
        self.load_demos.pressed.connect(self.load_demos_func)
        self.undo_demo.pressed.connect(self.undo_demo_func)
        self.get_init.pressed.connect(self._ctrl.get_init_scene)
        self.process_data.pressed.connect(self.process_keyframes)
        self.cancel.pressed.connect(self.clear_demos)

        self.show()
        self.run()

    def set_keyframe_func(self):
        # Adds current scene to keyframe
        self._ctrl.set_keyframe(self._demo_num, self._keyframe_num)
        self._keyframe_num += 1

    def clear_keyframes_func(self):
        # Clears keyframes for current demo
        self._ctrl.clear_keyframes(self._demo_num)
        self._keyframe_num = 0

    def undo_keyframe_func(self):
        # Clears keyframes for current demo
        self._ctrl.undo_keyframe()

    def next_demo_func(self):
        # Go to next demo
        self._demo_num += 1
        self._keyframe_num = 0

    def write_demos(self):
        # Saves all demos with keyframes
        script_path = os.path.dirname(os.path.realpath(__file__))
        self._data_folder = os.path.realpath(script_path + '/../../../../data')
        demo_file = QFileDialog.getSaveFileName(self, caption='Input Directory Name', directory=self._data_folder,
                                                filter='All files (*.*)')
        if demo_file[0] == '':
            pass
        else:
            self._ctrl.write_keyframes_demos(demo_file[0])

    def load_demos_func(self):
        # Load demos with keyframes
        script_path = os.path.dirname(os.path.realpath(__file__))
        self._data_folder = os.path.realpath(script_path + '/../../../../data')
        demo_file = QFileDialog.getSaveFileName(self, caption='Input Directory Name', directory=self._data_folder,
                                                filter='All files (*.*)')
        if demo_file[0] == '':
            pass
        else:
            self._ctrl.load_keyframes_demos(demo_file[0])
            self._demo_num = self._model.get_num_demos()

    def undo_demo_func(self):
        self._demo_num -= 1
        self._keyframe_num = self._model.get_num_keyframes()
        self._ctrl.undo_demo()

    def process_keyframes(self):
        # Go to next demo
        self.close()
        self._ctrl.process_keyframes()

    def clear_demos(self):
        # Clears the demos and closes the window
        self._ctrl.clear_demos()
        self.close()

    def closeEvent(self, event):
        self.vid_thread.exit()
        self.dope_thread.exit()
        event.accept()

    def run_once(self):
        self.update_from_model()

    def run(self):
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.run_once)
        self.timer.start(100)

    def update_from_model(self):
        self._keyframe_num = self._model.get_num_keyframes()
        self.demo_num.setText('{}'.format(self._model.get_num_demos()))
        self.keyframe_num.setText('{}'.format(self._model.get_num_keyframes()))

class VidThread(QThread):
    changePixmap = pyqtSignal(QImage)

    def __init__(self):
        super(VidThread, self).__init__()
        self.vid_sub = rospy.Subscriber("dope/rgb_points", Image, self.image_callback)
        self.image = None
        self.rate = rospy.Rate(10)

    def image_callback(self, data):
        self.image = data

    def run(self):
        while True:
            if self.image is not None:
                # Change the image to cv2 image
                br = bridge.CvBridge()
                cv_image = br.imgmsg_to_cv2(self.image)
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

                # Convert to QtFormat
                h, w, ch = cv_image.shape
                bytesPerLine = ch * w
                convertToQtFormat = QImage(cv_image.data, w, h, bytesPerLine, QImage.Format_RGB888)
                p = convertToQtFormat.scaled(910, 670, Qt.KeepAspectRatio)
                self.changePixmap.emit(p)
                self.rate.sleep()

class DopeThread(QThread):
    def __init__(self):
        super(DopeThread, self).__init__()
        self.root = py_trees.composites.Sequence(name='srp_md_dope')
        self.root.add_children([act.InfiniteDopeAct(name='act_infinite_dope')])
        self.tree = py_trees_ros.trees.BehaviourTree(self.root)
        self.tick_period = 2

    def run(self):
        self.tree.setup(timeout=5)
        self.tree.tick_tock(self.tick_period)

    def exit(self):
        self.tree.interrupt()
        self.tree.blackboard_exchange.unregister_services()
        super(DopeThread, self).exit()

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
