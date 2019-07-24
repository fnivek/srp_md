""" Terminal Viewer.

A class for interacting and viewing srp from the terminal.

"""
# SRP_MD imports
import view
from srp_md import learn
from srp_md import sense

# Python imports
import sys
import select
import logging
import time


class TermView(view.BaseView):
    def __init__(self, model, ctrl):
        super(TermView, self).__init__(model, ctrl)
        self._logger = logging.getLogger(__name__)
        self._state = 'print_menu'
        self._main_menu = (
            'Semantic Robot Programing with Multiple Demonstrations\n\n'
            'Current State:\n'
            '  Learner: {}\n'
            '  Sensor: {}\n'
            '  Number of demos: {}\n\n'
            'Menu:\n'
            '  0. Exit\n'
            '  1. Select learner\n'
            '  2. Select sensor\n'
            '  3. Learn\n'
            '  4. Take demo\n'
            '  5. Actions\n\n'
            'Input:\n'
        )
        self._learners = {n: s for n, s in enumerate(learn.learners.keys())}
        self._sensors = {n: s for n, s in enumerate(sense.sensors.keys())}
        self._filename = './demos/test.txt'
        self.running = True

    def update_from_model(self):
        self._srp_state = [self._model.get_learner(),
                           self._model.get_sensor(),
                           self._model.get_num_demos()]

    def run_once(self):
        # Update
        self.update_from_model()
        # Render
        self.print_menu()
        # Handle input
        self.handle_input()

    def run(self):
        self.run_once()
        time.sleep(0.01)

    def print_menu(self):
        if self._state == 'print_menu':
            print self._main_menu.format(*self._srp_state)
            self._state = 'wait_for_input'

        elif self._state == 'select_learner':
            print 'options:'
            for learner in self._learners:
                print '\t{}\t{}'.format(learner, self._learners[learner])
            self._state = 'wait_for_learner'

        elif self._state == 'select_sensor':
            print 'options:'
            for sensor in self._sensors:
                print '\t{}\t{}'.format(sensor, self._sensors[sensor])
            self._state = 'wait_for_sensor'

        elif self._state == 'select_action':
            print 'options:'
            for action in self._model._actions:
                print '\t{}\t{}'.format(action, self._model._actions[action])
            self._state = 'wait_for_action'

    def handle_input(self):
        # Get user input non-blocking
        choice = None
        istream = select.select([sys.stdin], [], [], 0)[0]
        for s in istream:
            line = s.readline().rstrip()
            if line:
                choice = line
                break
        if choice is None:
            return

        # Handle user input
        if self._state == 'wait_for_input':
            self._state = 'print_menu'
            if choice == '0':
                self.running = False
                self._state = 'end'
            elif choice == '1':
                self._state = 'select_learner'
            elif choice == '2':
                self._state = 'select_sensor'
            elif choice == '3':
                print 'Learning...'
                self._ctrl.learn()
                print 'Learned'
            elif choice == '4':
                print 'Generate a demo'
                self._ctrl.generate_demo()
                print 'Got demo'
            elif choice == '5':
                self._state = 'select_action'
            else:
                print 'Invalid choice'

        elif self._state == 'wait_for_learner':
            choice = int(choice)
            name = self._learners[int(choice)]
            self._logger.info('setting learner to %s', name)
            self._ctrl.set_learner(self._learners[choice])
            self._state = 'print_menu'

        elif self._state == 'wait_for_sensor':
            choice = int(choice)
            name = self._sensors[int(choice)]
            self._logger.info('setting sensor to %s', name)
            self._ctrl.set_sensor(self._sensors[choice])
            self._state = 'print_menu'

        elif self._state == 'wait_for_action':
            choice = int(choice)
            name = self._model._actions[int(choice)]
            self._logger.info('choosing action: %s', name)
            if name == 'write_demos':
                self._ctrl.write_demos(self._filename)
            elif name == 'load_demos':
                self._ctrl.load_demos(self._filename)
            elif name == 'undo_demo':
                self._ctrl.undo_demo()
            elif name == 'redo_demo':
                self._ctrl.redo_demo()
            elif name == 'clear_demos':
                self._ctrl.clear_demos()
            self._state = 'print_menu'
