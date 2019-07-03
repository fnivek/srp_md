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


class TermView(view.BaseView):
    def __init__(self, model, ctrl):
        super(TermView, self).__init__(model, ctrl)
        self._logger = logging.getLogger(__name__)
        self._state = 'print_menu'
        self._main_menu = (
            'Semantic Robot Programing with Multiple Demonstrations\n\n'
            'Menu:\n'
            '  1. Select learner\n'
            '  2. Select sensor\n'
            '  3. Learn\n'
            '  4. Take snapshot\n\n'
            'Input:\n'
        )
        self._learners = {n: s for n, s in enumerate(learn.learners.keys())}
        self._sensors = {n: s for n, s in enumerate(sense.sensors.keys())}

    def update_from_model(self):
        pass

    def run(self):
        # Render
        self.print_menu()

        # Handle input
        self.handle_input()

    def print_menu(self):
        if self._state == 'print_menu':
            print self._main_menu
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

        # Hanle user input
        if self._state == 'wait_for_input':
            self._state = 'print_menu'
            if choice == '1':
                self._state = 'select_learner'
            elif choice == '2':
                self._state = 'select_sensor'
            elif choice == '3':
                print 'Learning...'
                self._ctrl.learn()
                print 'Learned'
            elif choice == '4':
                print 'Taking a snapshot'
                self._ctrl.snapshot()
                print 'Took snapshot'
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
