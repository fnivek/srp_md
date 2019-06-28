""" Terminal Viewer.

A class for interacting and viewing srp from the terminal.

"""
import view
from srp_md import learn
from srp_md import sense

import sys
import select


class TermView(view.BaseView):
    def __init__(self, model, ctrl):
        super(TermView, self).__init__(model, ctrl)
        self._state = 'print_menu'

    def update_from_model(self):
        pass

    def run(self):
        # Render
        self.print_menu()

        # Handle input
        self.handle_input()

    def print_menu(self):
        if self._state == 'print_menu':
            print """
Semantic Robot Programing with Multiple Demonstrations

Menu:
  1. Select learner
  2. Select sensor
  3. Learn
  4. Take snapshot

Input:
"""
            self._state = 'wait_for_input'

        elif self._state == 'select_learner':
            print 'options:', learn.learners.keys()
            self._state = 'wait_for_learner'

        elif self._state == 'select_sensor':
            print 'options:', sense.sensors.keys()
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
            print 'setting learner to', choice
            self._ctrl.set_learner(choice)
            self._state = 'print_menu'

        elif self._state == 'wait_for_sensor':
            print 'setting sensor to', choice
            self._ctrl.set_sensor(choice)
            self._state = 'print_menu'
