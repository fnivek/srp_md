# SRP-MD
This is a repo for semantic robot programing with multiple goal demonstrations.

## Software Design
Design principles:
* Modularity - so that I can publish code pieces

Major components:
* Perception
* Planning
* Acting
* Learning
* User Interface
* Evaluation
* Robot Interface

### Perception
This module estimates the state of the world.

### Planning
This module plans a set of actions to take to move the world from an initial state to a desired state.

### Acting
This module encapsulates the behavior of the robot.

### Learning
This module learns a desired goal state from several demonstrations

### User Interface
This module provides methods for a human to interact with the software and robot.

### Evaluation
This module implements experiments and metrics to understand and benchmark the software.

### Robot Interface
This module translates software commands into real robot actions
