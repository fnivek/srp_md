# SRP-MD
This is a repo for semantic robot programing with multiple goal demonstrations.

## Install
TODO (maybe never)

## How to Run
TODO

## Software Design
Design principles:
* Modularity - so that we can publish and reuse code pieces
    * Avoid dependency on ROS if possible
    * Keep functional units separate
    * Avoid dependencies between units
* Research code - we should always strive to write exceptional code, but this is research code not production code and does not need to live up to high standards
* We follow the ROS style guides for C++ and Python

The primary programing language will be python primarily because it will be quicker to develop the code base in python.
Other languages will be used as needed, e.g. C++ for PCL or Javascript for web interface.

Major components:
* Sense
* Plan
* Act
* Learn
* UI
* Eval
* Robot Interface
* Glue/Main

### Sense
This module estimates the state of the world.

### Plan
This module plans a set of actions to take to move the world from an initial state to a desired state.

### Act
This module encapsulates the behavior of the robot.

### Learn
This module learns a desired goal state from several demonstrations

### UI
This module provides methods for a human to interact with the software and robot.

### Eval
This module implements experiments and metrics to understand and benchmark the software.

### Robot Interface
This module translates software commands into real robot actions.

### Glue/Main
This module is the code that puts everything together in one place.
