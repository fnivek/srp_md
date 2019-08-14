# SRP-MD
This is a repo for semantic robot programing with multiple goal demonstrations.

## TODO
- [ ] Use libDAI to perform inference on a goal
  - [ ] Send an observation and learned factors to factor_graph_node
  - [ ] Send back scene graph representation
- [ ] Use libDAI to perform inference on an observation
- [ ] Learn factors from block world sensor (factor_graph_learn.py)
  - [ ] Frequentist
  - [ ] Research other methods (Scikit learn good starting point)
- [ ] Produce logically consistent scenes in block world sensor
- [ ] Make a PoseCNN sensor (or switch to a different object detector)
- [ ] Convert a goal and initial scene to a plan
- [ ] Use mobile manipulation to execute the plan
- [ ] Visualize factor graphs (could use graphviz or matplotlib)
- [ ] Consider [networkx](https://github.com/NetworkX/NetworkX) for use in the project
- [ ] Think about how to do evaluation code (if any is needed at all)
- [ ] TODO's in code:
  - [ ] CMakeLists.txt 46: # TODO(Kevin): Use a find_package or some other way to get these libraries
  - [ ] scene_graph.py 35: # TODO(?): Keep existing relations
  - [ ] goal_generator.py 20: TODO(Kevin)
- [ ] Add error handling to controller.py
- [ ] Update term_view.py to have all the features of pyqt_view.py
- [ ] Add a web interface

## Install
TODO (maybe never)

## Dependencies
* libDAI
  * libgmp-dev
  * cimg-dev
* PyQt5
* Python3
* ROS Melodic

## How to Run
`roslaunch srp_md srp_md.launch`

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
