""" Semantic Robot Programing Multiple Demonstrations.

Defines a class that encapsulates all functionality.

"""
from __future__ import absolute_import
from __future__ import print_function
from builtins import str
from builtins import object
from . import learn
from . import sense
from . import goal
from . import evaluate
from . import plan
from . import act

# Python imports
import logging as log
import pickle
import networkx as nx
import matplotlib.pyplot as plt
import rospy
import message_filters
from sensor_msgs.msg import CameraInfo, Image as ImageSensor_msg, PointCloud2


class SrpMd(object):
    """ Semantic Robot Programing Multiple Demonstrations.

    Implements a semantic robot programing with multiple demonstrations. The
    algorithms can be configured with the set_<x> functions. See the associated
    python package for avaliable algorithms.

    """
    # TODO(Kevin): Set defaults when they exist
    def __init__(self, learner=None, sensor=None, goal_generator=None,
                 goal_evaluator='adapt_goal_evaluator'):
        # Logging
        self._logger = log.getLogger(__name__)

        # Vars
        self._raw_images = []
        self._current_image = None
        self._demo_graphs = []
        self._initial_scenes = []
        self._current_scene = None
        self._initial_graphs = []
        self._goal = None
        self._goal_instances = []
        self._new_image = None
        self._new_pcd = None

        self._actions = {1: 'Write demos', 2: 'Load demos', 3: 'Undo demo',
                         4: 'Redo demo', 5: 'Clear demos'}
        self._undoed = []
        self._factors = None
        self.demo_types = ["only_goal", "only_not_goal", "random"]
        self._sense_category = {'fake': ['fake_sensor'],
                                'version': ['example_sensor', 'can_tower_sensor'],
                                'factor': ['posecnn_sensor', 'block_world_sensor', 'pen_world_sensor',
                                           'book_world_sensor', 'abstract_world_sensor']}
        self._solution_filename = None

        # Set the default srp_md strategies
        self.set_learner(learner)
        self.set_sensor(sensor)
        self.set_goal_generator(goal_generator)
        self.set_goal_evaluator(goal_evaluator)
        self._planner = plan.Planner()
        self._actor = act.Actor()

        # Make subscribers for image
        self._image_sub = message_filters.Subscriber('/head_camera/rgb/image_raw', ImageSensor_msg)
        self._info_sub = message_filters.Subscriber('/head_camera/rgb/camera_info', CameraInfo)
        self._points_sub = message_filters.Subscriber('/head_camera/depth/points', PointCloud2)
        self._ts = message_filters.TimeSynchronizer([self._image_sub, self._info_sub], 100)
        self._ts.registerCallback(self.image_callback)
        self._points_sub.registerCallback(self.points_callback)

    def get_num_demos(self):
        return len(self._raw_images)

    def get_num_inits(self):
        return len(self._initial_scenes)

    """ Learner.

    Functions to interact with the learner.

    """
    def get_learner(self):
        return self._learner_name

    def set_learner(self, learner):
        self._learner_name = learner
        if self._learner_name is None:
            self._learner = None
        else:
            self._learner = learn.learners[learner]()

    def update_learner_config(self, **kwargs):
        self._learner.update_config(**kwargs)

    def learn(self):
        if self._sensor is None:
            self._logger.error('Please select sensor!')
        if self._learner is None:
            self._logger.error('Please select learner!')
        else:
            self._logger.debug('Learning...')
            if None in self._demo_graphs or len(self._demo_graphs) == 0:
                self._logger.warning('No processed data to be learned from!')
            elif self.get_learner() == 'factor_graph_learner':
                self._factors = self._learner.learn(self._demo_graphs, self._sensor.properties)
                self._logger.debug('Factors learned: %s', self._factors.keys())
            else:
                self._goal = self._learner.learn(self._demo_graphs)
                self._logger.debug('Learned: %s', self._goal)

    """ Sensor.

    Functions to interact with the sensor.

    """
    def get_sensor(self):
        return self._sensor_name

    def set_sensor(self, sensor):
        self._sensor_name = sensor
        if self._sensor_name is None:
            self._sensor = None
        else:
            self._sensor = sense.sensors[sensor]()

    def update_sensor_config(self, **kwargs):
        self._sensor.update_config(**kwargs)

    def accept_data(self, tag):
        """
        Accepts images and saves them with tagged types
        """
        # State the tag type
        self._logger.debug('Accepted {}'.format(tag))

        # Initialize new image and point cloud
        self._new_image = None
        self._new_pcd = None

        # Wait for message with timeout
        start = rospy.get_rostime()
        timeout = rospy.Duration(15) # Wait for x seconds?
        rate = rospy.Rate(100)
        while ((self._new_image is None or self._new_pcd is None) and
                (not rospy.is_shutdown()) and (rospy.get_rostime() - start < timeout)):
            rate.sleep()

        # If no image was got from listener, then log error
        if self._new_image is None or len(self._new_image.keys()) == 0:
            self._logger.error('Failed to get an image within {}s'.format(timeout.to_sec()))
        elif self._new_pcd is None:
            self._logger.error('Failed to get the pcd within {}s'.format(timeout.to_sec()))
        else:
            self._new_image["points"] = self._new_pcd
            # If this image was a demonstration, add to list of demonstrations
            if tag == "demonstration":
                self._raw_images.append(self._new_image)
                self._current_image = self._new_image
            # If this image was an initial scene, add to list of initial scenes
            elif tag == "initial scene":
                self._initial_scenes.append(self._new_image)
                self._current_scene = self._new_image
            # If the tag was wrong, log error
            else:
                self._logger.error('Wrong tag: {}'.format(tag))

    def image_callback(self, image, info):
        # Save the image and info got into new image dictionary
        self._new_image = {}
        self._new_image["image"] = image
        self._new_image["info"] = info

    def points_callback(self, points):
        # Save the image and info got into new image dictionary
        self._new_pcd = points

    def process_data(self):
        # If sensor is not set, log error
        if self._sensor is None:
            self._logger.error('Please select sensor!')
        # If the initial scene was not given, process data for all currently saved demonstrations
        self._logger.debug('Processing demonstrations...')
        for i in range(len(self._raw_images)):
            new_graph = self._sensor.process_data(self._raw_images[i])
            self._demo_graphs.append(new_graph)
        self._logger.debug('The demonstrations are: {}'.format(self._demo_graphs))
        # If the initial scene was given, process data for that scene
        self._logger.debug('Processing initial scene...')
        for i in range(len(self._initial_scenes)):
            new_graph = self._sensor.process_data(self._initial_scenes[i])
            self._initial_graphs.append(new_graph)
        self._logger.debug('The initial scenes are: {}'.format(self._initial_graphs))

    """ Goal Generator.

    Functions to interact with the goal generator.

    """
    def get_goal_generator(self):
        return self._goal_generator_name

    def set_goal_generator(self, goal_generator):
        self._goal_generator_name = goal_generator
        if self._goal_generator_name is None:
            self._goal_generator = None
        else:
            self._goal_generator = goal.goal_generators[goal_generator]()

    def generate_goal(self):
        # If goal generator is not set, log error
        if self._goal_generator is None:
            self._logger.error('Please select goal generator!')
        else:
            # If using factor graph learner, do:
            if self.get_learner() == 'factor_graph_learner':
                # If initial scene was not set, log error
                if len(self._initial_scenes) == 0:
                    self._logger.error('No scene detected!\n')
                    self._goal_instances = []
                # If initial scene was seen, get goal instance
                else:
                    self._logger.info('Generating goal scenes from initial scenes...\n')
                    for i in range(len(self._initial_graphs)):
                        goal_instance = self._goal_generator.generate_goal(self._factors, self._initial_graphs[i])
                        self._goal_instances.append(goal_instance)
                    self._logger.debug('The goal scenes are: {}'.format(self._goal_instances))
            # If not using factor graph generator, just generate goal without inputs
            else:
                self._goal_instances = self._goal_generator.generate_goal()

    def update_goal_generator_config(self, **kwargs):
        self._goal_generator.update_config(**kwargs)

    """ Goal Evaluator.

    Functions to evaluate generated goal.

    """
    def set_goal_evaluator(self, goal_evaluator):
        self._goal_evaluator_name = goal_evaluator
        if self._goal_evaluator_name is None:
            self._goal_evaluator = None
        else:
            self._goal_evaluator = evaluate.goal_evaluators[goal_evaluator]()

    def evaluate_goal(self):
        evaluations = []
        if self._goal_evaluator is None:
            self._logger.error('Please select goal evaluator!')
        elif self._sensor is None:
            self._logger.error('Please select sensor!')
        elif self._goal_instances is None:
            self._logger.error('Please generate goal instance to evaluate!')
        else:
            for i in range(len(self._goal_instances)):
                evaluation = self._goal_evaluator.evaluate_goal(self._sensor, self.get_sensor(), self._goal_instances[i])
                evaluations.append(evaluation)
        return evaluations

    """ Actions.

    Functions to execute actions.

    """
    def write_demos(self, filename="./demos/raw_images.txt"):
        pickle.dump(self._raw_images, open(filename, 'wb'))
        self._logger.info('Success writing demo images to file {}\n'.format(filename))

    def load_demos(self, filename="./demos/raw_images.txt"):
        self._raw_images = pickle.load(open(filename, 'rb'))
        self._logger.info('Success loading demo images from file {}\n'.format(filename))
        if len(self._raw_images) != 0:
            self._current_image = self._raw_images[-1]
        else:
            self._current_image = None

    def write_inits(self, filename="./inits/initial_scenes.txt"):
        pickle.dump(self._initial_scenes, open(filename, 'wb'))
        self._logger.info('Success writing initial scene images to file {}\n'.format(filename))

    def load_inits(self, filename="./inits/initial_scenes.txt"):
        self._initial_scenes = pickle.load(open(filename, 'rb'))
        self._logger.info('Success loading initial scene images from file {}\n'.format(filename))
        if len(self._initial_scenes) != 0:
            self._current_scene = self._initial_scenes[-1]
        else:
            self._current_scene = None

    def write_goals(self, filename="./goals/goal_instances_paired.txt"):
        paired = zip(self._initial_graphs, self._goal_instances)
        pickle.dump(paired, open(filename, 'wb'))
        self._logger.info('Success writing paired goal instances with initial scene graphs to file {}\n'.format(filename))

    def undo_demo(self):
        if len(self._raw_images) == 0:
            self._logger.info('No demos to undo!\n')
        else:
            last_demo = self._raw_images.pop()
            self._undoed.append(last_demo)
            self._logger.info('Success undoing last demo\n')
        if self._sensor_name in self._sense_category['factor']:
            if len(self._raw_images) != 0:
                self._current_image = self._raw_images[-1]
            else:
                self._current_image = None

    def redo_demo(self):
        if len(self._undoed) == 0:
            self._logger.info('No undoed demos to redo!\n')
        else:
            last_demo = self._undoed.pop()
            self._raw_images.append(last_demo)
            self._logger.info('Success redoing last demo\n')
        if self._sensor_name in self._sense_category['factor']:
            self._current_image = self._raw_images[-1]

    def clear_demos(self):
        self._raw_images = []
        self._demo_graphs = []
        self._logger.info('Success clearing demos\n')
        self._current_image = None

    def clear_inits(self):
        self._initial_scenes = []
        self._initial_graphs = []
        self._logger.info('Success clearing initial scenes\n')
        self._current_scene = None

    def clear_goals(self):
        self._goal_instances = []
        self._logger.info('Success clearing goals\n')

    def show_graph(self):
        # If the scene graph is None, print warning
        if len(self._demo_graphs) is None:
            self._logger.warning('No scene graph to show!\n')
        # If scene graph exists, do:
        else:
            scene_graph = self._demo_graphs[-1]
            # Initialize graph object
            G = nx.DiGraph()

            # Add the objects as graph nodes
            obj_names = scene_graph.get_obj_names()
            node_labels = {obj_names[i]: r"$" + scene_graph.get_obj_names()[i] + "$"
                           for i in range(scene_graph.num_objs())}
            G.add_nodes_from(obj_names)

            # Add the relations
            edge_labels = {}
            for relation in scene_graph.relations:
                if relation.value != "disjoint":
                    object_ids = relation.get_objs()
                    e = tuple(["X_" + str(object_ids[0]), "X_" + str(object_ids[1])])
                    edge_labels[e] = relation.value
                    G.add_edge(*e)

            # Get the layout of nodes
            pos = nx.spring_layout(G)

            # Draw the graph
            nx.draw_networkx_nodes(G, pos, node_color=scene_graph.get_prop_values("color"),
                                   node_size=500)
            nx.draw_networkx_edges(G, pos, width=1.0, alpha=0.5)
            nx.draw_networkx_labels(G, pos, node_labels, font_size=16)
            nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)

            # Show the plotted graph
            self._logger.info('Showing the scene graph...\n')
            plt.show()

            # G = nx.DiGraph()
            # G.add_edges_from(
            #     [('A', 'B'), ('A', 'C'), ('D', 'B'), ('E', 'C'), ('E', 'F'),
            #      ('B', 'H'), ('B', 'G'), ('B', 'F'), ('C', 'G')])

            # val_map = {'A': 1.0,
            #            'D': 0.5714285714285714,
            #            'H': 0.0}

            # values = [val_map.get(node, 0.25) for node in G.nodes()]

            # # Specify the edges you want here
            # red_edges = [('A', 'C'), ('E', 'C')]
            # edge_colours = ['black' if not edge in red_edges else 'red'
            #                 for edge in G.edges()]
            # black_edges = [edge for edge in G.edges() if edge not in red_edges]

            # # Need to create a layout when doing
            # # separate calls to draw nodes and edges
            # pos = nx.spring_layout(G)
            # nx.draw_networkx_nodes(G, pos, cmap=plt.get_cmap('jet'),
            #                        node_color=values, node_size=500)
            # nx.draw_networkx_labels(G, pos)
            # nx.draw_networkx_edges(G, pos, edgelist=red_edges, edge_color='r', arrows=True)
            # nx.draw_networkx_edges(G, pos, edgelist=black_edges, arrows=False)
            # plt.show()

    def plan(self):
        if len(self._initial_graphs) == 0 and len(self._goal_instances) == 0:
            self._logger.warning('No initial graphs or no goal instances, using example files to plan instead')
            self._planner.plan()
        else:
            # For this to work properly, generate goal must be run before hands!
            self._planner.plan(self._initial_graphs[-1], self._goal_instances[-1])

    def act(self):
        if self._solution_filename is None:
            self._actor.act()
        else:
            # For this to work properly, generate goal must be run before hands!
            self._actor.act(self._solution_filename)
