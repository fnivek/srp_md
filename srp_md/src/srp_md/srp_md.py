""" Semantic Robot Programing Multiple Demonstrations.

Defines a class that encapsulates all functionality.

"""
from __future__ import absolute_import
from __future__ import print_function
# from builtins import str
from builtins import object
from . import learn
from . import sense
from . import goal
from . import evaluate
from . import plan
from . import act
from .pcd_io import write_pcd, pointcloud2_to_array, array_to_pointcloud2
from .utils import pose_difference
from srp_md_msgs.srv import PoseToSceneGraph, PoseToSceneGraphRequest
import srp_md

# Python imports
import os
import logging as log
import pickle
import networkx as nx
import matplotlib.pyplot as plt
import rospy
import message_filters
import tf
import cv_bridge as bridge
import cv2
import numpy as np
import math
import py_trees, py_trees_ros
from sensor_msgs.msg import CameraInfo, Image as ImageSensor_msg, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped, Pose, Vector3
from collections import defaultdict
from vision_msgs.msg import BoundingBox3D
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from copy import deepcopy
import random


class SrpMd(object):
    """ Semantic Robot Programing Multiple Demonstrations.

    Implements a semantic robot programing with multiple demonstrations. The
    algorithms can be configured with the set_<x> functions. See the associated
    python package for avaliable algorithms.

    """
    # TODO(Kevin): Set defaults when they exist
    def __init__(self, learner='factor_graph_learner', sensor='dope_sensor',
        goal_generator='factor_graph_goal_generator', goal_evaluator='adapt_goal_evaluator'):
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

        script_path = os.path.dirname(os.path.realpath(__file__))
        self._data_folder = os.path.realpath(script_path + '/../../../data')

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
        self._tf_sub = tf.TransformListener()
        self._pose_to_scene_graph_client = rospy.ServiceProxy('pose_to_scene_graph', PoseToSceneGraph)

    def get_num_demos(self):
        return len(self._raw_images)

    def get_num_keyframes(self):
        try:
            return len(self._raw_images[-1])
        except IndexError:
            return 0

    def get_num_inits(self):
        return len(self._initial_scenes)

    def set_scenes(self, demo_graphs, init_graphs):
        self._demo_graphs = demo_graphs
        self._initial_graphs = init_graphs

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

        try:
            trans, rot = self._tf_sub.lookupTransform('/base_link', '/head_camera_rgb_optical_frame', rospy.Time(0))
            to_base_link = tf.transformations.quaternion_matrix(rot)
            to_base_link[0:3, 3] = tf.transformations.translation_matrix(trans)[0:3, 3]
            self._new_image["tf"] = to_base_link
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
            self._logger.error('Failed to transform from /head_tilt_link to /base_link: {}'.format(e))

    def points_callback(self, points):
        # Save the image and info got into new image dictionary
        self._new_pcd = points

    def process_data(self):
        # Reset all graphs
        self._demo_graphs = []
        self._initial_graphs = []
        # If sensor is not set, log error
        if self._sensor is None:
            self._logger.error('Please select sensor!')
        # If the initial scene was not given, process data for all currently saved demonstrations
        self._logger.debug('Processing demonstrations...')
        for i, raw_image in enumerate(self._raw_images):
            raw_image['indices'] = []
            new_graph = self._sensor.process_data(raw_image)
            self._demo_graphs.append(new_graph)
            new_graph.to_png(self._data_folder + '/graphs/demonstration_graph_{}.png'.format(i), draw_disjoint=False, flip_relations=True)
        self._logger.debug('The demonstrations are: {}'.format(self._demo_graphs))
        # If the initial scene was given, process data for that scene
        self._logger.debug('Processing initial scene...')
        for i, init_scene in enumerate(self._initial_scenes):
            init_scene['indices'] = []
            new_graph = self._sensor.process_data(init_scene)
            self._initial_graphs.append(new_graph)
            new_graph.to_png(self._data_folder + '/graphs/test_graph_{}.png'.format(i), draw_disjoint=False, flip_relations=True)
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
        # Clear old goal instances
        self._goal_instances = []
        # If goal generator is not set, log error
        if self._goal_generator is None:
            self._logger.error('Please select goal generator!')
        else:
            # If using factor graph learner, do:
            if self.get_learner() == 'factor_graph_learner':
                # If initial scene was not set, log error
                if len(self._initial_graphs) == 0:
                    self._logger.error('No scene detected!\n')
                    self._goal_instances = []
                # If initial scene was seen, get goal instance
                else:
                    self._logger.info('Generating goal scenes from initial scenes...\n')
                    for i, init_graph in enumerate(self._initial_graphs):
                        goal_instance = self._goal_generator.generate_goal(self._factors, init_graph)
                        self._goal_instances.append(goal_instance)
                        goal_instance.to_png(self._data_folder + '/graphs/generated_goal_{}.png'.format(i), draw_disjoint=False, flip_relations=True)
                    self._logger.debug('The goal scenes are:\n{}'.format(''.join([str(goal) + '\n' for goal in self._goal_instances])))
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
    def save_for_user(self, filename, image_set):
        ind = filename.rfind(".")
        filename = filename[:ind] + "_" + filename[ind + 1:]
        for i, image in enumerate(image_set):
            br = bridge.CvBridge()
            cv_image = br.imgmsg_to_cv2(image["image"],)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            cv2.imwrite(filename + "_{}.png".format(i + 1), cv_image)
            write_pcd(filename + "_{}.pcd".format(i + 1), image["points"], overwrite=True)
            with open(filename + "_{}.txt".format(i + 1), "w") as f:
                for i in range(image["tf"].shape[0]):
                    for j in range(image["tf"].shape[1]):
                        f.write("{} ".format(image["tf"][i, j]))
                    f.write("\n")

    def write_demos(self, filename="./demos/raw_images.txt"):
        pickle.dump(self._raw_images, open(filename, 'wb'))
        self._logger.info('Success writing demo data to file {}\n'.format(filename))
        self.save_for_user(filename, self._raw_images)
        self._logger.info('Success saving image, pointcloud, and tf to respective files\n')

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
        self.save_for_user(filename, self._initial_scenes)
        self._logger.info('Success saving image, pointcloud, and tf to respective files\n')

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

    def accept_keyframe(self, demo_num, keyframe_num):
        """
        Accepts images and saves them as keyframes
        """
        # State the tag type
        self._logger.debug('Accepted keyframe number {} for demo number {}'.format(keyframe_num + 1, demo_num + 1))

        # Initialize new image and point cloud
        self._new_image = None
        self._new_pcd = None

        # Wait for message with timeout
        self._logger.debug('Getting current scene info. Please wait...')
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
            self._logger.debug('Received all the data!')
            self._new_image["points"] = self._new_pcd

            if len(self._raw_images) < demo_num:
                self._logger.error('The images and demos are not stored correctly')

            elif len(self._raw_images) == demo_num:
                self._raw_images.append([self._new_image])
            else:
                self._raw_images[demo_num].append(self._new_image)
            self._current_image = self._new_image

    def write_keyframes_demos(self, dirname):
        os.mkdir(dirname)
        pickle_name = os.path.join(dirname, "data.pickle")
        pickle.dump(self._raw_images, open(pickle_name, 'wb'))
        self._logger.info('Success writing demo data to file {}\n'.format(pickle_name))

        for i, demo in enumerate(self._raw_images):
            demo_dirname = os.path.join(dirname, str(i + 1))
            os.mkdir(demo_dirname)

            keyframe_filename = os.path.join(demo_dirname, "frame")

            for j in range(len(demo)):
                image = demo[j]
                br = bridge.CvBridge()
                cv_image = br.imgmsg_to_cv2(image["image"],)
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                cv2.imwrite(keyframe_filename + "_{}.png".format(j + 1), cv_image)
                write_pcd(keyframe_filename + "_{}.pcd".format(j + 1), image["points"], overwrite=True)
                with open(keyframe_filename + "_{}.txt".format(j + 1), "w") as f:
                    for k in range(image["tf"].shape[0]):
                        for l in range(image["tf"].shape[1]):
                            f.write("{} ".format(image["tf"][k, l]))
                        f.write("\n")

        self._logger.info('Success saving image, pointcloud, and tf to respective files\n')

    def load_keyframes_demos(self, pickle_name):
        self._raw_images = pickle.load(open(pickle_name, 'rb'))
        self._logger.info('Success loading demo images from file {}\n'.format(pickle_name))
        if len(self._raw_images) != 0:
            self._current_image = self._raw_images[-1]
        else:
            self._current_image = None

    def undo_keyframe(self):
        try:
            self._raw_images[-1].pop()
        except IndexError:
            self._logger.error('No Keyframes to undo!')

    def clear_keyframes(self, demo_num):
        self._raw_images[demo_num] = []

    def process_keyframes(self):
        # Process the images for cropping
        self.filter_keyframes()

        # Reset all graphs
        self._demo_graphs = []
        self._initial_graphs = []

        # If sensor is not set, log error
        if self.get_sensor() is not "dope_sensor":
            self._logger.error('Please use DOPE!')

        # If the initial scene was not given, process data for all currently saved demonstrations
        self._logger.debug('Processing demonstrations...')
        for demo_num, demo in enumerate(self._raw_images):
            new_graph = self.generate_obj_ass_sg(demo_num, demo[1:])
            self._demo_graphs.append(new_graph)
            new_graph.to_png(self._data_folder + '/graphs/demonstration_graph_{}.png'.format(demo_num), draw_disjoint=False, flip_relations=True)
        self._logger.debug('The demonstrations are:\n{}'.format(''.join([str(demo) + '\n' for demo in self._demo_graphs])))

        # If the initial scene was given, process data for that scene
        self._logger.debug('Processing initial scene...')
        for i in range(len(self._initial_scenes)):
            init_scene = self._initial_scenes[i]
            init_scene["indices"] = []
            new_graph = self._sensor.process_data(init_scene)
            self._initial_graphs.append(new_graph)
            new_graph.to_png(self._data_folder + '/graphs/test_graph_{}.png'.format(i), draw_disjoint=False, flip_relations=True)
        self._logger.debug('The initial scenes are:\n{}'.format(''.join([str(init_graph) + '\n' for init_graph in self._initial_graphs])))

        self.demo_publisher(self._raw_images)

    def filter_keyframes(self, depth_diff_threshold=0.02, depth_filter_threshold=1.7):
        # For each demos, do:
        for i, demo in enumerate(self._raw_images):
            # For each keyframe, do:
            for j, frame in enumerate(demo):
                # Get the stored image and pointcloud
                image = frame["image"]
                pcd = frame["points"]

                # Get the depth value from pointcloud
                depth = np.zeros((pcd.height, pcd.width), dtype='float32')
                ind = 0
                for p in pc2.read_points(pcd, skip_nans=False, field_names=("x", "y", "z")):
                    if p[2] > depth_filter_threshold:
                        pass
                    else:
                        depth[int(math.floor(ind / pcd.width)), int(ind % pcd.width)] = p[2]
                    ind += 1

                # Convert to cv2 image
                br = bridge.CvBridge()
                cv_image = br.imgmsg_to_cv2(image)

                # If first image, just store to past data
                if j==0:
                    cv_image_init = cv_image
                    prev_depth = depth
                    frame['indices'] = []

                # For following images, do:
                else:
                    # Compare prev_depth and depth for background subtraction
                    filter_array = np.nan_to_num(np.abs(prev_depth-depth)) > depth_diff_threshold

                    # Morphology
                    filter_array = filter_array.astype('uint8') * 255
                    kernel = np.ones((5, 5), np.uint8)
                    filter_array = cv2.morphologyEx(filter_array, cv2.MORPH_OPEN, kernel, iterations=1)

                    # Filter the images and give white background
                    filter_array = filter_array > 0

                    # Find the largest connected component (Our new object!)
                    nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(filter_array.astype('uint8'), connectivity=4)
                    # TODO(Henry): Ensure that stats is not an empty array as in the case of no difference between two frames
                    sizes = stats[:, -1]
                    max_label = 1
                    max_size = sizes[1]
                    for k in range(2, nb_components):
                        if sizes[k] > max_size:
                            max_label = k
                            max_size = sizes[k]

                    largest_comp_mask = np.zeros(output.shape)
                    largest_comp_mask[output == max_label] = 255

                    # Use convex hull to get the full compact points
                    contours, _ = cv2.findContours(largest_comp_mask.astype('uint8'), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                    largest_sum = 0
                    largest_convex_mask = np.zeros_like(largest_comp_mask, dtype='uint8')
                    for k in range(len(contours)):
                        hull_indices = cv2.convexHull(contours[k], returnPoints=False)
                        hull_indices = [ind for row in hull_indices.tolist() for ind in row]
                        convex_mask = np.zeros_like(largest_comp_mask, dtype='uint8')
                        cv2.fillConvexPoly(convex_mask, np.array([contours[k][l, :] for l in hull_indices], np.int32), 1)
                        if convex_mask.sum() > largest_sum:
                            largest_sum = convex_mask.sum()
                            largest_convex_mask = convex_mask

                    # Get all non-zero indices
                    pc_indices = largest_convex_mask.nonzero()
                    frame["indices"] = [row * pcd.width + col for row, col in
                                        zip(pc_indices[0].tolist(), pc_indices[1].tolist())]

                    negative = largest_convex_mask == 0
                    cv_image_filtered = np.zeros(cv_image.shape, dtype='uint8')
                    cv_image_filtered[:, :, 0] = (cv_image[:, :, 0] * largest_convex_mask).astype('uint8') + negative.astype('uint8') * 255 #cv_image_init[:, :, 0].astype('float32')
                    cv_image_filtered[:, :, 1] = (cv_image[:, :, 1] * largest_convex_mask).astype('uint8') + negative.astype('uint8') * 255 #cv_image_init[:, :, 1].astype('float32')
                    cv_image_filtered[:, :, 2] = (cv_image[:, :, 2] * largest_convex_mask).astype('uint8') + negative.astype('uint8') * 255 #cv_image_init[:, :, 2].astype('float32')
                    cv_image_filtered = br.cv2_to_imgmsg(cv_image_filtered, "rgb8")
                    frame['image'] = cv_image_filtered

                    prev_depth = depth

    def generate_obj_ass_sg(self, demo_num, demo):
        # Initialize variables
        all_obj_bboxes = {}
        # object_counts = defaultdict(int)
        uuid = 0
        req = PoseToSceneGraphRequest()
        req.names = []
        req.objects = []

        # For each, image, process with dope and add to all_obj_bboxes set, depending on the object's pose
        for i, image in enumerate(demo):
            self._sensor.process_data(image)
            obj_bboxes = py_trees.blackboard.Blackboard().get('obj_bboxes')

            # Skip null detctions
            if obj_bboxes is None or len(obj_bboxes.keys()) == 0:
                continue

            # For the first keyframe, just add all objects
            if i == 0:
                for obj_key in obj_bboxes.keys():
                    # Add with new uuid
                    ind = obj_key.rfind("_")
                    # new_id = obj_key[:ind] + '_' + str(object_counts[obj_key[:ind]])
                    new_id = obj_key[:ind] + '_' + str(uuid)
                    # print(type(new_id))
                    all_obj_bboxes[new_id] = obj_bboxes[obj_key]
                    req.names.append(new_id)
                    req.objects.append(obj_bboxes[obj_key])
                    # object_counts[obj_key[:ind]] += 1
                    uuid += 1

            # For later keyframes, compare with existing bath of objects
            else:
                # For each new object, do:
                for obj_post_key in obj_bboxes.keys():
                    """
                    Add something here to filter out false positives?
                    """

                    closest_obj_name = None
                    closest_dist = 10000
                    # For each old objects, do:
                    for obj_prev_key in all_obj_bboxes.keys():
                        dist = pose_difference(all_obj_bboxes[obj_prev_key].center, obj_bboxes[obj_post_key].center)
                        if dist < closest_dist:
                            closest_dist = dist
                            closest_obj_name = obj_post_key

                    # If the object is far away from previous objects, add to the dictionary
                    if closest_dist > 0.01:
                        ind = obj_post_key.rfind("_")
                        # new_id = obj_post_key[:ind] + '_' + str(object_counts[obj_post_key[:ind]])
                        new_id = obj_post_key[:ind] + '_' + str(uuid)
                        # print(type(new_id))
                        all_obj_bboxes[new_id] = obj_bboxes[obj_post_key]
                        req.names.append(new_id)
                        req.objects.append(obj_bboxes[obj_post_key])
                        # object_counts[obj_post_key[:ind]] += 1
                        uuid += 1

        # self._logger.debug('Collected objects: {}'.format(all_obj_bboxes))
        py_trees.blackboard.Blackboard().set('demo_{}'.format(str(demo_num)), all_obj_bboxes)

        # table_uuid = sum(object_counts.values())
        table_uuid = uuid
        req.names.append('table')
        req.objects.append(BoundingBox3D())

        try:
            resp = self._pose_to_scene_graph_client(req)
        except rospy.ServiceException, e:
            self._logger.error('Failed to get scene graph from pose: {}'.format(e))
            return None
        # Convert ros msg to scene graph
        # self._logger.debug('Pose to scene graph result: {}'.format(resp))
        # Make srp_md objects
        objs = []
        for name in req.names:
            label = name
            uuid = table_uuid
            if name.find('table') == -1:
                label = name[:name.rfind('_')]
                uuid = int(name[name.rfind('_') + 1:])
            objs.append(srp_md.Object(name=name, id_num=uuid, uuid=uuid, assignment={'class': label}))

        # Build the scene graph
        scene_graph = srp_md.SceneGraph(objs)

        # Update all relations from response
        for name1, name2, rel_value in zip(resp.object1, resp.object2, resp.relation):
            obj1 = scene_graph.get_obj_by_name(name1)
            obj2 = scene_graph.get_obj_by_name(name2)
            rel = scene_graph.get_rel_by_objs(obj1, obj2)
            rel.value = rel_value

            # Check if obj1 and obj2 are fliped
            if rel.obj1 != obj1:
                rel.rev_relation()

        print("Scenegraph for demo {}:\n{}".format(demo_num, str(scene_graph)))
        return scene_graph

    def demo_publisher(self, demo):
        bbox_pub_list = []
        pc_pub_list = []
        rate = rospy.Rate(10)
        for i in range(len(demo)):
            bbox_pub_name = 'demo_{}_bboxes'.format(str(i))
            pc_pub_name = 'demo_{}_pc'.format(str(i))
            bbox_pub_list.append(rospy.Publisher(bbox_pub_name, BoundingBoxArray, queue_size=2))
            pc_pub_list.append(rospy.Publisher(pc_pub_name, PointCloud2, queue_size=1))

        for i in range(20):
            for i in range(len(demo)):
                bbox_array = BoundingBoxArray()
                bbox_array.header.frame_id = "head_camera_rgb_optical_frame"#"base_link"
                obj_bboxes = py_trees.blackboard.Blackboard().get('demo_{}'.format(str(i)))
                for obj_key in obj_bboxes.keys():
                    bbox = BoundingBox()
                    bbox.header.frame_id = "head_camera_rgb_optical_frame"#"base_link"
                    bbox.pose = obj_bboxes[obj_key].center
                    bbox.dimensions = obj_bboxes[obj_key].size
                    bbox_array.boxes.append(bbox)
                bbox_pub_list[i].publish(bbox_array)
                pc = demo[i][-1]["points"]
                pc.header.stamp = rospy.Time.now()
                pc_pub_list[i].publish(pc)

            rate.sleep()


    def grocery_experiment(self):
        # Run dope action node
        pass
