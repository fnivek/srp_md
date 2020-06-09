""" Actions.

Define the actions used in srp_md.

"""
from copy import deepcopy

import rospy
import actionlib
import tf
import py_trees
import py_trees_ros
import message_filters
import time
import threading

import random

import numpy as np

from gazebo_ros_link_attacher.srv import *
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest

from visualization_msgs.msg import MarkerArray, Marker
from moveit_msgs.msg import PlanningScene, CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from control_msgs.msg import GripperCommandGoal, GripperCommandAction
import move_base_msgs.msg as move_base_msgs
from srp_md_msgs.msg import *
from dope_msgs.msg import DopeAction, DopeGoal, DopeResult
from geometry_msgs.msg import Pose, PoseStamped, Transform, PoseArray, TransformStamped, Vector3, Point, Quaternion
from sensor_msgs.msg import CameraInfo, Image as ImageSensor_msg, PointCloud2
from vision_msgs.msg import BoundingBox3D, Detection3D, ObjectHypothesisWithPose
from gazebo_msgs.msg import *
import actionlib_msgs.msg as actionlib_msgs
from grasploc_wrapper_msgs.msg import GrasplocAction, GrasplocGoal, GrasplocResult
from scipy.spatial.transform import Rotation as R
import srp_md

import fetch_manipulation_pipeline.msg
from behavior_manager.conditions.arm_tucked_condition import ArmTuckedCondition
from behavior_manager.interfaces.manipulation_behavior_new import (TuckWithCondBehavior, HeadMoveBehavior, TuckBehavior,
                                                                   FullyCollapseTorso, FullyExtendTorso)
from behavior_manager.interfaces.fetch_manipulation_behavior import *
from behavior_manager.interfaces.sleep_behavior import *

gripper_length = 0.15
# gripper_length = 0.22
pre_grasp_offset = 0.17
# pre_grasp_offset = 0.07

to_grasp_tf = TransformStamped()
to_grasp_tf.header.frame_id = "gripper_link"
to_grasp_tf.transform.translation.x = pre_grasp_offset * 0.8
to_grasp_tf.transform.rotation.w = 1.0

to_grasp_full_tf = TransformStamped()
to_grasp_full_tf.header.frame_id = "gripper_link"
to_grasp_full_tf.transform.translation.x = pre_grasp_offset * 0.95
to_grasp_full_tf.transform.rotation.w = 1.0

to_place_tf = TransformStamped()
to_place_tf.header.frame_id = "gripper_link"
to_place_tf.transform.translation.x = pre_grasp_offset * 0.8
to_place_tf.transform.rotation.w = 1.0

to_push_tf = TransformStamped()
to_push_tf.header.frame_id = "base_link"
# to_push_tf.transform.translation.x = pre_grasp_offset * 1.2
to_push_tf.transform.translation.x = 0.27 + 0.05
to_push_tf.transform.rotation.w = 1.0

to_grab_box_tf = TransformStamped()
to_grab_box_tf.header.frame_id = "gripper_link"
# to_grab_box_tf.transform.translation.y = - pre_grasp_offset * 2.7
to_grab_box_tf.transform.translation.y = -(0.48 + 0.03) / 4
to_grab_box_tf.transform.rotation.w = 1.0

to_grab_box_tf_append = TransformStamped()
to_grab_box_tf_append.header.frame_id = "gripper_link"
# to_grab_box_tf.transform.translation.y = - pre_grasp_offset * 2.7
to_grab_box_tf_append.transform.translation.y = -(0.48 + 0.03) / 4 * 3
to_grab_box_tf_append.transform.rotation.w = 1.0

to_grab_box_back_tf = TransformStamped()
to_grab_box_back_tf.header.frame_id = "base_link"
to_grab_box_back_tf.transform.translation.y = pre_grasp_offset * 0.5
to_grab_box_back_tf.transform.rotation.w = 1.0

to_small_back_tf = TransformStamped()
to_small_back_tf.header.frame_id = "gripper_link"
to_small_back_tf.transform.translation.x = - pre_grasp_offset * 0.3
to_small_back_tf.transform.rotation.w = 1.0

to_back_tf = TransformStamped()
to_back_tf.header.frame_id = "gripper_link"
to_back_tf.transform.translation.x = - pre_grasp_offset * 0.8
to_back_tf.transform.rotation.w = 1.0

up_tf_after_place = TransformStamped()
# up_tf_after_place.header.frame_id = "gripper_link"
up_tf_after_place.header.frame_id = "base_link"
up_tf_after_place.transform.translation.z = 0.15
# up_tf_after_place.transform.translation.x = -0.15
up_tf_after_place.transform.rotation.w = 1

up_tf = TransformStamped()
up_tf.header.frame_id = "base_link"
# up_tf.transform.translation.z = 0.25
up_tf.transform.translation.z = 0.15
up_tf.transform.rotation.w = 1

up_tf_little = TransformStamped()
up_tf_little.header.frame_id = "base_link"
up_tf_little.transform.translation.z = 0.03
up_tf_little.transform.rotation.w = 1

down_tf = TransformStamped()
down_tf.header.frame_id = "base_link"
down_tf.transform.translation.z = -0.03
down_tf.transform.rotation.w = 1

left_tf = TransformStamped()
left_tf.header.frame_id = "base_link"
left_tf.transform.translation.y = 0.03
left_tf.transform.rotation.w = 1

right_tf = TransformStamped()
right_tf.header.frame_id = "base_link"
right_tf.transform.translation.y = -0.03
right_tf.transform.rotation.w = 1

back_tf = TransformStamped()
back_tf.header.frame_id = "base_link"
back_tf.transform.translation.x = -0.03
back_tf.transform.rotation.w = 1

bb = BoundingBox3D()
bb.center.position.x = 1.0
bb.center.position.y = 0.0
bb.center.position.z = 0.75
bb.center.orientation.x = 0.0
bb.center.orientation.y = 0.087
bb.center.orientation.z = 0.0
bb.center.orientation.w = 0.996
bb.size.x = 0.25
bb.size.y = 0.25
bb.size.z = 0.5
py_trees.blackboard.Blackboard().set('crop_box', bb)


grocery_box_offset = Vector3()
grocery_box_offset.x = 0
grocery_box_offset.y = 0.25
grocery_box_offset.z = 0
py_trees.blackboard.Blackboard().set('grocery_box_offset', grocery_box_offset)

grocery_box_size = Vector3()
grocery_box_size.x = 0.27432
grocery_box_size.y = 0.4838
grocery_box_size.z = 0.1524
py_trees.blackboard.Blackboard().set('grocery_box_size', grocery_box_size)

grocery_collision_box_name = []
grocery_collision_box_name.append('grocery_collision_box_left')
grocery_collision_box_name.append('grocery_collision_box_right')
grocery_collision_box_name.append('grocery_collision_box_front')
grocery_collision_box_name.append('grocery_collision_box_back')
py_trees.blackboard.Blackboard().set('grocery_collision_box_name', grocery_collision_box_name)

def ResetAct(name):
    """ Bring robot back to default pose.

    Lift the torso, tuck the arm, open the gripper and look strait.

    """
    seq = py_trees.composites.Sequence(name='seq_{}'.format(name))
    root = py_trees.composites.Parallel(
        name='pal_{}'.format(name),
        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL,
        synchronize=True,
        allow_failure=False)
    root.add_children([
        seq,
        HeadMoveBehavior('act_{}_look_strait'.format(name), 'MoveStraight'),
        OpenGripperAct('act_{}_open_gripper'.format(name))
    ])
    seq.add_children([
        FullyExtendTorso('act_{}_extend_torso'.format(name)),
        TuckWithCondBehavior('act_{}_tuck_arm'.format(name), tuck_pose='tuck'),
        FullyCollapseTorso('act_{}_collapse_torso'.format(name))
    ])
    return root

def GripperAct(name, percent_open, max_effort=None):
    gripper_max_pose = 0.100852034986  # gripper_max_pose measured on robot
    goal = GripperCommandGoal()
    pose = gripper_max_pose * percent_open
    goal.command.position = pose
    if pose > gripper_max_pose:
        goal.command.position = gripper_max_pose
    elif pose < 0.0:
        goal.command.position = 0.0
    if max_effort is not None:
        goal.command.max_effort = max_effort
    return py_trees_ros.actions.ActionClient(
        name=name,
        action_spec=GripperCommandAction,
        action_goal=goal,
        action_namespace='/gripper_controller/gripper_action'
    )

def OpenGripperAct(name, max_effort=None):
    return GripperAct(name, 1.0, max_effort)

def CloseGripperAct(name, max_effort=None):
    return GripperAct(name, 0.0, max_effort)

class MoveToPoseAct(py_trees_ros.actions.ActionClient):
    def __init__(self, name, pose=Pose(), *argv, **kwargs):
        super(MoveToPoseAct, self).__init__(
            name=name,
            action_spec=MoveToPoseAction,
            action_goal=MoveToPoseGoal(),
            action_namespace='move',
            *argv,
            **kwargs
        )
        self.pose = pose
        self.action_goal.pose = self.pose

class MoveToFirstPoseAct(py_trees_ros.actions.ActionClient):
    def __init__(self, name, poses_key, place_if=False, object_index=None, *argv, **kwargs):
        super(MoveToFirstPoseAct, self).__init__(
            name=name,
            action_spec=MoveToFirstPoseAction,
            action_goal=MoveToFirstPoseGoal(),
            action_namespace='move_to_first_pose',
            *argv,
            **kwargs
        )
        self._poses_key = poses_key
        self._place_if = place_if
        self._poses = None
        self._object_index = object_index

    def initialise(self):
        super(MoveToFirstPoseAct, self).initialise()
        self.action_goal.poses = py_trees.blackboard.Blackboard().get(self._poses_key)
        self._poses = py_trees.blackboard.Blackboard().get(self._poses_key)
        # self._object_index = object_index

    def update(self):
        self.logger.debug("{0}.update()".format(self.__class__.__name__))
        if not self.action_client:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            return py_trees.Status.INVALID
        # Ensure we have a poeses
        if self.action_goal.poses is None:
            return py_trees.Status.FAILURE
        # pity there is no 'is_connected' api like there is for c++
        if not self.sent_goal:
            self.action_client.send_goal(self.action_goal)
            self.sent_goal = True
            self.feedback_message = "sent goal to the action server"
            return py_trees.Status.RUNNING
        if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED, actionlib_msgs.GoalStatus.PREEMPTED]:
            return py_trees.Status.FAILURE
        result = self.action_client.get_result()
        # print('result', result)
        if result is not None:
            if result.success:
                # print('pose_index: ',result.pose_index)

                if self._place_if:
                    obj_bboxes_moved = py_trees.blackboard.Blackboard().get('obj_bboxes')
                    obj_bboxes_moved[self._object_index].center.position.x = self._poses[result.pose_index].position.x
                    obj_bboxes_moved[self._object_index].center.position.y = self._poses[result.pose_index].position.y
                    py_trees.blackboard.Blackboard().set('obj_bboxes', obj_bboxes_moved)
                return py_trees.Status.SUCCESS
            else:
                print("result.success is not true")
        else:
            # print('Still running')
            return py_trees.Status.RUNNING



class MoveToRelativePoseAct(py_trees_ros.actions.ActionClient):
    def __init__(self, name, transform=TransformStamped(), *argv, **kwargs):
        super(MoveToRelativePoseAct, self).__init__(
            name=name,
            action_spec=MoveToRelativePoseAction,
            action_goal=MoveToRelativePoseGoal(),
            action_namespace='relative_move',
            *argv,
            **kwargs
        )
        self.action_goal.transform = transform

class RelativeCartesianMoveAct(py_trees_ros.actions.ActionClient):
    def __init__(self, name, pose_diff_msg=TransformStamped(), *argv, **kwargs):
        super(RelativeCartesianMoveAct, self).__init__(
            name=name,
            action_spec=RelativeCartesianMoveAction,
            action_goal=RelativeCartesianMoveGoal(),
            action_namespace='relative_cartesian_move',
            *argv,
            **kwargs
        )
        self.action_goal.pose_diff_msg = pose_diff_msg

# class GetDopeSnapshotAct(py_trees_ros.actions.ActionClient):
#     def __init__(self, name, object_name=None, *argv, **kwargs):
#         super(GetDopeSnapshotAct, self).__init__(
#             name=name,
#             action_spec=DopeAction,
#             action_goal=DopeGoal(),
#             action_namespace='dope',
#             *argv,
#             **kwargs
#         )

#         self._timeout = 5
#         self._listener = tf.TransformListener()

#         self.image_sub = message_filters.Subscriber(
#             '/head_camera/rgb/image_raw',
#             ImageSensor_msg
#         )
#         self.info_sub = message_filters.Subscriber(
#             '/head_camera/rgb/camera_info',
#             CameraInfo
#         )
#         self.ts = message_filters.TimeSynchronizer([self.image_sub, self.info_sub], 100)
#         self._blackboard = py_trees.blackboard.Blackboard()
#         self._pc = self._blackboard.get('depth_pc')
#         # print('init_ pc: ',type(self._pc))
#         self.ts.registerCallback(self.image_callback)
#         self._model_state = None
#         self._object_name = object_name

#     def initialise(self):
#         super(GetDopeSnapshotAct, self).initialise()
#         self._pc = self._blackboard.get('depth_pc')

#     def image_callback(self, image, info):
#         self.action_goal = DopeGoal()
#         self.action_goal.image = image
#         self.action_goal.cam_info = info
#         # print('image_callback pc: ',type(self._pc))
#         # self.action_goal.pc = self._pc

#     def update(self):
#         self.logger.debug("{0}.update()".format(self.__class__.__name__))
#         if not self.action_client:
#             self.feedback_message = "no action client, did you call setup() on your tree?"
#             return py_trees.Status.INVALID
#         # if not self.sent_goal and self.action_goal.cam_info != CameraInfo() and self.action_goal.image != ImageSensor_msg():
#         if not self.sent_goal:
#             # self._pc = self._blackboard.get('depth_pc')
#             # print('send_goal pc: ',type(self.action_goal.pc))
#             # print('self._pc: ',type(self._pc))
#             self.action_goal.pc = self._pc
#             # print("printing things:")
#             time.sleep(1)
#             # print(self.action_goal.image)

#             # print(self.action_goal.cam_info)
#             # print(type(self.action_goal.cam_info))
#             self.action_client.send_goal(self.action_goal)
#             # print('send_goal pc: ',type(self.action_goal.pc))
#             self.sent_goal = True
#             self.feedback_message = "sent goal to the action server"
#             return py_trees.Status.RUNNING
#         self.feedback_message = self.action_client.get_goal_status_text()
#         if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
#                                               actionlib_msgs.GoalStatus.PREEMPTED]:
#             return py_trees.Status.FAILURE
#         result = self.action_client.get_result()
#         # print(result)

#         if result:
#             # print(result.detections)
#             # print(type(result))
#             # test_result = DopeResult()
#             # test_result.detections.append(Detection3D())
#             # print("printing test_result: ")
#             # print(test_result)
#             # print('detection.bbox: ', result.detections[0].bbox)
#             # self._model_state = py_trees.blackboard.Blackboard().get('model_state')
#             # object_name = []
#             # object_index = []
#             # # cracker gelatin meat mustard soup sugar
#             # All_size = []
#             # All_size.append(Vector3(0.0718, 0.164, 0.213))
#             # All_size.append(Vector3(0.09, 0.097, 0.029))
#             # All_size.append(Vector3(0.103, 0.057, 0.084))
#             # All_size.append(Vector3(0.08, 0.06, 0.19))
#             # All_size.append(Vector3(0.068, 0.068, 0.102))
#             # All_size.append(Vector3(0.045, 0.097, 0.174))
#             # All_name = []
#             # All_name.append("cracker")
#             # All_name.append("gelatin")
#             # All_name.append("meat")
#             # All_name.append("mustard")
#             # All_name.append("soup")
#             # All_name.append("sugar")
#             # dope_result = DopeResult()
#             # if self._model_state is None:
#             #     return py_trees.Status.FAILURE

#             # for name in self._model_state.name:
#             #     if name == "ground_plane" or name == "fetch"or name == "table" or name == "lower_table":
#             #         continue
#             #     else:
#             #         if name[-1] == 't':
#             #             object_name.append(name[0:-5] + '_0')
#             #             object_index.append(All_name.index(name[0:-5]))
#             #         else:
#             #             object_name.append(name[0:-7] + '_' + str(int(name[-1]) + 1))
#             #             object_index.append(All_name.index(name[0:-7]) + 1)
#             #     dope_result.detections.append(Detection3D())
#             #     index_cur = len(dope_result.detections) - 1
#             #     dope_result.detections[index_cur].results.append(ObjectHypothesisWithPose())
#             #     dope_result.detections[index_cur].results[0].id = object_index[len(object_index) - 1]
#             #     dope_result.detections[index_cur].results[0].score = 0.9
#             #     dope_result.detections[index_cur].results[0].pose = self._model_state.pose[self._model_state.name.index(name)]
#             #     dope_result.detections[index_cur].bbox.center = self._model_state.pose[self._model_state.name.index(name)]
#             #     dope_result.detections[index_cur].bbox.size = All_size[object_index[len(object_index) - 1] - 1]

#             # # print("dope_result: ")
#             # # print(dope_result)
#             # print("object_name: ", object_name)
#             # num_object = None
#             # if not self._object_name == None:
#             #     if self._object_name[-1] == '0':
#             #         num_object = None
#             #         name_model = self._object_name[0:-2] + "_test"
#             #     else:
#             #         num_object = int(self._object_name[-1] - 1)
#             #         name_model = self._object_name[0:-2] + "_test" + str(num_object)

#             #     object_index = self._model_state.name.index(name_model)
#             #     # print(type(self._model_state))
#             #     # print(self._model_state.name[object_index])
#             #     # print(type(self._model_state.pose[object_index]))
#             #     # print(self._model_state.twist[object_index])


#             obj_bboxes_prev = py_trees.blackboard.Blackboard().get('obj_bboxes')
#             obj_bboxes_post = {}
#             uuid = 0
#             class_ids = rospy.get_param("/dope/class_ids")
#             class_names = {class_id: name for name, class_id in class_ids.iteritems()}
#             # print(class_names)
#             for detection in result.detections:
#                 pose_stamped = PoseStamped()
#                 pose_stamped.header.frame_id = '/head_camera_rgb_optical_frame' # shouldn't this be same as self.imsage_sub?
#                 pose_stamped.header.stamp = rospy.Time(0)
#                 pose_stamped.pose = detection.bbox.center
#                 try:
#                     tfed_pose_stamped = self._listener.transformPose('/base_link', pose_stamped)
#                     detection.bbox.center = tfed_pose_stamped.pose
#                 except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
#                     print 'Failed to transform from /head_camera_rgb_optical_frame to /base_link: {}'.format(e)
#                     return None

#                 obj_bboxes_post[class_names[detection.results[0].id] + '_' + str(uuid)] = detection.bbox
#                 uuid += 1

#             # print 'obj_bboxes_post: {}'.format(obj_bboxes_post)

#             if obj_bboxes_prev is None:
#                 obj_bboxes = obj_bboxes_post
#             else:
#                 obj_bboxes = obj_bboxes_prev
#                 for obj_prev_key in obj_bboxes_prev.keys():
#                     closest_obj_name = None
#                     closest_dist = 10000
#                     for obj_post_key in obj_bboxes_post.keys():
#                         # dist = srp_md.pose_difference(obj_bboxes_prev[obj_prev_key].center, obj_bboxes_post[obj_post_key].center)
#                         pose_1 = obj_bboxes_prev[obj_prev_key].center
#                         pose_2 = obj_bboxes_post[obj_post_key].center
#                         dist = position_diff = np.sqrt((pose_1.position.x - pose_2.position.x) ** 2 + (pose_1.position.y - pose_2.position.y) ** 2 +
#                             (pose_1.position.z - pose_2.position.z) ** 2)
#                         if dist < closest_dist:
#                             closest_dist = dist
#                             closest_obj_name = obj_post_key
#                     print('closest_dist: ', closest_dist)
#                     # print 'Closest distance {}, with prev object {} and post object {}'.format(closest_dist, obj_prev_key, closest_obj_name)
#                     if closest_dist <= 0.06:
#                         obj_bboxes[obj_prev_key] = obj_bboxes_post[closest_obj_name]
#                         del obj_bboxes_post[closest_obj_name]
#                     else:
#                         print 'No closest object found. Defaulting with previously seen object bbox value for {}'.format(obj_prev_key)

#             py_trees.blackboard.Blackboard().set('obj_bboxes', obj_bboxes)
#             print "obj bboxes: ", obj_bboxes

#             return py_trees.Status.SUCCESS
#         else:
#             self.feedback_message = self.override_feedback_message_on_running
#             return py_trees.Status.RUNNING

class GetFakeDopeSnapshotAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, model_state_key="model_state"):
        super(GetFakeDopeSnapshotAct, self).__init__(name)
        self._model_state_key = model_state_key
        self._model_state = None
        self._dope_detection_pub = None
        self._set_model_state_pub = None
        self._obj_bboxes = dict()
        self._marker = None
        self._grocery_box_size = None

    def setup(self, timeout):
        # self._dope_detection_pub = rospy.Publisher('/original_grasploc_poses', DopeActionResult, queue_size=10)
        self._marker_sub = rospy.Subscriber('/dope/markers', MarkerArray, self.initial_callback)
        self._dope_detection_pub = rospy.Publisher('/dope/markers', MarkerArray, queue_size=10)
        self._set_model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        return True

    def initial_callback(self, marker):
        self._marker = marker

    def update(self):
        print(type(self._marker))
        self._model_state = py_trees.blackboard.Blackboard().get('model_state')
        self._grocery_box_size = py_trees.blackboard.Blackboard().get('grocery_box_size')

        object_name = []
        object_index = []
        # cracker gelatin meat mustard soup sugar
        All_offset = []
        All_offset.append(Vector3(-0.014, -0.014, 0.1))
        All_offset.append(Vector3(-0.023, -0.01, 0.0145))
        All_offset.append(Vector3(-0.032, -0.026, 0.038))
        All_offset.append(Vector3(-0.016, -0.025, 0.095))
        All_offset.append(Vector3(-0.01, 0.0, 0.053))
        All_offset.append(Vector3(-0.0095, -0.017, 0.088))
        All_size = []
        All_size.append(Vector3(0.0718, 0.164, 0.213))
        All_size.append(Vector3(0.09, 0.097, 0.029))
        All_size.append(Vector3(0.103, 0.057, 0.084))
        All_size.append(Vector3(0.08, 0.06, 0.19))
        All_size.append(Vector3(0.068, 0.068, 0.102))
        All_size.append(Vector3(0.045, 0.097, 0.174))
        Table_size = []
        Table_size.append(Vector3(0.8, 1.5, 0.55))
        Table_size.append(Vector3(0.45, 0.5, 0.50))
        table_size_z = 0.01
        # Table_sizemod.append(Vector3(0.85, 1, 0.09))
        # Table_size.append(Vector3(0.45, 0.5, 0.09))
        All_name = []
        All_name.append("cracker")
        All_name.append("gelatin")
        All_name.append("meat")
        All_name.append("mustard")
        All_name.append("soup")
        All_name.append("sugar")
        dope_result = DopeResult()
        plane_bboxes = []
        grocery_bboxes = []
        if self._model_state is None:
            return py_trees.Status.FAILURE

        for name in self._model_state.name:
            if name == "ground_plane" or name == "fetch":
                continue
            elif name == "table" or name == "Table":
                table_bbox = BoundingBox3D()
                table_bbox.center = self._model_state.pose[self._model_state.name.index(name)]
                table_bbox.size = Table_size[0]
                # table_bbox.center.position.z = table_bbox.center.position.z + table_bbox.size.z / 2
                # specialized for table using now
                table_bbox.center.position.z = table_bbox.center.position.z + table_bbox.size.z
                # specialized for gazebo
                table_bbox.center.position.y = 0
                table_bbox.size.z = table_size_z
                plane_bboxes.append(table_bbox)
                continue
            elif name == "lower_table":
                table_bbox = BoundingBox3D()
                table_bbox.center = self._model_state.pose[self._model_state.name.index(name)]
                table_bbox.size = Table_size[1]
                table_bbox.center.position.z = table_bbox.center.position.z + table_bbox.size.z / 2
                table_bbox.size.z = table_size_z
                plane_bboxes.append(table_bbox)
                continue
            elif "conveyor" in name:
                continue
            elif "box" in name:
                grocery_bbox = BoundingBox3D()
                grocery_bbox.center = self._model_state.pose[self._model_state.name.index(name)]
                # specialized for the grocery using now
                grocery_bbox.center.position.z = grocery_bbox.center.position.z + self._grocery_box_size.z / 2
                grocery_bbox.size = self._grocery_box_size
                grocery_bboxes.append(grocery_bbox)
                continue
            else:
                if name[-1] == 't':
                    object_name.append(name[0:-5] + '_0')
                    object_index.append(All_name.index(name[0:-5]) + 1)
                else:
                    object_name.append(name[0:-7] + '_' + str(int(name[-1]) + 1))
                    object_index.append(All_name.index(name[0:-7]) + 1)
            dope_result.detections.append(Detection3D())
            index_cur = len(dope_result.detections) - 1
            dope_result.detections[index_cur].results.append(ObjectHypothesisWithPose())
            dope_result.detections[index_cur].results[0].id = object_index[len(object_index) - 1]
            dope_result.detections[index_cur].results[0].score = 0.9

            # print("plane_bboxes: ", plane_bboxes)

            pose_object = self._model_state.pose[self._model_state.name.index(name)]
            pose_ori = R.from_quat([
                pose_object.orientation.x,
                pose_object.orientation.y,
                pose_object.orientation.z,
                pose_object.orientation.w])
            pose_rotated = R.from_quat([0.5, 0.5 ,0.5 ,0.5])

            pose_real = pose_ori * pose_rotated
            print("pose_ori: ", pose_ori.as_quat())
            print("pose_real: ", pose_real.as_quat())
            pose_real_quat = pose_real.as_quat()
            transformed_pose = Pose()
            size_object = Vector3()
            transformed_pose.orientation.x = pose_real_quat[0]
            transformed_pose.orientation.y = pose_real_quat[1]
            transformed_pose.orientation.z = pose_real_quat[2]
            transformed_pose.orientation.w = pose_real_quat[3]
            tf = R.from_quat([
                transformed_pose.orientation.x,
                transformed_pose.orientation.y,
                transformed_pose.orientation.z,
                transformed_pose.orientation.w])

            offset = pose_ori.apply([All_offset[object_index[len(object_index) - 1] - 1].x, All_offset[object_index[len(object_index) - 1] - 1].y, All_offset[object_index[len(object_index) - 1] - 1].z])
            # transformed_pose.position.x = pose_object.position.x #+ All_offset[object_index[len(object_index) - 1] - 1].x
            # transformed_pose.position.y = pose_object.position.y #+ All_offset[object_index[len(object_index) - 1] - 1].y
            # transformed_pose.position.z = pose_object.position.z #+ All_offset[object_index[len(object_index) - 1] - 1].z
            print("offset: ", offset)
            transformed_pose.position.x = pose_object.position.x + offset[0]
            transformed_pose.position.y = pose_object.position.y + offset[1]
            transformed_pose.position.z = pose_object.position.z + offset[2]
            print("transformed_pose: ", transformed_pose)
            # print("transformed_pose.position: ", transformed_pose.position)
            # print("pose_object.position", pose_object.position)
            # transformed_pose.position = pose_object.position
            size_object.x = All_size[object_index[len(object_index) - 1] - 1].y
            size_object.y = All_size[object_index[len(object_index) - 1] - 1].z
            size_object.z = All_size[object_index[len(object_index) - 1] - 1].x
            # dope_result.detections[index_cur].results[0].pose = self._model_state.pose[self._model_state.name.index(name)]
            # dope_result.detections[index_cur].bbox.center = self._model_state.pose[self._model_state.name.index(name)]
            # dope_result.detections[index_cur].bbox.size = All_size[object_index[len(object_index) - 1] - 1]
            dope_result.detections[index_cur].results[0].pose = transformed_pose
            dope_result.detections[index_cur].bbox.center = transformed_pose
            dope_result.detections[index_cur].bbox.size = size_object
            self._obj_bboxes[object_name[len(object_name) - 1]] = dope_result.detections[index_cur].bbox

        test_markerarray = MarkerArray()
        delete_all_marker = Marker()
        delete_all_marker.action = 3
        test_markerarray.markers.append(delete_all_marker)
        if self._marker == None:
            sequence = 1
        else:
            sequence = self._marker.markers[len(self._marker.markers) - 1].header.seq
            sequence = sequence + 1
        for i in range(len(self._obj_bboxes)):
            marker_temp = Marker()
            # marker_temp.header.frame_id = "head_camera_rgb_optical_frame"
            marker_temp.header.frame_id = "base_link"

            marker_temp.header.stamp.secs = 2672
            marker_temp.header.stamp.nsecs = 673000000
            marker_temp.header.stamp=rospy.Time.now()
            marker_temp.header.seq = sequence
            marker_temp.ns = 'bboxes'
            marker_temp.id = i + 1
            # marker_temp.type = dope_result.detections[i].results[0].id
            marker_temp.type = 1
            # marker_temp.type = 7
            marker_temp.action = 0
            marker_temp.pose = dope_result.detections[i].results[0].pose
            marker_temp.scale = dope_result.detections[i].bbox.size
            marker_temp.color.r = 1.0
            marker_temp.color.a = 0.4

            marker_text = Marker()
            # marker_text.header.frame_id = "head_camera_rgb_optical_frame"
            marker_text.header.frame_id = "base_link"

            marker_text.header.stamp.secs = 2672 + i
            marker_text.header.stamp.nsecs = 673000000 + i * 10000000
            marker_text.header.stamp=rospy.Time.now()
            marker_text.header.seq = sequence
            marker_text.ns = 'text'
            marker_text.id = i + 1
            marker_text.type = 9
            marker_text.action = 0
            marker_text.pose = dope_result.detections[i].results[0].pose
            marker_text.scale = Vector3(0.05, 0.05, 0.05)
            marker_text.color.r = 1.0
            marker_text.color.a = 0.4
            marker_text.text = All_name[dope_result.detections[i].results[0].id - 1] + "(0.9)"

            test_markerarray.markers.append(marker_temp)
            test_markerarray.markers.append(marker_text)
            # print(marker_temp.header)
            # print(marker_temp.pose)
        # print(len(self._obj_bboxes))
        self._dope_detection_pub.publish(test_markerarray)
        # print("plane_bboxes: ", plane_bboxes)
        # blackboard.set(self._out_poses_key, offset_poses)
        print("self._obj_bboxes: ", self._obj_bboxes)
        py_trees.blackboard.Blackboard().set('plane_bboxes', plane_bboxes)
        py_trees.blackboard.Blackboard().set('grocery_bboxes', grocery_bboxes)
        py_trees.blackboard.Blackboard().set('obj_bboxes', self._obj_bboxes)

        return py_trees.Status.SUCCESS

class TeleportObjectAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, obj_name, model_state_key="model_state", free_space_poses_key="free_space_poses"):
        super(TeleportObjectAct, self).__init__(name)
        self._model_state_key = model_state_key
        self._model_state = None
        self._set_model_state = None
        self._set_model_state_pub = None
        self._free_space_poses_key = free_space_poses_key
        self._free_space_poses = None
        self._obj_name = obj_name
        self._obj_index = None

    def setup(self, timeout):
        self._set_model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        return True


    def update(self):
        self._model_state = py_trees.blackboard.Blackboard().get(self._model_state_key)
        self._free_space_poses = py_trees.blackboard.Blackboard().get(self._free_space_poses_key)

        object_move_name = None
        object_move_name = self._obj_name[0:-1] + 'test_' + str(int(self._obj_name[-1]) - 1)
        if self._obj_name[-1] == '0':
            object_move_name = object_move_name[0:-3]
        # print(object_move_name)
        self._obj_index = self._model_state.name.index(object_move_name)
        print("self._obj_index: ", self._obj_index)

        pose_first = self._free_space_poses[0]

        orientation_initial = self._model_state.pose[self._obj_index].orientation

        print("orientation_initial: ", orientation_initial)

        pose_rotated = R.from_quat([0.5, 0.5 ,0.5 ,0.5])

        All_offset = []
        All_offset.append(Vector3(-0.014, -0.014, 0.1))
        All_offset.append(Vector3(-0.023, -0.01, 0.0145))
        All_offset.append(Vector3(-0.032, -0.026, 0.038))
        All_offset.append(Vector3(-0.016, -0.025, 0.095))
        All_offset.append(Vector3(-0.01, 0.084, 0.053))
        All_offset.append(Vector3(-0.0095, -0.017, 0.088))

        All_name = []
        All_name.append("cracker")
        All_name.append("gelatin")
        All_name.append("meat")
        All_name.append("mustard")
        All_name.append("soup")
        All_name.append("sugar")

        pose_real = R.from_quat([
                orientation_initial.x,
                orientation_initial.y,
                orientation_initial.z,
                orientation_initial.w])

        object_name_index = All_name.index(self._obj_name[0:-2])
        # print("object_name_index: ", object_name_index)
        # pose_real = pose_ori * pose_rotated

        # print("pose_real: ", pose_real.as_quat())
        # print("pose_ori: ", pose_ori.as_quat())

        offset = pose_real.apply([All_offset[object_name_index].x, All_offset[object_name_index].y, All_offset[object_name_index].z])
        print("offset: ", offset)


        self._set_model_state = ModelState()
        self._set_model_state.model_name = object_move_name
        self._set_model_state.twist = self._model_state.twist[self._obj_index]
        self._set_model_state.pose = self._model_state.pose[self._obj_index]
        self._set_model_state.pose.position = self._free_space_poses[0].position

        self._set_model_state.pose.position.x = self._set_model_state.pose.position.x - offset[0]
        self._set_model_state.pose.position.y = self._set_model_state.pose.position.y - offset[1]
        self._set_model_state.pose.position.z = self._set_model_state.pose.position.z - offset[2]

        self._set_model_state_pub.publish(self._set_model_state)

        return py_trees.Status.SUCCESS


        # 0 for normal stabilization, 1 for tilt, 2 for flat
class StabilizeObjectAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, obj_name, model_state_key="model_state", free_space_poses_key="free_space_poses", relation_key='relation', mode=0):
        super(StabilizeObjectAct, self).__init__(name)
        self._model_state_key = model_state_key
        self._model_state = None
        self._set_model_state = None
        self._set_model_state_pub = None
        self._free_space_poses_key = free_space_poses_key
        self._free_space_poses = None
        self._obj_name = obj_name
        self._obj_index = None
        self._relation_key = relation_key
        self._relation = None
        self._mode = mode

    def setup(self, timeout):
        self._set_model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        return True


    def update(self):
        self._model_state = py_trees.blackboard.Blackboard().get(self._model_state_key)
        test_grocery_box_size = py_trees.blackboard.Blackboard().get("grocery_box_size")
        print("self._grocery_box_size: ", test_grocery_box_size)
        # print("self._model_state: ", self._model_state)
        self._relation = py_trees.blackboard.Blackboard().get(self._relation_key)
        self._free_space_poses = py_trees.blackboard.Blackboard().get(self._free_space_poses_key)
        if self._obj_name == 'table':
            return py_trees.Status.SUCCESS
        object_move_name = None
        # print("self._obj_name: ", self._obj_name)
        object_move_name = self._obj_name[0:-1] + 'test_' + str(int(self._obj_name[-1]) - 1)
        if self._obj_name[-1] == '0':
            object_move_name = object_move_name[0:-3]

        self._obj_index = self._model_state.name.index(object_move_name)

        orientation_initial = self._model_state.pose[self._obj_index].orientation

        # print("orientation_initial: ", orientation_initial)

        pose_rotated = R.from_quat([0.5, 0.5 ,0.5 ,0.5])

        All_size = []
        All_size.append(Vector3(0.0718, 0.164, 0.213))
        All_size.append(Vector3(0.09, 0.097, 0.029))
        All_size.append(Vector3(0.103, 0.057, 0.084))
        All_size.append(Vector3(0.08, 0.06, 0.19))
        All_size.append(Vector3(0.068, 0.068, 0.102))
        All_size.append(Vector3(0.045, 0.097, 0.174))

        All_offset = []
        All_offset.append(Vector3(-0.014, -0.014, 0.1))
        All_offset.append(Vector3(-0.023, -0.01, 0.0145))
        All_offset.append(Vector3(-0.032, -0.026, 0.038))
        All_offset.append(Vector3(-0.016, -0.025, 0.095))
        All_offset.append(Vector3(-0.01, 0.084, 0.053))
        All_offset.append(Vector3(-0.0095, -0.017, 0.088))

        All_name = []
        All_name.append("cracker")
        All_name.append("gelatin")
        All_name.append("meat")
        All_name.append("mustard")
        All_name.append("soup")
        All_name.append("sugar")

        pose_real = R.from_quat([
                orientation_initial.x,
                orientation_initial.y,
                orientation_initial.z,
                orientation_initial.w])

        object_name_index = All_name.index(self._obj_name[0:-2])

        offset = pose_real.apply([All_offset[object_name_index].x, All_offset[object_name_index].y, All_offset[object_name_index].z])
        # print("offset: ", offset)

        if self._mode == 1 and 'Flat' in self._relation:
            return py_trees.Status.SUCCESS

        self._set_model_state = ModelState()
        self._set_model_state.model_name = object_move_name
        self._set_model_state.twist = self._model_state.twist[self._obj_index]
        self._set_model_state.pose = self._model_state.pose[self._obj_index]

        current_pose = self._model_state.pose[self._obj_index]

        poses_3=[]
        for i in range(17):
            poses_3.append(Pose())
            poses_3[i].position = current_pose.position

        poses_3[0].orientation = Quaternion(0, 0, 0, 1)
        poses_3[1].orientation = Quaternion(0, 0, 0.7071, 0.7071)
        poses_3[2].orientation = Quaternion(0.5, -0.5, 0.5, 0.5)
        poses_3[3].orientation = Quaternion(0, 1, 0, 0)
        poses_3[4].orientation = Quaternion(0.7071, 0, 0, 0.7071)
        poses_3[5].orientation = Quaternion(-0.7071, 0, 0, 0.7071)
        poses_3[6].orientation = Quaternion(0, 0, -0.7071, 0.7071)
        poses_3[7].orientation = Quaternion(-0.5, -0.5, -0.5, 0.5)
        poses_3[8].orientation = Quaternion(0.5, 0.5, -0.5, 0.5)
        poses_3[9].orientation = Quaternion(0.5, 0.5, 0.5, -0.5)
        poses_3[10].orientation = Quaternion(0, 0, 1, 0)
        poses_3[11].orientation = Quaternion(-0.7071, 0, 0.7071, 0)
        poses_3[12].orientation = Quaternion(-0.5, 0.5, 0.5, 0.5)
        poses_3[13].orientation = Quaternion(0, 0, -1, 0)
        poses_3[14].orientation = Quaternion(0, 0, 0.7071, -0.7071)
        poses_3[15].orientation = Quaternion(0, -0.7071, 0, 0.7071)
        poses_3[16].orientation = Quaternion(0.7071, 0, 0.7071, 0)

        dist = [];
        for i in range(len(poses_3)):
            dist.append(srp_md.pose_difference(poses_3[i], current_pose))
        self._set_model_state.pose.orientation = poses_3[dist.index(min(dist))].orientation

        print("poses_3[dist.index(min(dist))].orientation: ", poses_3[dist.index(min(dist))].orientation)
        print("current_pose: ", current_pose.orientation)

        self._set_model_state.twist.linear = Vector3()
        self._set_model_state.twist.angular = Vector3()


        if self._mode == 1 and 'Flat' in self._relation:
            return py_trees.Status.SUCCESS

        if self._mode == 2 and 'Flat' in self._relation:
            self._set_model_state.twist.linear = Vector3()
            self._set_model_state.twist.angular = Vector3()
            self._set_model_state.pose.orientation.x = 0.2705981
            self._set_model_state.pose.orientation.y = -0.2705981
            self._set_model_state.pose.orientation.z = 0.6532815
            self._set_model_state.pose.orientation.w = 0.6532815
            self._set_model_state.pose.orientation = self._model_state.pose[self._obj_index].orientation
            # self._set_model_state.pose.position = self._free_space_poses[0].position
            # self._set_model_state.pose.position.x = self._free_space_poses[0].position.x
            # self._set_model_state.pose.position.y = self._free_space_poses[0].position.y
            self._set_model_state.pose.position.y = self._free_space_poses[0].position.y + All_size[object_name_index].z / 4 * 3
            # self._set_model_state.pose.position.z = self._set_model_state.pose.position.z - 0.1
            self._set_model_state.twist.angular.x = 0.23

        elif self._mode == 3:
            self._set_model_state.twist = self._model_state.twist[self._obj_index]
            self._set_model_state.pose = self._model_state.pose[self._obj_index]
            # self._set_model_state.pose.position.y = self._free_space_poses[0].position.y + All_size[object_name_index].z / 4 * 3

            self._set_model_state.twist.linear = Vector3()
            self._set_model_state.twist.angular = Vector3()
            self._set_model_state.pose.orientation.x = 0.5
            self._set_model_state.pose.orientation.y = -0.5
            self._set_model_state.pose.orientation.z = 0.5
            self._set_model_state.pose.orientation.w = 0.5

        # print('self._set_model_state.pose.position: ', self._set_model_state.pose.position)
        self._set_model_state_pub.publish(self._set_model_state)
        time.sleep(0.05)

        for name in self._model_state.name:
            if 'grocery' in name:
                print("name: ", name)
                belt_index = self._model_state.name.index(name)
                set_belt_state = ModelState()
                set_belt_state.model_name = name
                set_belt_state.twist = self._model_state.twist[belt_index]
                set_belt_state.pose = self._model_state.pose[belt_index]
                set_belt_state.pose.orientation.x = 0
                set_belt_state.pose.orientation.y = 0
                set_belt_state.pose.orientation.z = 0
                set_belt_state.pose.orientation.w = 1
                self._set_model_state_pub.publish(set_belt_state)
                time.sleep(0.02)

        return py_trees.Status.SUCCESS

class ObjectTranslationAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, model_state_key="model_state"):
        super(ObjectTranslationAct, self).__init__(name)
        self._model_state_key = model_state_key
        self._model_state = None
        self._set_model_state = None
        self._set_model_state_pub = None
        self._obj_name = None
        self._obj_index = None
        self._moving_step = 0.006
        self._sleep_time = 0.001
        self._distance_limit = 0.60
        self._y_limit = -0.30

    def setup(self, timeout):
        self._set_model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        return True


    def update(self):
        self._model_state = py_trees.blackboard.Blackboard().get(self._model_state_key)

        self._obj_name = self._model_state.name

        print(self._model_state)

        # test_grocery_box_size = py_trees.blackboard.Blackboard().get("grocery_box_size")
        conveyor_belt_index = None
        for name in self._obj_name:
            if 'belt' in name:
                conveyor_belt_index = self._obj_name.index(name)
                break
        # Assume the orientation is identity
        conveyor_belt_position = self._model_state.pose[conveyor_belt_index].position

        conveyor_belt_size = Vector3(0.6, 5.0, 0.3)
        conveyor_belt_minx = conveyor_belt_position.x - conveyor_belt_size.x / 2
        conveyor_belt_maxx = conveyor_belt_position.x + conveyor_belt_size.x / 2
        conveyor_belt_miny = conveyor_belt_position.y - conveyor_belt_size.y / 2
        conveyor_belt_maxy = conveyor_belt_position.y + conveyor_belt_size.y / 2

        moving_object_name = []

        dis = []
        y_value = []
        fetch_index = self._obj_name.index('fetch')
        fetch_position = self._model_state.pose[fetch_index].position
        for i in range(len(self._obj_name)):
            if 'plane' in self._obj_name[i] or 'fetch' in self._obj_name[i] or 'grocery' in self._obj_name[i] or 'belt' in self._obj_name[i] or 'table' in self._obj_name[i]:
                continue
            else:
                obj_position = self._model_state.pose[i].position
                print("obj_position: ", obj_position)
                dis_temp = np.sqrt((fetch_position.x - obj_position.x) * (fetch_position.x - obj_position.x) + (fetch_position.y - obj_position.y) * (fetch_position.y - obj_position.y))
                if obj_position.x <= conveyor_belt_maxx and obj_position.x >= conveyor_belt_minx and obj_position.y <= conveyor_belt_maxy and obj_position.y >= conveyor_belt_miny:
                    moving_object_name.append(self._obj_name[i])
                    dis.append(dis_temp)
                    y_value.append(obj_position.y)
                    print(i)
        print("moving_object_name: ", moving_object_name)

        if (len(dis) == 0):
            return py_trees.Status.SUCCESS
        while (min(dis) > self._distance_limit and max(y_value) < self._y_limit):
            # dis_index = 0
            for name in moving_object_name:
                object_index = self._obj_name.index(name)
                dis_index = moving_object_name.index(name)
                self._set_model_state = ModelState()
                self._set_model_state.model_name = self._obj_name[object_index]
                self._set_model_state.twist = self._model_state.twist[object_index]
                self._set_model_state.pose = self._model_state.pose[object_index]
                self._set_model_state.pose.position.y = y_value[dis_index] + self._moving_step
                obj_position = self._model_state.pose[object_index].position
                dis[dis_index] = np.sqrt((fetch_position.x - obj_position.x) * (fetch_position.x - obj_position.x) + (fetch_position.y - obj_position.y) * (fetch_position.y - obj_position.y))
                y_value[dis_index] = y_value[dis_index] + self._moving_step
                self._set_model_state_pub.publish(self._set_model_state)
                time.sleep(self._sleep_time)
        time.sleep(self._sleep_time * 500)


        return py_trees.Status.SUCCESS

class ObjectTestInitializationAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, model_state_key="model_state"):
        super(ObjectTestInitializationAct, self).__init__(name)
        self._model_state_key = model_state_key
        self._model_state = None
        self._set_model_state = None
        self._set_model_state_pub = None
        self._sleep_time = 0.005

    def setup(self, timeout):
        self._set_model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        return True


    def update(self):
        self._model_state = py_trees.blackboard.Blackboard().get(self._model_state_key)

        self._obj_name = self._model_state.name


        set_box_state = ModelState()
        set_box_state.model_name = 'cracker_test'
        set_box_state.pose.position.x = 0.45
        set_box_state.pose.position.y = -0.7411
        set_box_state.pose.position.z = 0.595
        set_box_state.pose.orientation = Quaternion(0.5, -0.5, 0.5, 0.5)
        self._set_model_state_pub.publish(set_box_state)
        time.sleep(self._sleep_time)

        set_box_state = ModelState()
        set_box_state.model_name = 'cracker_test_0'
        set_box_state.pose.position.x = 0.45
        set_box_state.pose.position.y = -1.686
        set_box_state.pose.position.z = 0.595
        set_box_state.pose.orientation = Quaternion(0.5, -0.5, 0.5, 0.5)
        self._set_model_state_pub.publish(set_box_state)
        time.sleep(self._sleep_time)

        set_box_state = ModelState()
        set_box_state.model_name = 'gelatin_test'
        set_box_state.pose.position.x = 0.45
        set_box_state.pose.position.y = -1.015
        set_box_state.pose.position.z = 0.55
        set_box_state.pose.orientation = Quaternion(0,0,0,1)
        self._set_model_state_pub.publish(set_box_state)
        time.sleep(self._sleep_time)

        set_box_state = ModelState()
        set_box_state.model_name = 'soup_test'
        set_box_state.pose.position.x = 0.5505
        set_box_state.pose.position.y = -1.012
        set_box_state.pose.position.z = 0.55
        set_box_state.pose.orientation = Quaternion(0,0,0,1)
        self._set_model_state_pub.publish(set_box_state)
        time.sleep(self._sleep_time)

        set_box_state = ModelState()
        set_box_state.model_name = 'soup_test_0'
        set_box_state.pose.position.x = 0.5585
        set_box_state.pose.position.y = -1.112
        set_box_state.pose.position.z = 0.55
        set_box_state.pose.orientation = Quaternion(0,0,0,1)
        self._set_model_state_pub.publish(set_box_state)
        time.sleep(self._sleep_time)

        set_box_state = ModelState()
        set_box_state.model_name = 'meat_test'
        set_box_state.pose.position.x = 0.47
        set_box_state.pose.position.y = -1.099
        set_box_state.pose.position.z = 0.5535
        set_box_state.pose.orientation = Quaternion(0,0,0,1)
        self._set_model_state_pub.publish(set_box_state)
        time.sleep(self._sleep_time)

        set_box_state = ModelState()
        set_box_state.model_name = 'mustard_test'
        set_box_state.pose.position.x = 0.47
        set_box_state.pose.position.y = -1.188
        set_box_state.pose.position.z = 0.55
        set_box_state.pose.orientation = Quaternion(0,0,0,1)
        self._set_model_state_pub.publish(set_box_state)
        time.sleep(self._sleep_time)

        set_box_state = ModelState()
        set_box_state.model_name = 'sugar_test'
        set_box_state.pose.position.x = 0.40
        set_box_state.pose.position.y = -1.925
        set_box_state.pose.position.z = 0.582
        set_box_state.pose.orientation = Quaternion(0.5, -0.5, 0.5, 0.5)
        self._set_model_state_pub.publish(set_box_state)
        time.sleep(self._sleep_time)

        set_box_state = ModelState()
        set_box_state.model_name = 'soup_test_1'
        set_box_state.pose.position.x = 0.5328
        set_box_state.pose.position.y = -1.9626
        set_box_state.pose.position.z = 0.55
        set_box_state.pose.orientation = Quaternion(0,0,0,1)
        self._set_model_state_pub.publish(set_box_state)
        time.sleep(self._sleep_time)

        set_box_state = ModelState()
        set_box_state.model_name = 'soup_test_2'
        set_box_state.pose.position.x = 0.5328
        set_box_state.pose.position.y = -2.06
        set_box_state.pose.position.z = 0.55
        set_box_state.pose.orientation = Quaternion(0,0,0,1)
        self._set_model_state_pub.publish(set_box_state)
        time.sleep(self._sleep_time)

        set_box_state = ModelState()
        set_box_state.model_name = 'meat_test_0'
        set_box_state.pose.position.x = 0.47
        set_box_state.pose.position.y = -2.13
        set_box_state.pose.position.z = 0.5535
        set_box_state.pose.orientation = Quaternion(0,0,0,1)
        self._set_model_state_pub.publish(set_box_state)
        time.sleep(self._sleep_time)

        set_box_state = ModelState()
        set_box_state.model_name = 'meat_test_1'
        set_box_state.pose.position.x = 0.47
        set_box_state.pose.position.y = -2.21
        set_box_state.pose.position.z = 0.5535
        set_box_state.pose.orientation = Quaternion(0,0,0,1)
        self._set_model_state_pub.publish(set_box_state)
        time.sleep(self._sleep_time)

        return py_trees.Status.SUCCESS

def callback(data, args):
        print("data: ", data)

def callback_test(data):
    # print(type(args))
    py_trees.blackboard.Blackboard().set('model_state', data)


class FetchInitializationAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, model_state_key="model_state"):
        super(FetchInitializationAct, self).__init__(name)
        self._model_state_key = model_state_key
        self._model_state = None
        self._set_model_state = None
        self._set_model_state_pub = None
        self._set_model_state_subs = None
        self._sleep_time = 0.005

    def setup(self, timeout):
        self._set_model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        self._set_model_state_subs = rospy.Subscriber('/gazebo/model_states', ModelStates, callback_test)
        return True

    def update(self):
        self._model_state = py_trees.blackboard.Blackboard().get(self._model_state_key)
        self._obj_name = self._model_state.name
        fetch_index = self._obj_name.index('fetch')
        print("self._model_state: ", self._model_state.pose[fetch_index])

        self._obj_name = self._model_state.name

        fetch_index = self._obj_name.index('fetch')
        fetch_pose = self._model_state.pose[fetch_index]
        fetch_twist = self._model_state.twist[fetch_index]

        # print("fetch_pose: ", fetch_pose)

        set_box_state = ModelState()
        set_box_state.model_name = 'fetch'
        set_box_state.pose = fetch_pose
        # set_box_state.pose.position.x = set_box_state.pose.position.x - 1
        set_box_state.twist = fetch_twist
        set_box_state.twist.angular.z = 0.3
        # print("set_box_state: ", set_box_state)
        self._set_model_state_pub.publish(set_box_state)
        # time.sleep(self._sleep_time)

        # time.sleep(1)
        for i in range(1000):
            self._model_state = py_trees.blackboard.Blackboard().get(self._model_state_key)
            fetch_pose = self._model_state.pose[fetch_index]
            print(fetch_pose.orientation.w)
            set_box_state = ModelState()
            set_box_state.model_name = 'fetch'
            set_box_state.pose = fetch_pose
            # set_box_state.pose.position.x = set_box_state.pose.position.x - 1
            set_box_state.twist = fetch_twist
            set_box_state.twist.angular.z = 0.3
            # print("set_box_state: ", set_box_state)
            self._set_model_state_pub.publish(set_box_state)
            if fetch_pose.orientation.w < 0.9:
                set_box_state = ModelState()
                set_box_state.model_name = 'fetch'
                set_box_state.pose = fetch_pose
                # set_box_state.pose.position.x = set_box_state.pose.position.x - 1
                set_box_state.twist = fetch_twist
                set_box_state.twist.angular.z = 0
                # print("set_box_state: ", set_box_state)
                print("sent")
                self._set_model_state_pub.publish(set_box_state)
                break

            time.sleep(self._sleep_time)

        self._model_state = py_trees.blackboard.Blackboard().get(self._model_state_key)
        print("self._model_state: ", self._model_state.pose[fetch_index])
        self._obj_name = self._model_state.name

        fetch_index = self._obj_name.index('fetch')
        fetch_pose = self._model_state.pose[fetch_index]
        fetch_twist = self._model_state.twist[fetch_index]

        # print("fetch_pose: ", fetch_pose)

        set_box_state = ModelState()
        set_box_state.model_name = 'fetch'
        set_box_state.pose = fetch_pose
        # set_box_state.pose.position.x = set_box_state.pose.position.x - 1
        set_box_state.twist = fetch_twist
        set_box_state.twist.angular.z = 5
        # print("set_box_state: ", set_box_state)
        self._set_model_state_pub.publish(set_box_state)
        time.sleep(self._sleep_time)

        return py_trees.Status.SUCCESS

class GroceryBoxInitializationAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, model_state_key="model_state"):
        super(GroceryBoxInitializationAct, self).__init__(name)
        self._model_state_key = model_state_key
        self._model_state = None
        self._set_model_state = None
        self._set_model_state_pub = None
        self._sleep_time = 0.005

    def setup(self, timeout):
        self._set_model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        return True


    def update(self):
        self._model_state = py_trees.blackboard.Blackboard().get(self._model_state_key)

        self._obj_name = self._model_state.name


        # set_box_state = ModelState()
        # set_box_state.model_name = 'grocery_box'
        # set_box_state.pose.position.x = 0.50
        # set_box_state.pose.position.y = 0
        # set_box_state.pose.position.z = 0.57
        # set_box_state.pose.orientation = Quaternion(0,0,0,1)
        # self._set_model_state_pub.publish(set_box_state)
        # time.sleep(self._sleep_time)

        # set_box_state = ModelState()
        # set_box_state.model_name = 'grocery_box_0'
        # set_box_state.pose.position.x = 0.50
        # set_box_state.pose.position.y = 0.51
        # set_box_state.pose.position.z = 0.57
        # set_box_state.pose.orientation = Quaternion(0,0,0,1)
        # self._set_model_state_pub.publish(set_box_state)
        # time.sleep(self._sleep_time)

        set_box_state = ModelState()
        set_box_state.model_name = 'grocery_box_1'
        set_box_state.pose.position.x = 3
        set_box_state.pose.position.y = -4
        set_box_state.pose.position.z = 0 + 0.35
        set_box_state.pose.orientation = Quaternion(0,0,-0.707,0.707)
        self._set_model_state_pub.publish(set_box_state)
        time.sleep(self._sleep_time)

        set_box_state = ModelState()
        set_box_state.model_name = 'grocery_box_0'
        set_box_state.pose.position.x = 3
        set_box_state.pose.position.y = -4
        set_box_state.pose.position.z = 0.03 + 0.35
        set_box_state.pose.orientation = Quaternion(0,0,-0.707,0.707)
        self._set_model_state_pub.publish(set_box_state)
        time.sleep(self._sleep_time)

        set_box_state = ModelState()
        set_box_state.model_name = 'grocery_box'
        set_box_state.pose.position.x = 3
        set_box_state.pose.position.y = -4
        set_box_state.pose.position.z = 0.06 + 0.35
        set_box_state.pose.orientation = Quaternion(0,0,-0.707,0.707)
        self._set_model_state_pub.publish(set_box_state)
        time.sleep(self._sleep_time)

        return py_trees.Status.SUCCESS

class ObjectInitializationAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, model_state_key="model_state"):
        super(ObjectInitializationAct, self).__init__(name)
        self._model_state_key = model_state_key
        self._model_state = None
        self._set_model_state = None
        self._set_model_state_pub = None
        self._obj_name = None
        self._obj_index = None
        self._sleep_time = 0.005
        self._y_value_initial = None
        self._y_step_value = None
        self._x_value = 0.45
        self._z_value = 0.7

    def setup(self, timeout):
        self._set_model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        return True


    def update(self):
        self._model_state = py_trees.blackboard.Blackboard().get(self._model_state_key)

        self._obj_name = self._model_state.name


        # set_box_state = ModelState()
        # set_box_state.model_name = 'grocery_box'
        # # set_box_state.twist = self._model_state.twist[object_index]
        # # set_box_state.pose = self._model_state.pose[object_index]
        # set_box_state.pose.position.x = 0.50
        # set_box_state.pose.position.y = 0
        # set_box_state.pose.position.z = 0.57
        # set_box_state.pose.orientation = Quaternion(0,0,0,1)
        # # self._y_value_initial = self._y_value_initial - self._y_step_value
        # self._set_model_state_pub.publish(set_box_state)
        # time.sleep(self._sleep_time)
        # set_box_state = ModelState()
        # set_box_state.model_name = 'grocery_box_0'
        # # set_box_state.twist = self._model_state.twist[object_index]
        # # set_box_state.pose = self._model_state.pose[object_index]
        # set_box_state.pose.position.x = 0.50
        # set_box_state.pose.position.y = 0.51
        # set_box_state.pose.position.z = 0.57
        # set_box_state.pose.orientation = Quaternion(0,0,0,1)
        # # self._y_value_initial = self._y_value_initial - self._y_step_value
        # self._set_model_state_pub.publish(set_box_state)
        # time.sleep(self._sleep_time)


        conveyor_belt_index = None
        for name in self._obj_name:
            if 'belt' in name:
                conveyor_belt_index = self._obj_name.index(name)
                break
        if conveyor_belt_index == None:
            return py_trees.Status.SUCCESS

        # Assume the orientation is identity
        conveyor_belt_position = self._model_state.pose[conveyor_belt_index].position

        conveyor_belt_size = Vector3(0.6, 5.0, 0.3)
        conveyor_belt_minx = conveyor_belt_position.x - conveyor_belt_size.x / 2
        conveyor_belt_maxx = conveyor_belt_position.x + conveyor_belt_size.x / 2
        conveyor_belt_miny = conveyor_belt_position.y - conveyor_belt_size.y / 2
        conveyor_belt_maxy = conveyor_belt_position.y + conveyor_belt_size.y / 2

        print(self._model_state)

        moving_object_name = []
        y_value = []

        fetch_index = self._obj_name.index('fetch')
        fetch_position = self._model_state.pose[fetch_index].position
        for i in range(len(self._obj_name)):
            if 'plane' in self._obj_name[i] or 'fetch' in self._obj_name[i] or 'grocery' in self._obj_name[i] or 'belt' in self._obj_name[i] or 'table' in self._obj_name[i]:
                continue
            else:
                obj_position = self._model_state.pose[i].position
                print("obj_position: ", obj_position)
                dis_temp = np.sqrt((fetch_position.x - obj_position.x) * (fetch_position.x - obj_position.x) + (fetch_position.y - obj_position.y) * (fetch_position.y - obj_position.y))
                if obj_position.x <= conveyor_belt_maxx and obj_position.x >= conveyor_belt_minx and obj_position.y <= conveyor_belt_maxy and obj_position.y >= conveyor_belt_miny:
                    moving_object_name.append(self._obj_name[i])
                    y_value.append(obj_position.y)
                    print(i)
        print("moving_object_name: ", moving_object_name)
        # Cracker x value : 0.45
        # grocery box x value: 0.5

        if len(moving_object_name) == 0:
            return py_trees.Status.SUCCESS

        moving_object_name_sorted = []

        for i in range(len(moving_object_name)):
            max_index = y_value.index(max(y_value))
            moving_object_name_sorted.append(moving_object_name[max_index])
            y_value[max_index] = -10000

        self._y_step_value = conveyor_belt_size.y / (len(moving_object_name_sorted) + 1)
        if self._y_step_value > 0.5:
            self._y_step_value = 0.5
        self._y_value_initial = conveyor_belt_maxy - self._y_step_value

        for i in range(len(moving_object_name_sorted)):
            object_index = self._obj_name.index(moving_object_name_sorted[i])
            self._set_model_state = ModelState()
            self._set_model_state.model_name = self._obj_name[object_index]
            self._set_model_state.twist = self._model_state.twist[object_index]
            self._set_model_state.pose = self._model_state.pose[object_index]
            self._set_model_state.pose.position.x = self._x_value
            self._set_model_state.pose.position.y = self._y_value_initial
            self._set_model_state.pose.position.z = self._z_value
            if 'cracker' in moving_object_name_sorted[i] or 'sugar' in moving_object_name_sorted[i]:
                self._set_model_state.pose.orientation = Quaternion(0.5, -0.5, 0.5, 0.5)
            elif 'soup' in moving_object_name_sorted[i]:
                self._set_model_state.pose.orientation = Quaternion(0, 0, 1, 0)
            else:
                self._set_model_state.pose.orientation = Quaternion(0, 0, 0, 1)
            self._y_value_initial = self._y_value_initial - self._y_step_value
            self._set_model_state_pub.publish(self._set_model_state)
            time.sleep(self._sleep_time)



        return py_trees.Status.SUCCESS

class GrasplocAct(py_trees_ros.actions.ActionClient):
    def __init__(self, name, in_pc_key, output_key='grasploc'):
        super(GrasplocAct, self).__init__(
            name=name,
            action_spec=GrasplocAction,
            action_goal=GrasplocGoal(),
            action_namespace='/grasploc'
        )
        self._in_pc_key = in_pc_key
        self._pc = None
        self._output_key = output_key
        self._test_bbox = None

    def initialise(self):
        super(GrasplocAct, self).initialise()

        # Get the point cloud
        blackboard = py_trees.blackboard.Blackboard()
        self._pc = blackboard.get(self._in_pc_key)
        self.action_goal.input_pc = self._pc
        self.action_goal.viewpoint.x = 0.159
        self.action_goal.viewpoint.y = 0.05
        self.action_goal.viewpoint.z = 1.414

    def update(self):
        """
        Check only to see whether the underlying action server has
        succeeded, is running, or has cancelled/aborted for some reason and
        map these to the usual behaviour return states.
        """
        self.logger.debug("{0}.update()".format(self.__class__.__name__))

        if not self.action_client:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            return py_trees.Status.INVALID
        # Ensure we have a pc
        if self.action_goal.input_pc is None:
            return py_trees.Status.FAILURE
        # pity there is no 'is_connected' api like there is for c++
        if not self.sent_goal:
            self.action_client.send_goal(self.action_goal)
            self.sent_goal = True
            self.feedback_message = "sent goal to the action server"
            return py_trees.Status.RUNNING
        if self.action_client.get_state() == actionlib_msgs.GoalStatus.ABORTED:
            return py_trees.Status.FAILURE

        result = self.action_client.get_result()
        if result:
            self.feedback_message = "goal reached"
            # Write result to blackboard
            if self._output_key is not None:
                blackboard = py_trees.blackboard.Blackboard()
                blackboard.set(self._output_key, result)
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = "moving"
            return py_trees.Status.RUNNING

class OffsetPoses(py_trees.behaviour.Behaviour):
    """!
    @brief      Add an offset to an iterable of poses.
    """
    def __init__(self, name, in_poses_key, out_poses_key, offset, debug=False):
        """!
        @brief      Constructs a new instance.

        @param      name           Behavior name
        @param      in_poses_key   In poses blackboard key
        @param      out_poses_key  The out poses blackboard key
        @param      offset         The offset in the poses frame of reference as will be used to create a
                                   numpy.array(offset)
        """
        super(OffsetPoses, self).__init__(name)
        self._in_poses_key = in_poses_key
        self._out_poses_key = out_poses_key
        self._offset = offset
        self._orig_pub = None
        self._new_pub = None
        self._debug = debug

    def setup(self, timeout):
        if self._debug:
            self._orig_pub = rospy.Publisher('/original_grasploc_poses', PoseArray, queue_size=10)
            self._new_pub = rospy.Publisher('/new_grasploc_poses', PoseArray, queue_size=10)
        return True

    def update(self):
        blackboard = py_trees.blackboard.Blackboard()
        poses = blackboard.get(self._in_poses_key)
        if poses is None:
            return py_trees.Status.FAILURE

        # Handle iterables and single poses
        try:
            pose_iter = iter(poses)
        except TypeError as e:
            pose_iter = iter([poses])

        offset_poses = []
        for pose in pose_iter:
            # Apply offset in poses frame of reference
            tf = R.from_quat([
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w])
            offset = tf.apply(self._offset)
            offset_pose = deepcopy(pose)
            offset_pose.position.x = pose.position.x + offset[0]
            offset_pose.position.y = pose.position.y + offset[1]
            offset_pose.position.z = pose.position.z + offset[2]
            offset_poses.append(offset_pose)
        object_bbox = blackboard.get('crop_box')
        cracker_box = object_bbox
        offset = 0.254
        offset_upper_offset = min(0.025, cracker_box.size.y / 4)
        offset_bottom_offset = max(0.02, cracker_box.size.y / 2)
        poses_filtered_half = list(filter(lambda x:(x.position.z - offset + offset_upper_offset)<= cracker_box.center.position.z + cracker_box.size.y / 2,
                                     offset_poses)
                             )
        poses_filtered_offset = list(filter(lambda x:x.position.z - offset >= cracker_box.center.position.z - cracker_box.size.y / 2 + offset_bottom_offset,
                                     poses_filtered_half)
                             )
        if self._debug:
            orig = PoseArray()
            new = PoseArray()
            orig.header.frame_id = 'base_link'
            new.header.frame_id = 'base_link'
            orig.poses = poses
            new.poses = offset_poses
            # print(orig)
            self._orig_pub.publish(orig)
            self._new_pub.publish(new)

        # blackboard.set(self._out_poses_key, offset_poses)
        blackboard.set(self._out_poses_key, offset_poses)
        return py_trees.Status.SUCCESS

def GrasplocPickAct(name, poses_key):
    temp_wall_pose = Pose()
    temp_wall_pose.position.x = 0.65
    temp_wall_pose.position.y = 0.0
    temp_wall_pose.position.z = 0.0
    temp_wall_pose.orientation.w = 1.0
    temp_wall_pose.orientation.x = 0.0
    temp_wall_pose.orientation.y = 0.0
    temp_wall_pose.orientation.z = 0.0

    pre_grasp = TransformStamped()
    pre_grasp.header.frame_id = 'base_link'
    pre_grasp.transform.translation.x = 0.4
    pre_grasp.transform.translation.y = -0.2
    pre_grasp.transform.translation.z = 0.45
    pre_grasp.transform.rotation.x = 0.0
    pre_grasp.transform.rotation.y = 1.0
    pre_grasp.transform.rotation.z = 0.0
    pre_grasp.transform.rotation.w = 0.0

    root = py_trees.composites.Sequence(name='seq_{}'.format(name))
    root.add_children([
        AddCollisionBoxAct('act_{}_temp_front_wall'.format(name), box_name='{}_temp_front_wall'.format(name),
                           box_pose=temp_wall_pose, box_size=[0.05, 10, 10]),
        TuckWithCondBehavior('act_{}_start_tucked'.format(name), tuck_pose='tuck'),
        OpenGripperAct('act_{}_open_gripper'.format(name)),
        MoveToRelativePoseAct('act_{}_move_pre_grasp'.format(name), pre_grasp),
        RemoveCollisionBoxAct('act_{}_remove_temp_wall', box_name='{}_temp_front_wall'.format(name)),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        MoveToFirstPoseAct('act_{}_pick_grasploc'.format(name), poses_key=poses_key)
    ])
    return root

class ChooseGrasplocObjAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, bbox_key='obj_bboxes', object_index=None, relative_object_index=None, relation='None', crop_box_key='crop_box', obj_dim_key = 'object_dim',
                 relative_obj_bbox_key = 'relative_bbox', relation_key = 'relation', distance_key = 'distance',
                 inhand_collision_object_key = 'inhand_collision_object'):
        super(ChooseGrasplocObjAct, self).__init__(name)
        self._bbox_key = bbox_key
        self._crop_box_key = crop_box_key
        self._obj_dim_key = obj_dim_key
        self._relative_obj_bbox_key = relative_obj_bbox_key
        self._distance_key = distance_key
        self._distance = 0.03
        self._relation_key = relation_key
        self._relation = None
        self._object_index = object_index
        self._relative_object_index = relative_object_index
        self._relation = relation
        self._inhand_collision_object_key = inhand_collision_object_key
        self._name = name

    def update(self):
        blackboard = py_trees.blackboard.Blackboard()
        bboxes = blackboard.get(self._bbox_key)
        if bboxes is None or len(bboxes) == 0:
            return py_trees.Status.FAILURE

        if self._relation == None or self._relation == 'Random' or self._object_index == 'table':
            self._relation = 'None'
            self._distance = 0
        elif self._relation == 'Near':
            pass

        print("self._object_index: ", self._object_index)
        print("bboxes[self._object_index]: ", bboxes[self._object_index])

        blackboard.set(self._crop_box_key, bboxes[self._object_index])
        size_box = bboxes[self._object_index].size
        collision_box_list = ['inhand_collision_object', bboxes[self._object_index].center, [size_box.x, size_box.y, size_box.z]]


        blackboard.set(self._inhand_collision_object_key, collision_box_list)

        if not self._relative_object_index == 'table':
            blackboard.set(self._relative_obj_bbox_key, bboxes[self._relative_object_index])
        blackboard.set(self._relation_key, self._relation)
        blackboard.set(self._obj_dim_key, bboxes[self._object_index].size)
        blackboard.set(self._distance_key, self._distance)

        return py_trees.Status.SUCCESS

class PushPoseGeneration(py_trees.behaviour.Behaviour):
    def __init__(self, name, grocery_bboxes_key='grocery_bboxes', push_pose_key='push_pose', grocery_box_index_key='grocery_box_index'):
        super(PushPoseGeneration, self).__init__(name)
        self._grocery_bboxes_key = grocery_bboxes_key
        self._grocery_bboxes = None
        self._push_pose_key = push_pose_key
        self._grocery_bbox = None
        self._grocery_box_index = None
        self._grocery_box_index_key = grocery_box_index_key

    def update(self):
        blackboard = py_trees.blackboard.Blackboard()
        self._grocery_bboxes = blackboard.get(self._grocery_bboxes_key)
        test_gripper_poses = []

        if not len(self._grocery_bboxes) == 0:
            dis = []
            for grocery_bbox in self._grocery_bboxes:
                grocery_position = grocery_bbox.center.position
                dis_temp = np.sqrt(grocery_position.x * grocery_position.x + grocery_position.y * grocery_position.y)
                dis.append(dis_temp)
            self._grocery_bbox = self._grocery_bboxes[dis.index(min(dis))]
            self._grocery_box_index = dis.index(min(dis))

        gripper_pose = Pose()
        gripper_pose.orientation.x = 0
        gripper_pose.orientation.y = 0
        gripper_pose.orientation.z = 0
        gripper_pose.orientation.w = 1

        gripper_pose.position.x = self._grocery_bbox.center.position.x + self._grocery_bbox.size.x / 2 - 0.05
        gripper_pose.position.y = self._grocery_bbox.center.position.y
        gripper_pose.position.z = self._grocery_bbox.center.position.z + self._grocery_bbox.size.z / 2 + 0.02
        test_gripper_poses.append(gripper_pose)

        blackboard.set(self._push_pose_key, test_gripper_poses)
        blackboard.set(self._grocery_box_index_key, self._grocery_box_index)
        return py_trees.Status.SUCCESS

class GrabBoxPoseGeneration(py_trees.behaviour.Behaviour):
    def __init__(self, name, grocery_bboxes_key='grocery_bboxes', grab_pose_key='grab_pose', grocery_box_index_key='grocery_box_index'):
        super(GrabBoxPoseGeneration, self).__init__(name)
        self._grocery_bboxes_key = grocery_bboxes_key
        self._grocery_bboxes = None
        self._grab_pose_key = grab_pose_key
        self._grocery_bbox = None
        self._grocery_box_index = None
        self._grocery_box_index_key = grocery_box_index_key

    def update(self):
        blackboard = py_trees.blackboard.Blackboard()
        self._grocery_bboxes = blackboard.get(self._grocery_bboxes_key)
        test_gripper_poses = []

        if not len(self._grocery_bboxes) == 0:
            dis = []
            for grocery_bbox in self._grocery_bboxes:
                grocery_position = grocery_bbox.center.position
                dis_temp = np.sqrt(grocery_position.x * grocery_position.x + grocery_position.y * grocery_position.y)
                dis.append(dis_temp)
            self._grocery_bbox = self._grocery_bboxes[dis.index(min(dis))]
            self._grocery_box_index = dis.index(min(dis))

        gripper_pose = Pose()
        gripper_pose.orientation.x = 0
        gripper_pose.orientation.y = 0
        gripper_pose.orientation.z = 0
        gripper_pose.orientation.w = 1

        gripper_pose.position.x = self._grocery_bbox.center.position.x - self._grocery_bbox.size.x / 2 + 0.05
        gripper_pose.position.y = self._grocery_bbox.center.position.y - self._grocery_bbox.size.y / 2 + 0.13
        gripper_pose.position.z = self._grocery_bbox.center.position.z + self._grocery_bbox.size.z / 2 - 0.02
        test_gripper_poses.append(gripper_pose)

        blackboard.set(self._grab_pose_key, test_gripper_poses)
        blackboard.set(self._grocery_box_index_key, self._grocery_box_index)
        return py_trees.Status.SUCCESS

class GraspPoseGeneration(py_trees.behaviour.Behaviour):
    def __init__(self, name, object_bbox_key='crop_box', grasp_points_key='grasp_pose', x_axis_key='x_normal', grocery_bboxes_key='grocery_bboxes'):
        super(GraspPoseGeneration, self).__init__(name)
        self._object_bbox_key = object_bbox_key
        self._grasp_points_key = grasp_points_key
        blackboard = py_trees.blackboard.Blackboard()
        self._test_bbox = blackboard.get(self._object_bbox_key)
        self._x_axis_key = x_axis_key
        self._grocery_bboxes_key = grocery_bboxes_key
        self._grocery_bboxes = None

    def update(self):
        blackboard = py_trees.blackboard.Blackboard()
        self._test_bbox = blackboard.get(self._object_bbox_key)
        self._grocery_bboxes = blackboard.get(self._grocery_bboxes_key)
        print(self._test_bbox)
        object_bbox = self._test_bbox
        object_pose = object_bbox.center
        test_pose = Pose(object_pose.position, object_pose.orientation)

        orientation_offset = []
        orientation_offset.append(R.from_quat([0,0.258819,0,0.9659258]))
        orientation_offset.append(R.from_quat([0,-0.258819,0,0.9659258]))

        gripper_ori_relative = []
        gripper_ori_relative.append(R.from_quat([0.7071068,0,0,0.7071068]))
        gripper_ori_relative.append(R.from_quat([-0.7071068,0,0,0.7071068]))

        gripper_ori_relative.append(R.from_quat([0, 0.7071068, 0,0.7071068]))
        gripper_ori_relative.append(R.from_quat([0, -0.7071068,0,0.7071068]))

        gripper_ori_relative.append(R.from_quat([0.5,0.5,0.5,0.5]))
        gripper_ori_relative.append(R.from_quat([0.5,-0.5,-0.5,0.5]))
        gripper_ori_relative.append(R.from_quat([-0.5,0.5,-0.5,0.5]))
        gripper_ori_relative.append(R.from_quat([-0.5,-0.5,0.5,0.5]))

        # gripper_ori_relative.append(R.from_quat([0,0.7071068,-0.7071068,0]))
        # gripper_ori_relative.append(R.from_quat([0,0.7071068,0.7071068,0]))

        gripper_grasp_length = min(0.017, object_bbox.size.y / 4)
        gripper_offset = []
        gripper_offset.append([-object_bbox.size.x / 2 + gripper_grasp_length, 0, 0])
        gripper_offset.append([-object_bbox.size.x / 2 + gripper_grasp_length, 0, 0])
        gripper_offset.append([-object_bbox.size.y / 2 + gripper_grasp_length, 0, 0])
        gripper_offset.append([-object_bbox.size.y / 2 + gripper_grasp_length, 0, 0])
        gripper_offset.append([-object_bbox.size.y / 2 + gripper_grasp_length, 0, 0])
        gripper_offset.append([-object_bbox.size.y / 2 + gripper_grasp_length, 0, 0])
        gripper_offset.append([-object_bbox.size.x / 2 + gripper_grasp_length, 0, 0])
        gripper_offset.append([-object_bbox.size.x / 2 + gripper_grasp_length, 0, 0])
        # gripper_offset.append([-object_bbox.size.x / 2 + gripper_grasp_length, 0, 0])
        # gripper_offset.append([-object_bbox.size.x / 2 + gripper_grasp_length, 0, 0])

        crack_ori = R.from_quat([
                object_pose.orientation.x,
                object_pose.orientation.y,
                object_pose.orientation.z,
                object_pose.orientation.w])

        test_gripper_poses = []
        x_axises = []
        for i in range(len(gripper_ori_relative)):
            rotated_gripper = crack_ori * gripper_ori_relative[i]
            print('rotated_gripper', rotated_gripper.as_quat())
            offset = rotated_gripper.apply(gripper_offset[i])
            gripper_pose = Pose()
            crack_ori_quat = rotated_gripper.as_quat()
            gripper_pose.orientation.x = crack_ori_quat[0]
            gripper_pose.orientation.y = crack_ori_quat[1]
            gripper_pose.orientation.z = crack_ori_quat[2]
            gripper_pose.orientation.w = crack_ori_quat[3]

            x_axis = rotated_gripper.apply([1,0,0])
            x_axises.append(x_axis)

            gripper_pose.position.x = object_pose.position.x + offset[0]
            gripper_pose.position.y = object_pose.position.y + offset[1]
            gripper_pose.position.z = object_pose.position.z + offset[2]
            test_gripper_poses.append(gripper_pose)

        for i in range(len(gripper_ori_relative)):
            gripper_pose_30_offset = Pose()
            gripper_pose_minus_30_offset = Pose()

            ori_before_offset = R.from_quat([
                test_gripper_poses[i].orientation.x,
                test_gripper_poses[i].orientation.y,
                test_gripper_poses[i].orientation.z,
                test_gripper_poses[i].orientation.w])

            rotated_gripper_30_offset = ori_before_offset * orientation_offset[0]
            rotated_gripper_minus_30_offset = ori_before_offset * orientation_offset[1]

            gripper_pose_30_offset.position = test_gripper_poses[i].position
            gripper_pose_minus_30_offset.position = test_gripper_poses[i].position
            gripper_pose_30_offset.position.z -= 0.001
            gripper_pose_minus_30_offset.position.z -= 0.001

            rotated_gripper_30_offset_quat = rotated_gripper_30_offset.as_quat()
            rotated_gripper_minus_30_offset_quat = rotated_gripper_minus_30_offset.as_quat()

            gripper_pose_30_offset.orientation.x = rotated_gripper_30_offset_quat[0]
            gripper_pose_30_offset.orientation.y = rotated_gripper_30_offset_quat[1]
            gripper_pose_30_offset.orientation.z = rotated_gripper_30_offset_quat[2]
            gripper_pose_30_offset.orientation.w = rotated_gripper_30_offset_quat[3]

            gripper_pose_minus_30_offset.orientation.x = rotated_gripper_minus_30_offset_quat[0]
            gripper_pose_minus_30_offset.orientation.y = rotated_gripper_minus_30_offset_quat[1]
            gripper_pose_minus_30_offset.orientation.z = rotated_gripper_minus_30_offset_quat[2]
            gripper_pose_minus_30_offset.orientation.w = rotated_gripper_minus_30_offset_quat[3]

            x_axis_30 = rotated_gripper_30_offset.apply([1,0,0])
            x_axis_minus_30 = rotated_gripper_minus_30_offset.apply([1,0,0])
            x_axises.append(x_axis_30)
            x_axises.append(x_axis_minus_30)
            print(x_axis_30)
            # test_gripper_poses.append(gripper_pose_30_offset)
            # test_gripper_poses.append(gripper_pose_minus_30_offset)
        # print(x_axises)

        blackboard.set(self._grasp_points_key, test_gripper_poses)
        blackboard.set(self._x_axis_key, x_axises)
        return py_trees.Status.SUCCESS

class FilterGrasplocPoints(py_trees.behaviour.Behaviour):
    def __init__(self, name, grasp_points_key='grasp_pose', normal_key='x_normal', filtered_grasp_points_key='filtered_grasploc'):
        super(FilterGrasplocPoints, self).__init__(name)
        self._grasp_points_key = grasp_points_key
        self._normal_key = normal_key
        self._filtered_grasp_points_key = filtered_grasp_points_key
        # self._center_axis = np.array([-1, 0, 1]) / np.sqrt(2) # 45deg from vertical towards robot
        # self._min_cos_theta = np.cos(np.pi / 4) # cos(x) -> +- x rads
        self._center_axis = np.array([0, 0, 1]) # 0deg from vertical towards robot
        self._min_cos_theta = np.cos(np.pi / 4) # cos(x) -> +- x rads

        # need to be tuned
    def update(self):
        blackboard = py_trees.blackboard.Blackboard()
        test_gripper_poses = blackboard.get(self._grasp_points_key)
        x_axises = blackboard.get(self._normal_key)

        if test_gripper_poses is None:
            return py_trees.Status.FAILURE

        poses_z_value = []
        test_gripper_poses_filtered = []
        # print("self._min_cos_theta", self._min_cos_theta)
        # print("self._min_cos_theta - 1", self._min_cos_theta - 1)
        for i in range(len(test_gripper_poses)):
            norm = np.array([x_axises[i][0], x_axises[i][1], x_axises[i][2]])
            norm = norm / np.linalg.norm(norm)
            # print(norm, np.arccos(np.dot(norm, self._center_axis)) * 180 / np.pi)
            if np.dot(norm, self._center_axis) > self._min_cos_theta or np.dot(norm, self._center_axis) < -self._min_cos_theta:
                # print('good')
                test_gripper_poses_filtered.append(test_gripper_poses[i])
                poses_z_value.append(test_gripper_poses[i].position.z)
            else:
                # print('bad')
                pass

        if len(test_gripper_poses_filtered) == 0:
            return py_trees.Status.FAILURE
        test_gripper_poses_filtered.sort(key=lambda x:x.position.z, reverse = True)
        # TODO(Kevin): Get which object to grab from the plan
        blackboard.set(self._filtered_grasp_points_key, test_gripper_poses_filtered)
        return py_trees.Status.SUCCESS

class GetTableAct(py_trees_ros.actions.ActionClient):
    def __init__(self, name, pc_key = 'depth_downsampled', *argv, **kwargs):
        super(GetTableAct, self).__init__(
            name=name,
            action_spec=GetTableAction,
            action_goal=GetTableGoal(),
            action_namespace='get_table',
            *argv,
            **kwargs
        )
        self._pc_key = pc_key

    def initialise(self):
        self.action_goal.points = py_trees.blackboard.Blackboard().get(self._pc_key)

    def update(self):
        if not self.action_client:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            return py_trees.Status.INVALID
        if not self.sent_goal:
            self.action_client.send_goal(self.action_goal)
            self.sent_goal = True
            self.feedback_message = "sent goal to the action server"
            return py_trees.Status.RUNNING
        self.feedback_message = self.action_client.get_goal_status_text()
        if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
                                              actionlib_msgs.GoalStatus.PREEMPTED]:
            return py_trees.Status.FAILURE
        result = self.action_client.get_result()
        if result:
            print("result.plane_bboxes: ")
            print(type(result.plane_bboxes))
            py_trees.blackboard.Blackboard().set('plane_bboxes', result.plane_bboxes)
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = self.override_feedback_message_on_running
            return py_trees.Status.RUNNING

class GetStackPoseAct(py_trees_ros.actions.ActionClient):
    def __init__(self, name, relative_bbox_key = 'relative_bbox', relation_key = 'relation', obj_dim_key = 'object_dim', *argv, **kwargs):
        super(GetStackPoseAct, self).__init__(
            name=name,
            action_spec=GetStackPoseAction,
            action_goal=GetStackPoseGoal(),
            action_namespace='GetStackPose',
            *argv,
            **kwargs
        )
        self._relative_bbox_key = relative_bbox_key
        self._relative_object_bbox = None
        self._obj_dim_key = obj_dim_key
        self._relation_key = relation_key
        self._relation = None
        self._object_dim = None

    def initialise(self):
        self._relative_object_bbox = py_trees.blackboard.Blackboard().get(self._relative_bbox_key)
        if self._relative_object_bbox == None:
            self._relative_object_bbox = BoundingBox3D()
        self.action_goal.bot_pose = self._relative_object_bbox.center
        print("self.action_goal.bot_pose: ", self.action_goal.bot_pose)
        self.action_goal.bot_dim = self._relative_object_bbox.size
        self.action_goal.dim = py_trees.blackboard.Blackboard().get(self._obj_dim_key)
        self._object_dim = self.action_goal.dim
        self._relation = py_trees.blackboard.Blackboard().get(self._relation_key)

    def update(self):
        if not self.action_client:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            return py_trees.Status.INVALID
        if not self.sent_goal:
            self.action_client.send_goal(self.action_goal)
            self.sent_goal = True
            self.feedback_message = "sent goal to the action server"
            return py_trees.Status.RUNNING
        self.feedback_message = self.action_client.get_goal_status_text()
        if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
                                              actionlib_msgs.GoalStatus.PREEMPTED]:
            return py_trees.Status.FAILURE
        if not 'Stacking' in self._relation:
            return py_trees.Status.SUCCESS
        result = self.action_client.get_result()
        if result:
            if 'Flat' in self._relation:
                result.pose.orientation.x = 0.2705981
                result.pose.orientation.y = 0.6532815
                result.pose.orientation.z = 0.2705981
                result.pose.orientation.w = 0.6532815
                result.pose.position.y = result.pose.position.y
            if 'Flat' not in self._relation:
                result.pose.orientation.x = 0
                result.pose.orientation.y = 0.7071
                result.pose.orientation.z = 0
                result.pose.orientation.w = 0.7071
                result.pose.position.y = result.pose.position.y
            result.pose.position.z = result.pose.position.z + self.action_goal.dim.y / 3.5
            # result.pose.position.z = result.pose.position.z + self.action_goal.dim.y
            print("result.pose", result.pose)
            upper_poses = []
            upper_poses.append(result.pose)
            # upper_poses[0].position.y = upper_poses[0].position.y - self._object_dim.y / 4
            print("self._object_dim: ", self._object_dim)
            py_trees.blackboard.Blackboard().set('free_space_poses', upper_poses)
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = self.override_feedback_message_on_running
            return py_trees.Status.RUNNING

class FreeSpaceFinderAct(py_trees_ros.actions.ActionClient):
    def __init__(self, name, obj_bbox_key='crop_box', plane_bboxes_key = 'plane_bboxes', relative_bbox_key = 'relative_bbox',
                 relation_key = 'relation', grocery_box_size_key='grocery_box_size',
                 grocery_bboxes_key = 'grocery_bboxes', distance_key = 'distance', object_index=None, *argv, **kwargs):
        super(FreeSpaceFinderAct, self).__init__(
            name=name,
            action_spec=FreeSpaceFinderAction,
            action_goal=FreeSpaceFinderGoal(),
            action_namespace='free_space_finder',
            *argv,
            **kwargs
        )
        self._obj_bbox_key = obj_bbox_key
        self._obj_bbox = None
        self._plane_bboxes = None
        self._plane_bboxes_key = plane_bboxes_key
        self._grasp_pose = None
        self._distance_key = distance_key
        self._relative_bbox_key = relative_bbox_key
        self._relation_key = relation_key
        self._relation = None
        self._object_index = object_index
        self._grocery_box_size_key = grocery_box_size_key
        self._grocery_box_size = None
        self._plane_bbox = None
        self._grocery_bbox = BoundingBox3D()
        self._grocery_bboxes_key = grocery_bboxes_key
        self._grocery_bboxes = None
        self._test_back_dis = 0.007

    def initialise(self):
        self.action_goal.points = py_trees.blackboard.Blackboard().get('depth_downsampled')
        self._obj_bbox = py_trees.blackboard.Blackboard().get(self._obj_bbox_key)
        self._grocery_bboxes = py_trees.blackboard.Blackboard().get(self._grocery_bboxes_key)
        self._obj_dim = self._obj_bbox.size
        grasp_poses = py_trees.blackboard.Blackboard().get('filtered_grasploc')
        self.action_goal.distance = py_trees.blackboard.Blackboard().get(self._distance_key)
        self.action_goal.relative_obj_bbox = py_trees.blackboard.Blackboard().get(self._relative_bbox_key)
        if self.action_goal.relative_obj_bbox == None:
            self.action_goal.relative_obj_bbox = BoundingBox3D()
        test_bbox = py_trees.blackboard.Blackboard().get(self._relative_bbox_key)
        self._grasp_pose = grasp_poses[0]
        self._relation = py_trees.blackboard.Blackboard().get(self._relation_key)
        self._grocery_box_size = py_trees.blackboard.Blackboard().get(self._grocery_box_size_key)


        if self._obj_bbox is None:
            self.action_goal.obj_bbox = BoundingBox3D()
        else:
             self.action_goal.obj_bbox = self._obj_bbox
        self.action_goal.relation = 'test'
        self._plane_bboxes = py_trees.blackboard.Blackboard().get(self._plane_bboxes_key)
        # self._grocery_bbox.size = Vector3(self._grocery_box_size.x, self._grocery_box_size.y, self._grocery_box_size.z)
        self._grocery_bbox.size = deepcopy(self._grocery_box_size)
        if len(self._plane_bboxes) == 0:
            self.action_goal.plane_bbox = BoundingBox3D()
        else:
            if len(self._plane_bboxes) > 1:
                if self._plane_bboxes[0].center.position.z > self._plane_bboxes[1].center.position.z:
                    self.action_goal.plane_bbox = self._plane_bboxes[1]
                    self._grocery_bbox.center = self._plane_bboxes[1].center
                    self._plane_bbox = self._plane_bboxes[1]
                    # self._grocery_bbox.size.z = self._plane_bboxes[1].size.z
                else:
                    self.action_goal.plane_bbox = self._plane_bboxes[0]
                    self._grocery_bbox.center = self._plane_bboxes[0].center
                    self._plane_bbox = self._plane_bboxes[0]
                    # self._grocery_bbox.size.z = self._plane_bboxes[0].size.z
            else:
                self.action_goal.plane_bbox = self._plane_bboxes[0]
                self._grocery_bbox.center = deepcopy(self._plane_bboxes[0].center)
                self._plane_bbox = self._plane_bboxes[0]

        if not len(self._grocery_bboxes) == 0:
            dis = []
            for grocery_bbox in self._grocery_bboxes:
                grocery_position = grocery_bbox.center.position
                dis_temp = np.sqrt(grocery_position.x * grocery_position.x + grocery_position.y * grocery_position.y)
                dis.append(dis_temp)
            self._grocery_bbox = deepcopy(self._grocery_bboxes[dis.index(min(dis))])

        self._grocery_bbox.center.position.x = self._grocery_bbox.center.position.x
        self._grocery_bbox.center.position.y = self._grocery_bbox.center.position.y
        print("self._grocery_box_size: ", self._grocery_box_size)
        self._grocery_bbox.size.z = self._plane_bboxes[0].size.z
        print("self._grocery_box_size: ", self._grocery_box_size)
        self.action_goal.plane_bbox = self._grocery_bbox
        self.action_goal.plane_bbox.center.position.z = self._plane_bbox.center.position.z
        obj_bboxes = py_trees.blackboard.Blackboard().get('obj_bboxes')

        grocery_box_manual_deduction = Vector3(0.0001, 0, 0)

        # self.action_goal.plane_bbox.center.position.x = self.action_goal.plane_bbox.center.position.x - grocery_box_manual_deduction.x
        # print("self.action_goal.plane_bbox: ", self.action_goal.plane_bbox)
        self.action_goal.plane_bbox.center.position.x = self.action_goal.plane_bbox.center.position.x
        self.action_goal.plane_bbox.center.position.y = self.action_goal.plane_bbox.center.position.y - grocery_box_manual_deduction.y
        # print("self.action_goal.plane_bbox.center.position: ", self.action_goal.plane_bbox.center.position)

        self.action_goal.plane_bbox.size.x = self._grocery_box_size.x - grocery_box_manual_deduction.x * 2
        self.action_goal.plane_bbox.size.y = self._grocery_box_size.y - grocery_box_manual_deduction.y * 2
        # print("self.action_goal.plane_bbox: ", self.action_goal.plane_bbox)

    def update(self):
        if not self.action_client:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            return py_trees.Status.INVALID
        print('self._relation', self._relation)
        if 'Stacking' in self._relation:
            print('relation is Stacking')
        if not 'Stacking' in self._relation:
            print('relation is not Stacking')
        if 'None' in self._relation or "Away" in self._relation:
            self.action_goal.distance = 0
        if 'Stacking' in self._relation:
            return py_trees.Status.SUCCESS
        if not self.sent_goal:
            print("self.action_goal.plane_bbox: ", self.action_goal.plane_bbox)
            # print(self.action_goal.relative_obj_bbox)
            # print("self.action_goal.obj_bbox: ", self.action_goal.obj_bbox)
            # print("self.action_goal.distance: ", self.action_goal.distance)

            self.action_client.send_goal(self.action_goal)
            self.sent_goal = True
            self.feedback_message = "sent goal to the action server"
            return py_trees.Status.RUNNING
        self.feedback_message = self.action_client.get_goal_status_text()
        if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
                                              actionlib_msgs.GoalStatus.PREEMPTED]:
            return py_trees.Status.FAILURE
        result = self.action_client.get_result()

        # print("len(result.pose): ", len(result.pose))

        obj_bboxes = py_trees.blackboard.Blackboard().get('obj_bboxes')
        Away_limit_dis = 0.05
        distance_ori = []
        poses_temp = []
        ori_position = self._obj_bbox.center.position
        obj_bboxes_value = obj_bboxes.values()
        relative_bbox_position = self.action_goal.relative_obj_bbox.center.position
        if result:
            poses = []
            for i in range(len(result.pose)/2):
                pose = result.pose[i]
                pose.orientation.x = 0
                pose.orientation.y = 0.7071068
                pose.orientation.z = 0
                pose.orientation.w = 0.7071068
                pose.position.z = self._grasp_pose.position.z
                if pose.position.z -  self._plane_bbox.center.position.z > self._obj_bbox.size.y * 1.5:
                    pose.position.z = self._plane_bbox.center.position.z + self._obj_bbox.size.y * 1.1
                # poses.append(pose)
                poses_temp.append(pose)

                distance_ori.append((pose.position.x - ori_position.x) * (pose.position.x - ori_position.x) + (pose.position.y - ori_position.y) * (pose.position.y - ori_position.y))
            if 'Flat' in self._relation:
                for pose in poses_temp:
                    pose.orientation.x = 0.2705981
                    pose.orientation.y = 0.6532815
                    pose.orientation.z = 0.2705981
                    pose.orientation.w = 0.6532815

            if 'Away' in self._relation:
            # if 'Away' in self._relation:
                print("relation is Away")
                for i in range(len(poses_temp)):
                    # if i > len(poses_temp):
                    # if i > len(poses_temp) / 100:
                    pose_index = distance_ori.index(max(distance_ori))
                    x_rand = poses_temp[pose_index].position.x
                    y_rand = poses_temp[pose_index].position.y
                    add_if = True
                    for object_index in range(len(obj_bboxes)):
                        distance = np.sqrt((obj_bboxes_value[object_index].center.position.x - x_rand) * (obj_bboxes_value[object_index].center.position.x - x_rand) + (obj_bboxes_value[object_index].center.position.y - y_rand) * (obj_bboxes_value[object_index].center.position.y - y_rand))
                        # print("distance: ", distance)
                        if distance < Away_limit_dis:
                            add_if = False
                    if add_if:
                        poses.append(poses_temp[distance_ori.index(max(distance_ori))])
                    distance_ori[distance_ori.index(max(distance_ori))] = -1
            elif 'Near' in self._relation and 'meat' in self._object_index:
                for i in range(len(poses_temp)):
                    if i > len(poses_temp) / 3:
                        pose_index = distance_ori.index(max(distance_ori))
                        poses.append(poses_temp[pose_index])
                        distance_ori[pose_index] = -1
            elif 'Near' in self._relation:
                for i in range(len(poses_temp)):
                    if i > len(poses_temp) / 3:
                        pose_index = distance_ori.index(min(distance_ori))
                        direction_test = (poses_temp[pose_index].position.y - relative_bbox_position.y) / (poses_temp[pose_index].position.x - relative_bbox_position.x)
                        poses_temp[pose_index].position.x = self._test_back_dis * np.sqrt(1 / (direction_test * direction_test + 1)) + poses_temp[pose_index].position.x
                        poses_temp[pose_index].position.y = self._test_back_dis * np.sqrt(1 - 1 / (direction_test * direction_test + 1)) + poses_temp[pose_index].position.y
                        poses.append(poses_temp[pose_index])
                        distance_ori[pose_index] = 1000
            else:
                for i in range(len(poses_temp)):
                    poses.append(poses_temp[distance_ori.index(min(distance_ori))])
                    distance_ori[distance_ori.index(min(distance_ori))] = 1000


            # print(distance_ori)
            py_trees.blackboard.Blackboard().set('free_space_poses', poses)
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = self.override_feedback_message_on_running
            return py_trees.Status.RUNNING

def PickWithPoseAct(name, key_str):
    """
    Picks up object given the key to blackboard
    """
    # Retrieve object name and its graspable pose from blackboard
    grasp_pose = py_trees.blackboard.Blackboard().get(key_str)

    # Generate a pre-grasp pose that is displaced in z-direction of grasp pose
    r = R.from_quat([grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w])
    position = r.apply([0, 0, -0.1])
    pre_grasp_pose = deepcopy(grasp_pose)
    pre_grasp_pose.position.x += position[0]
    pre_grasp_pose.position.y += position[1]
    pre_grasp_pose.position.z += position[2]

    # Generate up pose that is displaced in z-direction of pre-grasp pose in world coordinate
    up_pose = deepcopy(pre_grasp_pose)
    up_pose.position.z += 0.25

    # Initialize the root as sequence node
    root = py_trees.composites.Sequence(
        name='seq_{}'.format(name),
        children=None)

    # Add steps to execute pick action
    root.add_children([
        OpenGripperAct('act_{}_open_gripper'.format(name)),
        MoveToPoseAct(name='act_{}_move_to_up_pose_1'.format(name), pose=up_pose),
        MoveToPoseAct(name='act_{}_move_to_pre_grasp_pose_1'.format(name), pose=pre_grasp_pose),
        MoveToPoseAct(name='act_{}_move_to_grasp_pose'.format(name), pose=grasp_pose),
        CloseGripperAct('act_{}_close_gripper'.format(name)),
        MoveToPoseAct(name='act_{}_move_to_pre_grasp_pose_2'.format(name), pose=pre_grasp_pose),
        MoveToPoseAct(name='act_{}_move_to_up_pose_2'.format(name), pose=up_pose),
    ])
    return root

def PlaceWithPoseAct(name, key_str):
    """
    Places the object in hand to desired position
    """

    # Retrieve object name and its graspable pose from blackboard
    des_pose = py_trees.blackboard.Blackboard().get(key_str)

    # Generate pre-up pose that is displaced in z-direction of desired pose in world coordinate
    pre_up_pose = deepcopy(des_pose)
    pre_up_pose.position.z += 0.25

    # Generate a post-desired pose that is displaced in z-direction of desired pose
    r = R.from_quat([des_pose.orientation.x, des_pose.orientation.y, des_pose.orientation.z, des_pose.orientation.w])
    position = r.apply([0, 0, -0.1])
    post_des_pose = deepcopy(des_pose)
    post_des_pose.position.x += position[0]
    post_des_pose.position.y += position[1]
    post_des_pose.position.z += position[2]

    # Generate post-up pose that is displaced in z-direction of post-desired pose in world coordinate
    post_up_pose = deepcopy(post_des_pose)
    post_up_pose.position.z += 0.25

    # Initialize the root as sequence node
    root = py_trees.composites.Sequence(
        name='seq_{}'.format(name),
        children=None)

    # Add steps to execute pick action
    root.add_children([
        MoveToPoseAct(name='act_{}_move_to_pre_up_pose'.format(name), pose=pre_up_pose),
        MoveToPoseAct(name='act_{}_move_to_des_pose'.format(name), pose=des_pose),
        OpenGripperAct('act_{}_open_gripper'.format(name)),
        MoveToPoseAct(name='act_{}_move_to_post_des_pose'.format(name), pose=post_des_pose),
        MoveToPoseAct(name='act_{}_move_to_post_up_pose'.format(name), pose=post_up_pose),
    ])
    return root

def GetDesiredPoseAct(name, surface, cur_obj):
    """
    Get_Desired_Pose
    - Input: Specifies which surface to put the object
    - Output:
    -   If for table, do:
    -     Use table detector to get the shape of the table with pose
    -     Random generate 100 points on table and pick the farthest one from other objects (also shrink the sampling space so that object wont fall)
    -     Metric: max of minimum distances from any objects
    -   If for on top of other object, do:
    -     Get the bounding box of the bottom object and current gripped object to compute the z height it needs to be placed
    -     Place on top of (x, y) of the bottom object with computed z height plus some buffer
    """
    # Initialize the root as sequence node
    root = py_trees.composites.Sequence(
        name='seq_{}'.format(name),
        children=None)

    # Get the dimensions of current object
    cur_dim = tuple(rospy.get_param("/dope/dimensions")[cur_obj])
    # print("Bleach Dimensions: ", tuple(rospy.get_param("/dope/dimensions")["bleach"]))
    # print("Cracker Dimensions: ", tuple(rospy.get_param("/dope/dimensions")["cracker"]))
    # print("Gelatin Dimensions: ", tuple(rospy.get_param("/dope/dimensions")["gelatin"]))
    # print("Meat Dimensions: ", tuple(rospy.get_param("/dope/dimensions")["meat"]))
    # print("Mustard Dimensions: ", tuple(rospy.get_param("/dope/dimensions")["mustard"]))
    # print("Soup Dimensions: ", tuple(rospy.get_param("/dope/dimensions")["soup"]))
    # print("Sugar Dimensions: ", tuple(rospy.get_param("/dope/dimensions")["sugar"]))
    obj_dim = Vector3()
    obj_dim.x = cur_dim[0] / 100
    obj_dim.y = cur_dim[2] / 100
    obj_dim.z = cur_dim[1] / 100

    # If desired surface is table, do:
    if surface == "table":
        # Add steps to detect the table and get the desired pose based on free space
        root.add_children([
            GetTableAct(name='act_{}_get_table'.format(name)),
            FreeSpaceFinderAct(name='act_{}_free_space_finder'.format(name), obj_dim=obj_dim),
        ])
    # If desired surface is on an object, do:
    else:
        des_pose = obj_poses[surface]
        des_pose.position.z += cur_obj.height / 2 + 0.5 # add buffer
    return root

"""
Table_Detector

- Get point cloud from depth image and prune the points according to z-range
- Run ransac algorithm to find plane and prune the non-horizontal ones with dot product. Pick the lowest / largest plane
- Use min / max in x & y axis to find the extreme points (corners) of the point cloud
- Fit minimum all-inclusive polygon (likely rectangle) using the extreme points
"""

class CropPCAct(py_trees_ros.actions.ActionClient):
    def __init__(self, name, in_pc_key, crop_box_key, out_pc_key, invert=False, *argv, **kwargs):
        super(CropPCAct, self).__init__(
            name=name,
            action_spec=CropPCAction,
            action_goal=CropPCGoal(),
            action_namespace='crop_pc',
            *argv,
            **kwargs
        )
        self._in_pc_key = in_pc_key
        self._crop_box_key = crop_box_key
        self._out_pc_key = out_pc_key
        self._invert = invert

    def initialise(self):
        super(CropPCAct, self).initialise()
        # Get goal from blackboard
        blackboard = py_trees.blackboard.Blackboard()
        self.action_goal.in_pc = blackboard.get(self._in_pc_key)
        self.action_goal.crop_box = blackboard.get(self._crop_box_key)
        self.action_goal.invert = self._invert

    def update(self):
        """
        Check only to see whether the underlying action server has
        succeeded, is running, or has cancelled/aborted for some reason and
        map these to the usual behaviour return states.
        """
        self.logger.debug("{0}.update()".format(self.__class__.__name__))
        if not self.action_client:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            return py_trees.Status.INVALID
        # pity there is no 'is_connected' api like there is for c++
        if not self.sent_goal:
            self.action_client.send_goal(self.action_goal)
            self.sent_goal = True
            self.feedback_message = "sent goal to the action server"
            return py_trees.Status.RUNNING
        self.feedback_message = self.action_client.get_goal_status_text()
        if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
                                              actionlib_msgs.GoalStatus.PREEMPTED]:
            return py_trees.Status.FAILURE
        result = self.action_client.get_result()
        if result:
            py_trees.blackboard.Blackboard().set(self._out_pc_key, result.out_pc)
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = self.override_feedback_message_on_running
            return py_trees.Status.RUNNING

class TFPCAct(py_trees_ros.actions.ActionClient):
    def __init__(self, name, in_pc_key, frame_id, out_pc_key, *argv, **kwargs):
        super(TFPCAct, self).__init__(
            name=name,
            action_spec=TFPCAction,
            action_goal=TFPCGoal(),
            action_namespace='tf_pc',
            *argv,
            **kwargs
        )
        self._in_pc_key = in_pc_key
        self._frame_id = frame_id
        self.action_goal.frame_id = self._frame_id
        self._out_pc_key = out_pc_key

    def initialise(self):
        super(TFPCAct, self).initialise()
        # Get goal from blackboard
        blackboard = py_trees.blackboard.Blackboard()
        self.action_goal.in_pc = blackboard.get(self._in_pc_key)

    def update(self):
        """
        Check only to see whether the underlying action server has
        succeeded, is running, or has cancelled/aborted for some reason and
        map these to the usual behaviour return states.
        """
        self.logger.debug("{0}.update()".format(self.__class__.__name__))
        if not self.action_client:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            return py_trees.Status.INVALID
        # pity there is no 'is_connected' api like there is for c++
        if not self.sent_goal:
            self.action_client.send_goal(self.action_goal)
            self.sent_goal = True
            self.feedback_message = "sent goal to the action server"
            return py_trees.Status.RUNNING
        self.feedback_message = self.action_client.get_goal_status_text()
        if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
                                              actionlib_msgs.GoalStatus.PREEMPTED]:
            return py_trees.Status.FAILURE
        result = self.action_client.get_result()
        if result:
            py_trees.blackboard.Blackboard().set(self._out_pc_key, result.out_pc)
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = self.override_feedback_message_on_running
            return py_trees.Status.RUNNING

class PCPubAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, in_pc_key, topic):
        super(PCPubAct, self).__init__(name)
        self._in_pc_key = in_pc_key
        self._topic = topic
        self._pc = None

    def setup(self, timeout):
        self.pub = rospy.Publisher(self._topic, PointCloud2, queue_size=10)
        return True

    def initialise(self):
        blackboard = py_trees.blackboard.Blackboard()
        self._pc = blackboard.get(self._in_pc_key)

    def update(self):
        if self._pc is None:
            return py_trees.Status.FAILURE
        self.pub.publish(self._pc)
        return py_trees.Status.SUCCESS

# Move group planning scene
class AddCollisionBoxAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, frame_id='base_link', box_name=None, box_pose=None, box_size=None, box_bb_key=None):
        """!
        @brief      Add a collision box to the planning scene

        @param      name        Name of the behavior
        @param      frame_id    TF frame to place the box in
        @param      box_name    String id for the box, used to update and remove the box from planning scene
        @param      box_pose    The center box pose as geometry_msgs.msg.Pose
        @param      box_size    The box size as a list with [x, y, z]
        @param      box_bb_key  The box bb key if provided will read a box from the blackboard and overwrite the other
                                parameters, the blackboard value should be a list as such [box_name, box_pose, box_size]
        """
        super(AddCollisionBoxAct, self).__init__(name)
        self._pub = None
        self._frame_id = frame_id
        self._box_name = box_name
        self._box_pose = box_pose
        self._box_size = box_size
        self._box_bb_key = box_bb_key

    def setup(self, timeout):
        self._pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=1)
        return True

    def initialise(self):
        if self._box_bb_key is not None:
            self._box_name, self._box_pose, self._box_size = py_trees.blackboard.Blackboard().get(self._box_bb_key)

    def update(self):
        # Make the box
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = self._box_size

        # Define the collision object
        obj = CollisionObject()
        obj.header.frame_id = self._frame_id
        obj.id = self._box_name
        obj.primitives.append(box)
        obj.primitive_poses.append(self._box_pose)
        obj.operation = obj.ADD

        # Update the planning scene
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.world.collision_objects.append(obj);
        self._pub.publish(planning_scene);
        return py_trees.Status.SUCCESS

# Move group planning scene
class RemoveCollisionBoxAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, box_name=None, box_bb_key=None):
        """!
        @brief      Add a collision box to the planning scene

        @param      name        Name of the behavior
        @param      box_name    String id for the box, used to update and remove the box from planning scene
        @param      box_bb_key  The box bb key if provided will read a box from the blackboard and overwrite the other
                                parameters, the blackboard value should be a list as such [box_name, box_pose, box_size]
        """
        super(RemoveCollisionBoxAct, self).__init__(name)
        self._pub = None
        self._box_name = box_name
        self._box_bb_key = box_bb_key

    def setup(self, timeout):
        self._pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=1)
        return True

    def initialise(self):
        if self._box_bb_key is not None:
            self._box_name, _, _ = py_trees.blackboard.Blackboard().get(self._box_bb_key)

    def update(self):
        # Define the collision object
        obj = CollisionObject()
        obj.id = self._box_name
        obj.operation = obj.REMOVE

        # Update the planning scene
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.world.collision_objects.append(obj);
        self._pub.publish(planning_scene);
        return py_trees.Status.SUCCESS

class AddAllObjectCollisionBoxAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, frame_id='base_link', bbox_key='obj_bboxes', grocery_box_size_key='grocery_box_size',
                 grocery_box_offset_key = 'grocery_box_offset', plane_bbox_key = 'plane_bboxes', grocery_bboxes_key = 'grocery_bboxes'):
        super(AddAllObjectCollisionBoxAct, self).__init__(name)
        self._pub = None
        self._bbox_key = bbox_key
        self._box_name = None
        self._box_pose = None
        self._box_size = None
        self._frame_id = frame_id
        self._grocery_bboxes_key = grocery_bboxes_key
        self._grocery_bboxes = None
        self._grocery_box_size_key = grocery_box_size_key
        self._grocery_box_size = None

    def setup(self, timeout):
        self._pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=1)
        return True

    def update(self):
        # Make the box
        blackboard = py_trees.blackboard.Blackboard()

        bboxes = blackboard.get(self._bbox_key)
        self._grocery_bboxes = blackboard.get(self._grocery_bboxes_key)
        self._grocery_box_size = blackboard.get(self._grocery_box_size_key)
        print("self._grocery_box_size: ", self._grocery_box_size)

        grocery_collision_box_name = blackboard.get('grocery_collision_box_name')

        planning_scene = PlanningScene()
        planning_scene.is_diff = True

        grocery_collsion_box_bboxes = []

        for grocery_bbox in self._grocery_bboxes:
            grocery_box_position = grocery_bbox.center.position
            grocery_box_size = grocery_bbox.size


            left_bbox = BoundingBox3D()
            left_bbox.center = Pose()
            left_bbox.center.orientation.w = 1.0
            left_bbox.center.position.z = grocery_box_position.z
            left_bbox.center.position.x = grocery_box_position.x
            left_bbox.center.position.y = grocery_box_position.y + grocery_box_size.y / 2
            left_bbox.size.x = self._grocery_box_size.x
            left_bbox.size.z = self._grocery_box_size.z
            left_bbox.size.y = 0.002
            print("left_bbox.center.position: ", left_bbox.center.position)

            right_bbox = BoundingBox3D()
            right_bbox.center = Pose()
            right_bbox.center.orientation.w = 1.0
            right_bbox.center.position.z = grocery_box_position.z
            right_bbox.center.position.x = grocery_box_position.x
            right_bbox.center.position.y = grocery_box_position.y - grocery_box_size.y / 2
            right_bbox.size.x = self._grocery_box_size.x
            right_bbox.size.z = self._grocery_box_size.z
            right_bbox.size.y = 0.002
            print("right_bbox.center.position: ", right_bbox.center.position)

            front_bbox = BoundingBox3D()
            front_bbox.center = Pose()
            front_bbox.center.orientation.w = 1.0
            front_bbox.center.position.z = grocery_box_position.z - self._grocery_box_size.z / 4
            front_bbox.center.position.x = grocery_box_position.x - grocery_box_size.x / 2
            front_bbox.center.position.y = grocery_box_position.y
            front_bbox.size.y = self._grocery_box_size.y
            front_bbox.size.z = self._grocery_box_size.z / 2
            front_bbox.size.x = 0.002
            print("front_bbox.center.position: ", front_bbox.center.position)

            back_bbox = BoundingBox3D()
            back_bbox.center = Pose()
            back_bbox.center.orientation.w = 1.0
            back_bbox.center.position.z = grocery_box_position.z
            back_bbox.center.position.x = grocery_box_position.x + grocery_box_size.x / 2
            back_bbox.center.position.y = grocery_box_position.y
            back_bbox.size.y = self._grocery_box_size.y
            back_bbox.size.z = self._grocery_box_size.z
            back_bbox.size.x = 0.002
            print("back_bbox.center.position: ", back_bbox.center.position)

            grocery_collsion_box_bboxes.append(left_bbox)
            grocery_collsion_box_bboxes.append(right_bbox)
            grocery_collsion_box_bboxes.append(front_bbox)
            grocery_collsion_box_bboxes.append(back_bbox)

        for i in  range(len(grocery_collsion_box_bboxes)):
            self._box_name = grocery_collision_box_name[i % 4] + '_' + str(int(i // 4))
            print("self._box_name: ", self._box_name)
            self._box_pose = grocery_collsion_box_bboxes[i].center
            box_size = grocery_collsion_box_bboxes[i % 4].size
            self._box_size = [box_size.x + 0.01, box_size.y + 0.01, box_size.z + 0.01]
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = self._box_size

            # Define the collision object
            obj = CollisionObject()
            obj.header.frame_id = self._frame_id
            obj.id = self._box_name
            obj.primitives.append(box)
            obj.primitive_poses.append(self._box_pose)
            obj.operation = obj.ADD

        # Update the planning scene
            planning_scene.world.collision_objects.append(obj)


        for i in  range(len(bboxes)):
            self._box_name = bboxes.keys()[i]
            self._box_pose = bboxes[self._box_name].center
            box_size = bboxes[self._box_name].size
            self._box_size = [box_size.x + 0.01, box_size.y + 0.01, box_size.z + 0.01]
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = self._box_size

            # Define the collision object
            obj = CollisionObject()
            obj.header.frame_id = self._frame_id
            obj.id = self._box_name
            obj.primitives.append(box)
            obj.primitive_poses.append(self._box_pose)
            obj.operation = obj.ADD

        # Update the planning scene
            planning_scene.world.collision_objects.append(obj);

        # grocery box collision box

        self._pub.publish(planning_scene)
        return py_trees.Status.SUCCESS

class RemoveAllCollisionBoxAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, bbox_key='obj_bboxes'):
        super(RemoveAllCollisionBoxAct, self).__init__(name)
        self._pub = None
        self._bbox_key = bbox_key
        self._box_name = None

    def setup(self, timeout):
        self._pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=1)
        return True

    def update(self):
        # Make the box
        blackboard = py_trees.blackboard.Blackboard()
        bboxes = blackboard.get(self._bbox_key)
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        for i in  range(len(bboxes)):
            self._box_name = bboxes.keys()[i]

            obj = CollisionObject()
            obj.id = self._box_name
            obj.operation = obj.REMOVE

            planning_scene.world.collision_objects.append(obj);

        self._pub.publish(planning_scene);
        return py_trees.Status.SUCCESS

class AttachObjectAct(py_trees_ros.actions.ActionClient):
    def __init__(self, name, object_bbox_key = 'crop_box', object_name=None, to_attach=False, *argv, **kwargs):
        super(AttachObjectAct, self).__init__(
            name=name,
            action_spec=AttachObjectAction,
            action_goal=AttachObjectGoal(),
            action_namespace='attach_object',
            *argv,
            **kwargs
        )
        self._to_attach = to_attach
        self._object_name = object_name
        self._object_bbox_key = object_bbox_key
        self._object_bbox = None
        self._to_attach = to_attach

    def initialise(self):
        self._object_bbox = py_trees.blackboard.Blackboard().get(self._object_bbox_key)

    def update(self):
        # Define the collision object
        self.action_goal.toAttach = self._to_attach
        self.action_goal.object_name = self._object_name
        if self._to_attach:
            self.logger.debug("{0}.update()".format(self.__class__.__name__))
            if not self.action_client:
                self.feedback_message = "no action client, did you call setup() on your tree?"
                return py_trees.Status.INVALID
            # pity there is no 'is_connected' api like there is for c++
            if not self.sent_goal:
                self.action_client.send_goal(self.action_goal)
                self.sent_goal = True
                self.feedback_message = "sent goal to the action server"
                return py_trees.Status.RUNNING
            self.feedback_message = self.action_client.get_goal_status_text()
            if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
                                                  actionlib_msgs.GoalStatus.PREEMPTED]:
                return py_trees.Status.FAILURE
            result = self.action_client.get_result()
            if result:
                return py_trees.Status.SUCCESS
            else:
                self.feedback_message = self.override_feedback_message_on_running
                return py_trees.Status.RUNNING

        else:
            self.logger.debug("{0}.update()".format(self.__class__.__name__))
            if not self.action_client:
                self.feedback_message = "no action client, did you call setup() on your tree?"
                return py_trees.Status.INVALID
            # pity there is no 'is_connected' api like there is for c++
            if not self.sent_goal:
                self.action_client.send_goal(self.action_goal)
                self.sent_goal = True
                self.feedback_message = "sent goal to the action server"
                return py_trees.Status.RUNNING
            self.feedback_message = self.action_client.get_goal_status_text()
            if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
                                                  actionlib_msgs.GoalStatus.PREEMPTED]:
                return py_trees.Status.FAILURE
            result = self.action_client.get_result()
            if result:
                obj = CollisionObject()
                obj.id = self._object_name
                obj.operation = obj.REMOVE

            # Update the planning scene
                planning_scene = PlanningScene()
                planning_scene.is_diff = True
                planning_scene.world.collision_objects.append(obj);
                return py_trees.Status.SUCCESS
            else:
                self.feedback_message = self.override_feedback_message_on_running
                return py_trees.Status.RUNNING


        self._pub.publish(planning_scene);
        return py_trees.Status.SUCCESS

# Ignore collisions with gripper
class SetAllowGripperCollisionAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, allow):
        """!
        @brief      Add a collision box to the planning scene

        @param      name   Name of the behavior
        @param      allow  True to allow gripper collision false to disallow
        """
        super(SetAllowGripperCollisionAct, self).__init__(name)
        self._pub = None
        self._get_scene_client = None
        self._allow = allow

    def setup(self, timeout):
        self._pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=1)
        rospy.wait_for_service('/get_planning_scene', timeout=timeout)
        self._get_scene_client = rospy.ServiceProxy("/get_planning_scene", GetPlanningScene)
        return True

    def initialise(self):
        pass

    def update(self):
        # Get the current scene
        get_scene = PlanningSceneComponents(components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        allowed_collision_matrix = self._get_scene_client(get_scene).scene.allowed_collision_matrix
        for link in ['l_gripper_finger_link', 'r_gripper_finger_link', 'gripper_link']:
            try:
                i = allowed_collision_matrix.default_entry_names.index(link)
                if self._allow:
                    allowed_collision_matrix.default_entry_values[i] = self._allow
                else:
                    del allowed_collision_matrix.default_entry_names[i]
                    del allowed_collision_matrix.default_entry_values[i]
            except ValueError:
                if self._allow:
                    allowed_collision_matrix.default_entry_names.append(link)
                    allowed_collision_matrix.default_entry_values.append(self._allow)

        # Update the planning scene
        planning_scene = PlanningScene(is_diff=True, allowed_collision_matrix=allowed_collision_matrix)
        self._pub.publish(planning_scene);
        return py_trees.Status.SUCCESS

class GroceryLinkDettachingAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, grocery_box_index_key='grocery_box_index', model_state_key='model_state', grocery_bboxes_key='grocery_bboxes',
        link_name='link_1', relative_object_name='fetch', relative_link_name='wrist_roll_link'):
        super(GroceryLinkDettachingAct, self).__init__(name)
        # self._grocery_name = grocery_name
        self._link_dettacher_client = None
        self._model_state_key = model_state_key
        self._model_state = None
        self._grocery_bboxes_key = grocery_bboxes_key
        self._grocery_bboxes = None
        self._link_name = 'link_1'
        self._relative_object_name = relative_object_name
        self._relative_link_name = relative_link_name
        self._grocery_box_index_key = grocery_box_index_key
        self._grocery_box_index = None

    def setup(self, timeout):
        rospy.wait_for_service('/link_attacher_node/attach', timeout=timeout)
        self._link_dettacher_client = rospy.ServiceProxy("/link_attacher_node/detach", Attach)
        return True

    def initialise(self):
        pass

    def update(self):
        self._grocery_box_index = py_trees.blackboard.Blackboard().get(self._grocery_box_index_key)
        self._model_state = py_trees.blackboard.Blackboard().get(self._model_state_key)
        self._grocery_bboxes = py_trees.blackboard.Blackboard().get(self._grocery_bboxes_key)
        grocery_bbox = self._grocery_bboxes[self._grocery_box_index]
        if self._grocery_box_index > 0:
            grocery_box_name = 'grocery_box' + '_' + str(self._grocery_box_index - 1)
        elif self._grocery_box_index == 0:
            grocery_box_name = 'grocery_box'

        req = AttachRequest()
        # req.model_name_1 = self._object_name
        req.model_name_1 = grocery_box_name
        req.link_name_1 = self._link_name
        req.model_name_2 = self._relative_object_name
        # req.model_name_2 = object_move_name
        req.link_name_2 = self._relative_link_name
        self._link_dettacher_client.call(req)

        x_min = grocery_bbox.center.position.x - grocery_bbox.size.x / 2
        x_max = grocery_bbox.center.position.x + grocery_bbox.size.x / 2
        y_min = grocery_bbox.center.position.y - grocery_bbox.size.y / 2
        y_max = grocery_bbox.center.position.y + grocery_bbox.size.y / 2

        for i in range(len(self._model_state.name)):
            if 'test' in self._model_state.name[i] and self._model_state.pose[i].position.x > x_min and self._model_state.pose[i].position.x < x_max and self._model_state.pose[i].position.y > y_min and self._model_state.pose[i].position.y < y_max:
                req_object = AttachRequest()
                req_object.model_name_1 = self._model_state.name[i]
                req_object.link_name_1 = self._link_name
                req_object.model_name_2 = grocery_box_name
                req_object.link_name_2 = 'link_1'
                # req_object.model_name_2 = self._relative_object_name
                # req_object.link_name_2 = self._relative_link_name
                self._link_dettacher_client.call(req_object)
                time.sleep(0.05)

        return py_trees.Status.SUCCESS

class GroceryLinkAttachingAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, grocery_box_index_key='grocery_box_index', model_state_key='model_state', grocery_bboxes_key='grocery_bboxes',
        link_name='link_1', relative_object_name='fetch', relative_link_name='wrist_roll_link'):
        super(GroceryLinkAttachingAct, self).__init__(name)
        # self._grocery_name = grocery_name
        self._link_attacher_client = None
        self._model_state_key = model_state_key
        self._model_state = None
        self._grocery_bboxes_key = grocery_bboxes_key
        self._grocery_bboxes = None
        self._link_name = 'link_1'
        self._relative_object_name = relative_object_name
        self._relative_link_name = relative_link_name
        self._grocery_box_index_key = grocery_box_index_key
        self._grocery_box_index = None

    def setup(self, timeout):
        rospy.wait_for_service('/link_attacher_node/attach', timeout=timeout)
        self._link_attacher_client = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
        return True

    def initialise(self):
        pass

    def update(self):
        self._grocery_box_index = py_trees.blackboard.Blackboard().get(self._grocery_box_index_key)
        self._model_state = py_trees.blackboard.Blackboard().get(self._model_state_key)
        self._grocery_bboxes = py_trees.blackboard.Blackboard().get(self._grocery_bboxes_key)
        grocery_bbox = self._grocery_bboxes[self._grocery_box_index]
        if self._grocery_box_index > 0:
            grocery_box_name = 'grocery_box' + '_' + str(self._grocery_box_index - 1)
        elif self._grocery_box_index == 0:
            grocery_box_name = 'grocery_box'

        req = AttachRequest()
        # req.model_name_1 = self._object_name
        req.model_name_1 = grocery_box_name
        req.link_name_1 = self._link_name
        req.model_name_2 = self._relative_object_name
        # req.model_name_2 = object_move_name
        req.link_name_2 = self._relative_link_name
        print("req: ", req)
        self._link_attacher_client.call(req)

        x_min = grocery_bbox.center.position.x - grocery_bbox.size.x / 2
        x_max = grocery_bbox.center.position.x + grocery_bbox.size.x / 2
        y_min = grocery_bbox.center.position.y - grocery_bbox.size.y / 2
        y_max = grocery_bbox.center.position.y + grocery_bbox.size.y / 2

        for i in range(len(self._model_state.name)):
            if 'test' in self._model_state.name[i] and self._model_state.pose[i].position.x > x_min and self._model_state.pose[i].position.x < x_max and self._model_state.pose[i].position.y > y_min and self._model_state.pose[i].position.y < y_max:
                req_object = AttachRequest()
                req_object.model_name_1 = self._model_state.name[i]
                req_object.link_name_1 = self._link_name
                # req_object.model_name_2 = self._relative_object_name
                # req_object.link_name_2 = self._relative_link_name
                req_object.model_name_2 = grocery_box_name
                req_object.link_name_2 = 'link_1'
                # print("req_object: ", req_object)
                self._link_attacher_client.call(req_object)
                time.sleep(0.05)

        return py_trees.Status.SUCCESS

class SpawnModelAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, relative_object_name='fetch'):
        super(SpawnModelAct, self).__init__(name)
        # self._pub = None
        self._spawn_model_client = None
        # self._allow = allow
        # self._object_name = object_name


    def setup(self, timeout):
        # self._pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=1)
        rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=timeout)
        self._spawn_model_client = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

        return True

    def initialise(self):
        pass

    def update(self):

        # req = '{model_state: { model_name: coke_can, pose: { position: { x: 0.3, y: 0.2 ,z: 0 }, orientation: {x: 0, y: 0.491983115673, z: 0, w: 0.870604813099 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'
        # print("req: ", req)
        # req = SpawnModel()
        # print("req: ", req)
        req = SpawnModelRequest()
        req.model_name = 'table'
        f = open('/home/user/catkin_ws/src/mobile_manipulation/srp-md/data/gazebo_test/table_1/model.sdf','r')
        # sdff = f.read()
        req.model_xml = f.read()
        # req.robot_namespace
        req.initial_pose.orientation.w = 1
        req.initial_pose.position.x = 1
        req.reference_frame = 'world'

        self._spawn_model_client.call(req)

        req = SpawnModelRequest()
        req.model_name = 'lower_table'
        f = open('/home/user/catkin_ws/src/mobile_manipulation/srp-md/data/gazebo_test/table_2/model.sdf','r')
        # sdff = f.read()
        req.model_xml = f.read()
        # req.robot_namespace
        req.initial_pose.orientation.w = 1
        req.initial_pose.position.y = 1
        req.reference_frame = 'world'

        self._spawn_model_client.call(req)

        req = SpawnModelRequest()
        req.model_name = 'conveyor_belt'
        f = open('/home/user/catkin_ws/src/mobile_manipulation/srp-md/data/gazebo_test/conveyor_belt/model.sdf','r')
        # sdff = f.read()
        req.model_xml = f.read()
        # req.robot_namespace
        req.initial_pose.orientation.w = 1
        req.initial_pose.position.x = 1
        req.initial_pose.position.y = -3.24
        req.reference_frame = 'world'

        self._spawn_model_client.call(req)

        req = SpawnModelRequest()
        req.model_name = 'wall'
        f = open('/home/user/catkin_ws/src/mobile_manipulation/srp-md/data/gazebo_test/wall/model.sdf','r')
        # sdff = f.read()
        req.model_xml = f.read()
        # req.robot_namespace
        req.initial_pose.orientation.w = 1
        req.initial_pose.position.x = 0
        req.initial_pose.position.y = 0
        req.reference_frame = 'world'

        self._spawn_model_client.call(req)

        return py_trees.Status.SUCCESS

class LinkAttachingAct(py_trees.behaviour.Behaviour):
    # def __init__(self, name, object_name, link_name, relative_object_name='fetch', relative_link_name='l_gripper_finger_link'):
    def __init__(self, name, object_name, link_name, relative_object_name='fetch', relative_link_name='wrist_roll_link'):
        super(LinkAttachingAct, self).__init__(name)
        # self._pub = None
        self._link_attacher_client = None
        # self._allow = allow
        self._object_name = object_name
        self._link_name = link_name
        self._relative_object_name = relative_object_name
        self._relative_link_name = relative_link_name

    def setup(self, timeout):
        # self._pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=1)
        rospy.wait_for_service('/link_attacher_node/attach', timeout=timeout)
        self._link_attacher_client = rospy.ServiceProxy("/link_attacher_node/attach", Attach)

        return True

    def initialise(self):
        pass

    def update(self):
        object_move_name = None
        # print("self._obj_name: ", self._obj_name)
        object_move_name = self._object_name[0:-1] + 'test_' + str(int(self._object_name[-1]) - 1)
        if self._object_name[-1] == '0':
            object_move_name = object_move_name[0:-3]
        req = AttachRequest()
        # req.model_name_1 = self._object_name
        req.model_name_1 = object_move_name
        req.link_name_1 = self._link_name
        req.model_name_2 = self._relative_object_name
        # req.model_name_2 = object_move_name
        req.link_name_2 = self._relative_link_name
        self._link_attacher_client.call(req)
        # allowed_collision_matrix = self._link_attacher_client(get_scene).scene.allowed_collision_matrix
        return py_trees.Status.SUCCESS

class LinkDettachingAct(py_trees.behaviour.Behaviour):
    # def __init__(self, name, object_name, link_name, relative_object_name='fetch', relative_link_name='l_gripper_finger_link'):
    def __init__(self, name, object_name, link_name, relative_object_name='fetch', relative_link_name='wrist_roll_link'):
        super(LinkDettachingAct, self).__init__(name)
        # self._pub = None
        self._link_dettacher_client = None
        # self._allow = allow
        self._object_name = object_name
        self._link_name = link_name
        self._relative_object_name = relative_object_name
        self._relative_link_name = relative_link_name

    def setup(self, timeout):
        # self._pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=1)
        rospy.wait_for_service('/link_attacher_node/detach', timeout=timeout)
        self._link_dettacher_client = rospy.ServiceProxy("/link_attacher_node/detach", Attach)

        return True

    def initialise(self):
        pass

    def update(self):
        # test_move_base = move_base_msgs.MoveBaseGoal()
        # print(type(test_move_base))
        # print("test_move_base: ",test_move_base)
        object_move_name = None
        # print("self._obj_name: ", self._obj_name)
        object_move_name = self._object_name[0:-1] + 'test_' + str(int(self._object_name[-1]) - 1)
        if self._object_name[-1] == '0':
            object_move_name = object_move_name[0:-3]
        req = AttachRequest()
        # req.model_name_1 = self._object_name
        req.model_name_1 = object_move_name
        req.link_name_1 = self._link_name
        req.model_name_2 = self._relative_object_name
        # req.model_name_2 = object_move_name
        req.link_name_2 = self._relative_link_name
        self._link_dettacher_client.call(req)
        # allowed_collision_matrix = self._link_attacher_client(get_scene).scene.allowed_collision_matrix
        return py_trees.Status.SUCCESS

def AddAllCollisionBoxesAct(name):
    """
    Add all collision boxes
    """
    # Initialize the root as sequence node
    root = py_trees.composites.Sequence(
        name='{}_add_all_collision_boxes'.format(name),
        children=None)
    blackboard = py_trees.blackboard.Blackboard()
    # Specify the wall sizes and table size
    left_wall_pose = Pose()
    left_wall_pose.position.y = 0.5
    left_wall_pose.orientation.w = 1.0

    right_wall_pose = Pose()
    right_wall_pose.position.y = -0.5
    right_wall_pose.orientation.w = 1.0

    bot_wall_pose = Pose()
    bot_wall_pose.position.z = 0.45
    bot_wall_pose.orientation.w = 1.0

    table_pose = Pose()
    table_pose.position.x = 1.0
    table_pose.position.y = 0.0
    table_pose.position.z = 0.38
    table_pose.orientation.w = 1.0

    table = Pose()
    table.position.x = 0.70
    table.position.y = 0.0
    table.position.z = 0.280
    table.orientation.w = 1.0
    blackboard.set('table_collision_obj', ['table', table, [0.85, 4, table.position.z * 2]])

    # Add each collision boxes to the root node
    root.add_children([
        # do not forget to add this back when real robot test
        # AddCollisionBoxAct('act_add_left_wall', box_name='left_wall', box_pose=left_wall_pose, box_size=[10, 0.1, 10]),
        # AddCollisionBoxAct('act_add_right_wall', box_name='right_wall', box_pose=right_wall_pose, box_size=[10, 0.1, 10]),
        AddCollisionBoxAct('act_add_bot_wall', box_name='bot_wall', box_pose=bot_wall_pose, box_size=[0.5, 0.5, 0.01]),
        AddCollisionBoxAct('act_add_table', box_bb_key='table_collision_obj'),
    ])

    return root

def GetDopeAndPoseAct(name):
    root = py_trees.composites.Sequence(
        name='{}_get_dope_and_pose'.format(name),
        children=None)
    root.add_children([
        TuckWithCondBehavior('act_{}_tuck_arm'.format(name), tuck_pose='tuck'),

    ])
    return root


def MoveToStartAct(name, poses_key):
    """
    Bring the robot to initial position for experiment
    """
    # Initialize the root as sequence node
    root = py_trees.composites.Sequence(
        name='{}_move_to_start'.format(name),
        children=None)
    root.add_children([
        FullyExtendTorso('act_{}_extend_torso'.format(name)),
        SetAllowGripperCollisionAct('act_ignore_gripper_collision', allow=False),
        TuckBehavior(name='act_{}_tuck_arm'.format(name), tuck_pose='tuck'),
        # HeadMoveBehavior('act_{}_look_strait'.format(name), 'MoveStraight'),
        OpenGripperAct('act_{}_open_gripper'.format(name)),
        GrasplocPickAct('act_grasploc_pick', poses_key),
    ])
    return root


def PushBoxAct(name):
    """
    Bring the robot to initial position for experiment
    """
    # Initialize the root as sequence node
    pre_grasp = TransformStamped()
    pre_grasp.header.frame_id = 'base_link'
    pre_grasp.transform.translation.x = 0.4
    pre_grasp.transform.translation.y = -0.2
    pre_grasp.transform.translation.z = 0.45
    pre_grasp.transform.rotation.x = 0.0
    pre_grasp.transform.rotation.y = 1.0
    pre_grasp.transform.rotation.z = 0.0
    pre_grasp.transform.rotation.w = 0.0

    root = py_trees.composites.Sequence(
        name='{}_move_to_start'.format(name),
        children=None)
    root.add_children([
        py_trees_ros.subscribers.ToBlackboard(
            name='act_get_groundtruth',
            topic_name='/gazebo/model_states',
            topic_type=ModelStates,
            blackboard_variables={'model_state': None},
            clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE),
        FullyExtendTorso('act_{}_extend_torso'.format(name)),
        SetAllowGripperCollisionAct('act_ignore_gripper_collision', allow=False),
        TuckBehavior(name='act_{}_tuck_arm'.format(name), tuck_pose='tuck'),
        OpenGripperAct('act_{}_open_gripper'.format(name)),
        GetFakeDopeSnapshotAct('act_get_dope_snapshot'),
        AddAllObjectCollisionBoxAct('act_add_all_object_collision_box'),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        PushPoseGeneration('PushPoseGeneration'),
        MoveToRelativePoseAct('act_{}_move_pre_grasp'.format(name), pre_grasp),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        OffsetPoses('act_offset_place_poses',
                    offset=[-(gripper_length + pre_grasp_offset), 0, 0],
                    in_poses_key='push_pose',
                    out_poses_key='offset_place_pose', debug=True),
        MoveToFirstPoseAct('act_{}_pick_grasploc'.format(name), poses_key='offset_place_pose'),
        # MoveToFirstPoseAct('act_{}_pick_grasploc'.format(name), poses_key='offset_place_pose'),
        SleepBehavior('act_sleep_a_smidge', duration=1.5),
        RelativeCartesianMoveAct('act_move_to_grasp_pose', pose_diff_msg=to_grasp_full_tf),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        GroceryLinkAttachingAct('GroceryLinkAttachingAct'),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        SetAllowGripperCollisionAct('act_ignore_gripper_collision', allow=True),
        RelativeCartesianMoveAct('act_move_to_grasp_pose', pose_diff_msg=to_push_tf),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        GroceryLinkDettachingAct('GroceryLinkDettachingAct'),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        py_trees_ros.subscribers.ToBlackboard(
            name='act_get_groundtruth',
            topic_name='/gazebo/model_states',
            topic_type=ModelStates,
            blackboard_variables={'model_state': None},
            clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        SetAllowGripperCollisionAct('act_ignore_gripper_collision', allow=False),
        GetFakeDopeSnapshotAct('act_get_dope_snapshot'),
        AddAllObjectCollisionBoxAct('act_add_all_object_collision_box'),
        TuckBehavior(name='act_{}_tuck_arm'.format(name), tuck_pose='tuck'),
    ])
    return root

def GrabBoxAct(name):
    """
    Bring the robot to initial position for experiment
    """
    # Initialize the root as sequence node
    pre_grasp = TransformStamped()
    pre_grasp.header.frame_id = 'base_link'
    pre_grasp.transform.translation.x = 0.4
    pre_grasp.transform.translation.y = -0.2
    pre_grasp.transform.translation.z = 0.45
    pre_grasp.transform.rotation.x = 0.0
    pre_grasp.transform.rotation.y = 1.0
    pre_grasp.transform.rotation.z = 0.0
    pre_grasp.transform.rotation.w = 0.0

    root = py_trees.composites.Sequence(
        name='{}_move_to_start'.format(name),
        children=None)
    root.add_children([
        py_trees_ros.subscribers.ToBlackboard(
            name='act_get_groundtruth',
            topic_name='/gazebo/model_states',
            topic_type=ModelStates,
            blackboard_variables={'model_state': None},
            clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE),
        FullyExtendTorso('act_{}_extend_torso'.format(name)),
        SetAllowGripperCollisionAct('act_ignore_gripper_collision', allow=False),
        TuckBehavior(name='act_{}_tuck_arm'.format(name), tuck_pose='tuck'),
        OpenGripperAct('act_{}_open_gripper'.format(name)),
        GetFakeDopeSnapshotAct('act_get_dope_snapshot'),
        AddAllObjectCollisionBoxAct('act_add_all_object_collision_box'),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        GrabBoxPoseGeneration('GrabBoxPoseGeneration'),
        MoveToRelativePoseAct('act_{}_move_pre_grasp'.format(name), pre_grasp),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        OffsetPoses('act_offset_place_poses',
                    offset=[-(gripper_length + pre_grasp_offset), 0, 0],
                    in_poses_key='grab_pose',
                    out_poses_key='offset_place_pose', debug=True),
        MoveToFirstPoseAct('act_{}_pick_grasploc'.format(name), poses_key='offset_place_pose'),
        SleepBehavior('act_sleep_a_smidge', duration=1),
        RelativeCartesianMoveAct('act_move_to_grasp_pose', pose_diff_msg=to_grasp_tf),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        GroceryLinkAttachingAct('GroceryLinkAttachingAct'),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        SetAllowGripperCollisionAct('act_ignore_gripper_collision', allow=True),
        RelativeCartesianMoveAct('act_move_to_grasp_pose', pose_diff_msg=to_grab_box_tf),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        RelativeCartesianMoveAct('act_move_to_grasp_pose', pose_diff_msg=to_grab_box_tf_append),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        GroceryLinkDettachingAct('GroceryLinkDettachingAct'),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        RelativeCartesianMoveAct('act_move_to_grasp_pose', pose_diff_msg=to_grab_box_back_tf),
        SetAllowGripperCollisionAct('act_ignore_gripper_collision', allow=False),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        py_trees_ros.subscribers.ToBlackboard(
            name='act_get_groundtruth',
            topic_name='/gazebo/model_states',
            topic_type=ModelStates,
            blackboard_variables={'model_state': None},
            clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE),
        # SleepBehavior('act_sleep_a_smidge', duration=0.5),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        GetFakeDopeSnapshotAct('act_get_dope_snapshot'),
        AddAllObjectCollisionBoxAct('act_add_all_object_collision_box'),
        TuckBehavior(name='act_{}_tuck_arm'.format(name), tuck_pose='tuck'),
    ])
    return root

def PickAct(name, obj, surface, relation='None'):
    """
    Composite pick action for an object
    """
    # Initialize the root as sequence node
    root = py_trees.composites.Sequence(
        name='{}_pick_{}'.format(name, obj),
        children=None)

    # Add steps to execute pick action

    object_index = obj
    relative_object_index = surface

    grasploc_grasp_fall = py_trees.composites.Selector('grasploc_grasp_fall')
    grasploc_grasp_seq = py_trees.composites.Sequence('grasploc_grasp_seq')
    grasploc_grasp_seq.add_children([
        SetAllowGripperCollisionAct('act_ignore_gripper_collision', allow=True),
        SleepBehavior('act_sleep_a_smidge', duration=1.5),
        RelativeCartesianMoveAct('act_move_to_grasp_pose', pose_diff_msg=to_grasp_tf),
        LinkAttachingAct('LinkAttachAct',object_name=object_index, link_name='link_1'),
        CloseGripperAct('act_close_gripper'),
        RemoveCollisionBoxAct('act_remove_inhand_collision_object', box_name=object_index),
        AddCollisionBoxAct('act_add_inhand_collision_object', box_bb_key='inhand_collision_object'),
        py_trees_ros.subscribers.ToBlackboard(
            name='act_get_groundtruth',
            topic_name='/gazebo/model_states',
            topic_type=ModelStates,
            blackboard_variables={'model_state': None},
            clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE),
        StabilizeObjectAct('StabilizeObjectAct', obj_name=obj),
        StabilizeObjectAct('StabilizeObjectAct', obj_name=relative_object_index),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        AttachObjectAct('act_attach_object', object_name='inhand_collision_object', to_attach=True),
        RelativeCartesianMoveAct('act_move_up', pose_diff_msg=up_tf),

        SetAllowGripperCollisionAct('act_ignore_gripper_collision', allow=False)
    ])

    grasploc_grasp_fall.add_children([
        grasploc_grasp_seq,
        py_trees.meta.success_is_failure(SetAllowGripperCollisionAct)(name='act_ignore_gripper_collision', allow=False)
    ])

    root.add_children([
        py_trees_ros.subscribers.ToBlackboard(
            name='act_get_pc',
            topic_name='/head_camera/depth_registered/points',
            topic_type=PointCloud2,
            blackboard_variables={'in_pc': None},
            clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE),
        py_trees_ros.subscribers.ToBlackboard(
            name='act_get_groundtruth',
            topic_name='/gazebo/model_states',
            topic_type=ModelStates,
            blackboard_variables={'model_state': None},
            clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE),
        py_trees_ros.subscribers.ToBlackboard(
            name='act_get_pc',
            topic_name='/head_camera/depth_downsample/points',
            topic_type=PointCloud2,
            blackboard_variables={'depth_downsampled': None},
            clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE),
        py_trees_ros.subscribers.ToBlackboard(
            name='act_get_pc',
            # topic_name='/head_camera/depth/points',
            topic_name='/head_camera/depth_registered/points',
            topic_type=PointCloud2,
            blackboard_variables={'depth_pc': None},
            clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE),
        TFPCAct('act_tf_pc', 'in_pc', 'base_link', 'tfed_pc'),
        CropPCAct('act_test_crop_pc', 'tfed_pc', 'crop_box', 'croped_pc'),
        PCPubAct('act_pub_croped_pc', 'croped_pc', 'croped_pc'),

        AttachObjectAct('act_dettach_object', object_name='inhand_collision_object', to_attach=False),
        RemoveCollisionBoxAct('act_remove_inhand_collision_object', box_name='inhand_collision_object'),

        # GetDopeSnapshotAct('act_get_dope_snapshot', object_name=object_index),
        GetFakeDopeSnapshotAct('act_get_dope_snapshot'),

        # GetTableAct('act_get_table'),


        ChooseGrasplocObjAct('act_choose_grasploc_obj', relation=relation, object_index=object_index, relative_object_index=relative_object_index),
        AddAllObjectCollisionBoxAct('act_add_all_object_collision_box'),
        GraspPoseGeneration('generate_gripper_pose'),
        FilterGrasplocPoints('act_filt_grasploc'),
        OffsetPoses('act_offset_grasploc_poses',
                    offset=[-(gripper_length + pre_grasp_offset), 0, 0],
                    in_poses_key='filtered_grasploc',
                    out_poses_key='offset_grasploc', debug=True), # The gripper origin is in the wrist so move out along x
        # GetTableAct('act_get_table'),
        FreeSpaceFinderAct('act_find_free_space', object_index = object_index),
        GetStackPoseAct('act_get_stack_pose'),
        MoveToStartAct('act_grasploc_pick', 'offset_grasploc'),

        grasploc_grasp_fall,
    ])
    return root

def PlaceAct(name, obj, surface):
    """
    Composite place action for an object
    """

    # Initialize the root as sequence node
    root = py_trees.composites.Sequence(
        name='{}_place_{}_from_{}'.format(name, obj, surface),
        children=None)
    object_index = obj
    relative_object_index = surface

    place_act_fall = py_trees.composites.Selector('place_act_fall')
    place_act_seq = py_trees.composites.Sequence('place_act_seq')
    place_act_seq.add_children([
        OffsetPoses('act_offset_place_poses',
                    offset=[-(gripper_length + pre_grasp_offset), 0, 0],
                    in_poses_key='free_space_poses',
                    out_poses_key='offset_place_pose', debug=True),

        MoveToFirstPoseAct('act_pick_grasploc', poses_key='offset_place_pose', place_if=True, object_index=object_index),
        SleepBehavior('act_sleep_a_smidge', duration=1.5),
        RelativeCartesianMoveAct('act_move_to_grasp_pose', pose_diff_msg=to_place_tf),
        SleepBehavior('act_sleep_a_smidge', duration=1),
        OpenGripperAct('act_open_gripper'),
        LinkDettachingAct('LinkAttachAct', object_name=object_index, link_name='link_1'),
        py_trees_ros.subscribers.ToBlackboard(
            name='act_get_groundtruth',
            topic_name='/gazebo/model_states',
            topic_type=ModelStates,
            blackboard_variables={'model_state': None},
            clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE),
        StabilizeObjectAct('StabilizeObjectAct', obj_name=obj),

        # TeleportObjectAct('TeleportObject', obj_name=object_index),
        AttachObjectAct('act_dettach_object', object_name='inhand_collision_object', to_attach=False),
        RemoveCollisionBoxAct('act_remove_inhand_collision_object', box_name='inhand_collision_object'),

        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        GetFakeDopeSnapshotAct('act_get_dope_snapshot'),
        AddAllObjectCollisionBoxAct('act_add_all_object_collision_box'),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        RelativeCartesianMoveAct('act_move_up', pose_diff_msg=up_tf_after_place),

        # StabilizeObjectAct('StabilizeObjectAct', obj_name=obj),
        place_act_fall,
        SleepBehavior('act_sleep_a_smidge', duration=1),
        RemoveAllCollisionBoxAct('act_Remove_All_Object_CollisionBox')
    ])

    place_act_fall.add_children([
        TuckBehavior(name='act_tuck_arm', tuck_pose='tuck'),
        py_trees.meta.success_is_failure(RelativeCartesianMoveAct)(name='ToTuckRelativeCartesianMoveAct', pose_diff_msg=up_tf_little),
        TuckBehavior(name='act_tuck_arm_second_try', tuck_pose='tuck'),
        py_trees.meta.success_is_failure(RelativeCartesianMoveAct)(name='ToTuckRelativeCartesianMoveAct', pose_diff_msg=back_tf),
        TuckBehavior(name='act_tuck_arm_third_try', tuck_pose='tuck'),
        py_trees.meta.success_is_failure(RelativeCartesianMoveAct)(name='ToTuckRelativeCartesianMoveAct', pose_diff_msg=down_tf),
        TuckBehavior(name='act_tuck_arm_fourth_try', tuck_pose='tuck'),
        py_trees.meta.success_is_failure(RelativeCartesianMoveAct)(name='ToTuckRelativeCartesianMoveAct', pose_diff_msg=right_tf),
        TuckBehavior(name='act_tuck_arm_fifth_try', tuck_pose='tuck'),
        py_trees.meta.success_is_failure(RelativeCartesianMoveAct)(name='ToTuckRelativeCartesianMoveAct', pose_diff_msg=left_tf),
        TuckBehavior(name='act_tuck_arm_sixth_try', tuck_pose='tuck'),
    ])
    return place_act_seq


class InfiniteDopeAct(py_trees_ros.actions.ActionClient):
    def __init__(self, name, *argv, **kwargs):
        super(InfiniteDopeAct, self).__init__(
            name=name,
            action_spec=DopeAction,
            action_goal=DopeGoal(),
            action_namespace='dope',
            *argv,
            **kwargs
        )
        self.image_sub = rospy.Subscriber('/head_camera/rgb/image_raw', ImageSensor_msg, self.image_callback)
        self.info_sub = rospy.Subscriber('/head_camera/rgb/camera_info', CameraInfo, self.initial_callback)

    def initial_callback(self, camera_info):
        self.action_goal = DopeGoal()
        self.action_goal.cam_info = camera_info
        self.info_sub.unregister()

    def image_callback(self, image):
        self.action_goal.image = image

    def update(self):
        self.logger.debug("{0}.update()".format(self.__class__.__name__))
        if not self.action_client:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            return py_trees.Status.INVALID

        if not self.sent_goal:
            if self.action_goal.cam_info != CameraInfo() and self.action_goal.image != ImageSensor_msg():
                self.action_client.send_goal(self.action_goal)
                self.sent_goal = True
                self.feedback_message = "sent goal to the action server"
            return py_trees.Status.RUNNING

        if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
                                              actionlib_msgs.GoalStatus.PREEMPTED]:
            return py_trees.Status.FAILURE

        result = self.action_client.get_result()
        if result:
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = self.override_feedback_message_on_running
            return py_trees.Status.RUNNING


class GenerateGoalAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, srp, init_scene_key=None):
        """!
        @brief      Constructs a new instance.

        @param      name            The name displayed in the behavior tree visualizer
        @param      srp             A shared srp_md instance
        @param      init_scene_key  The blackboard key for the initial scene, if None then it is assumed that the srp_md
                                    object has an initial scene already
        """
        super(GenerateGoalAct, self).__init__(name)
        self._srp = srp
        self._thread = None
        self._init_scene_key = init_scene_key

    def setup(self, timeout):
        return True

    def initialise(self):
        # Get an initial scene form the blackboard
        if self._init_scene_key is not None:
            blackboard = py_trees.blackboard.Blackboard()
            self._srp.set_scenes(None, blackboard.get(self._init_scene_key))

        if self._thread is None:
            self._thread = threading.Thread(target=self._srp.generate_goal)
            self._thread.start()

    def update(self):
        # Wait until goal is generated
        if self._thread.is_alive():
            return py_trees.common.Status.RUNNING
        # Goal generated (or failure)
        self._thread = None
        if None in self._srp._goal_instances or len(self._srp._goal_instances) == 0:
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS


class PlanAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, srp, plan_key='plan'):
        """!
        @brief      Constructs a new instance.

        @param      name            The name displayed in the behavior tree visualizer
        @param      srp             A shared srp_md instance
        @param      plan_key        The blackboard key to store the plan at

        """
        super(PlanAct, self).__init__(name)
        self._srp = srp
        self._thread = None
        self._plan_key = plan_key

    def setup(self, timeout):
        return True

    def initialise(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._srp.plan)
            self._thread.start()

    def update(self):
        # Wait until goal is generated
        if self._thread.is_alive():
            return py_trees.common.Status.RUNNING
        # Goal generated (or failure)
        self._thread = None
        # Check for faliure
        if None in self._srp._plans or len(self._srp._plans) == 0:
            return py_trees.common.Status.FAILURE
        # Write plan to blackboard and return success
        py_trees.blackboard.Blackboard().set(self._plan_key, self._srp._plans[0])
        return py_trees.common.Status.SUCCESS


class _ExecutePlanAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, tree, plan_root_id, plan_key='plan', timeout=1):
        super(_ExecutePlanAct, self).__init__(name)
        self._tree = tree
        self._plan_root_id = plan_root_id
        self._plan_key = plan_key
        self._thread = None
        self._timeout = timeout
        self._plan_root = None

    def setup(self, timeout):
        self._timeout = timeout
        return True

    def _build_tree(self, plan):
        # No plan means failure
        if plan is None:
            self._plan_root = None
            return

        # Construct the plan as a behavior tree
        # TODO(Kevin): Convert plan to bt
        self._plan_root =py_trees.composites.Sequence('seq_{}_plan_root'.format(self.name))
        self._plan_root.add_children([
            py_trees.behaviours.Success('act_success')
        ])
        # If failed to setup then fail
        if not self._plan_root.setup(self._timeout):
            self._plan_root = None

    def initialise(self):
        if self._thread is None:
            self._thread = threading.Thread(
                target=self._build_tree, args=(py_trees.blackboard.Blackboard().get(self._plan_key),))
            self._thread.start()

    def update(self):
        # Wait until tree generated
        if self._thread.is_alive():
            return py_trees.common.Status.RUNNING
        # Plan converted (or failure)
        self._thread = None
        # Check for failure
        if self._plan_root is None:
            return py_trees.common.Status.FAILURE
        # Update the tree with the new plan
        self._tree.replace_subtree(self._plan_root_id, self._plan_root)
        self._plan_root_id = self._plan_root.id
        return py_trees.common.Status.SUCCESS


def ExecutePlanAct(name, tree, timeout=1):
    root =py_trees.composites.Sequence('seq_{}_root'.format(name))
    dummy =py_trees.composites.Sequence('seq_{}_dummy_node'.format(name))
    root.add_children([
        _ExecutePlanAct(name, tree, dummy.id),
        dummy
    ])
    return root
