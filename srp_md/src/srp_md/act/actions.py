""" Actions.

Define the actions used in srp_md.

"""
#
from copy import deepcopy

import rospy
import rospkg
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
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel, DeleteModelRequest

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

rospack = rospkg.RosPack()
srp_md_path = rospack.get_path('srp_md')

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

to_y_grasp_full_tf = TransformStamped()
to_y_grasp_full_tf.header.frame_id = "gripper_link"
to_y_grasp_full_tf.transform.translation.y = (pre_grasp_offset) * 0.95
to_y_grasp_full_tf.transform.rotation.w = 1.0

to_y_grasp_full_back_tf = TransformStamped()
to_y_grasp_full_back_tf.header.frame_id = "gripper_link"
to_y_grasp_full_back_tf.transform.translation.y = -(pre_grasp_offset) * 0.95
to_y_grasp_full_back_tf.transform.rotation.w = 1.0

to_place_tf = TransformStamped()
to_place_tf.header.frame_id = "gripper_link"
to_place_tf.transform.translation.x = pre_grasp_offset * 0.9
to_place_tf.transform.rotation.w = 1.0

to_push_tf = TransformStamped()
to_push_tf.header.frame_id = "base_link"
# to_push_tf.transform.translation.x = pre_grasp_offset * 1.2
to_push_tf.transform.translation.y = 0.1
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

to_move_away_tf_1 = TransformStamped()
to_move_away_tf_1.header.frame_id = "base_link"
to_move_away_tf_1.transform.translation.y = 0.55
# to_move_away_tf_1.transform.translation.x = -0.35
to_move_away_tf_1.transform.rotation.w = 1.0

to_move_away_tf_2 = TransformStamped()
to_move_away_tf_2.header.frame_id = "base_link"
# to_move_away_tf_2.transform.translation.y = 0.55
to_move_away_tf_2.transform.translation.x = -0.35
to_move_away_tf_2.transform.rotation.w = 1.0

to_grab_box_back_tf = TransformStamped()
to_grab_box_back_tf.header.frame_id = "gripper_link"
to_grab_box_back_tf.transform.translation.x = -(0.55 - 0.35 + 0.05)
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
up_tf_after_place.transform.translation.z = pre_grasp_offset * 1.2
# up_tf_after_place.transform.translation.x = -0.15
up_tf_after_place.transform.rotation.w = 1

up_tf = TransformStamped()
up_tf.header.frame_id = "base_link"
# up_tf.transform.translation.z = 0.25
up_tf.transform.translation.z = 0.21
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

class SetValueToBlackBoardAct(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(SetValueToBlackBoardAct, self).__init__(name)

    def update(self):
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

        conveyor_belt_size = Vector3(0.6, 5.0, 0.3)
        py_trees.blackboard.Blackboard().set('conveyor_belt_size', conveyor_belt_size)

        grocery_collision_box_name = []
        grocery_collision_box_name.append('grocery_collision_box_left')
        grocery_collision_box_name.append('grocery_collision_box_right')
        grocery_collision_box_name.append('grocery_collision_box_front')
        grocery_collision_box_name.append('grocery_collision_box_back')
        py_trees.blackboard.Blackboard().set('grocery_collision_box_name', grocery_collision_box_name)

        All_object_offset = []
        All_object_offset.append(Vector3(-0.014, -0.014 + 0.01, 0.1 - 0.098))
        All_object_offset.append(Vector3(0, 0, 0.0145 - 0.014))
        All_object_offset.append(Vector3(-0.032, -0.026, 0.038))
        All_object_offset.append(Vector3(-0.016, -0.025, 0.095))
        All_object_offset.append(Vector3(-0.01, 0.0, 0.053))
        All_object_offset.append(Vector3(-0.0095, -0.017 + 0.015, 0.088 - 0.088))
        py_trees.blackboard.Blackboard().set('All_object_offset', All_object_offset)

        All_object_name = []
        All_object_name.append("cracker")
        All_object_name.append("gelatin")
        All_object_name.append("meat")
        All_object_name.append("mustard")
        All_object_name.append("soup")
        All_object_name.append("sugar")
        py_trees.blackboard.Blackboard().set('All_object_name', All_object_name)

        All_object_size = []
        All_object_size.append(Vector3(0.0718, 0.164, 0.213))
        All_object_size.append(Vector3(0.09, 0.097, 0.029))
        All_object_size.append(Vector3(0.103, 0.057, 0.084))
        All_object_size.append(Vector3(0.08, 0.06, 0.19))
        All_object_size.append(Vector3(0.068, 0.068, 0.102))
        All_object_size.append(Vector3(0.045, 0.097, 0.174))
        py_trees.blackboard.Blackboard().set('All_object_size', All_object_size)

        All_Table_size = []
        All_Table_size.append(Vector3(0.8, 1.5, 0.55))
        All_Table_size.append(Vector3(0.75, 0.4, 0.35))
        py_trees.blackboard.Blackboard().set('All_Table_size', All_Table_size)

        return py_trees.Status.SUCCESS

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
    def __init__(self, name, poses_key,pick_if=False, place_if=False, object_index=None, *argv, **kwargs):
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
        self._pick_if = pick_if
        self._poses = None
        self._object_index = object_index

    def initialise(self):
        super(MoveToFirstPoseAct, self).initialise()
        self.action_goal.poses = py_trees.blackboard.Blackboard().get(self._poses_key)
        self._poses = py_trees.blackboard.Blackboard().get(self._poses_key)

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
        if result is not None:
            if result.success:
                # print('result.pose_index: ', result.pose_index)
                if self._pick_if:
                    py_trees.blackboard.Blackboard().set('pose_index_parformed', result.pose_index)

                if self._place_if:
                    obj_bboxes_moved = py_trees.blackboard.Blackboard().get('obj_bboxes')
                    obj_bboxes_moved[self._object_index].center.position.x = self._poses[result.pose_index].position.x
                    obj_bboxes_moved[self._object_index].center.position.y = self._poses[result.pose_index].position.y
                    py_trees.blackboard.Blackboard().set('obj_bboxes', obj_bboxes_moved)
                return py_trees.Status.SUCCESS
            else:
                pass
        else:
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

class RelativeCartesianMoveBlackboardAct(py_trees_ros.actions.ActionClient):
    def __init__(self, name, pose_key='relative_cartesian_move_key', *argv, **kwargs):
        super(RelativeCartesianMoveBlackboardAct, self).__init__(
            name=name,
            action_spec=RelativeCartesianMoveAction,
            action_goal=RelativeCartesianMoveGoal(),
            action_namespace='relative_cartesian_move',
            *argv,
            **kwargs
        )
        self._pose_key = pose_key
        self._pose = None

    def initialise(self):
        super(RelativeCartesianMoveBlackboardAct, self).initialise()
        self._pose = py_trees.blackboard.Blackboard().get(self._pose_key)
        self.action_goal.pose_diff_msg = self._pose

    def update(self):
        self.logger.debug("{0}.update()".format(self.__class__.__name__))
        if not self.action_client:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            return py_trees.Status.INVALID
        # Ensure we have a poeses
        if self.action_goal.pose_diff_msg is None:
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
        if result is not None:
            if result.success:
                return py_trees.Status.SUCCESS
            else:
                pass
        else:
            return py_trees.Status.RUNNING

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
        self._fetch_pose = None
        self._marker = None
        self._grocery_box_size = None
        self._conveyor_belt_size = None
        self._object_name_compensate_num = 6
        self._All_object_offset_key = 'All_object_offset'
        self._All_object_name_key = 'All_object_name'
        self._All_object_size_key = 'All_object_size'
        self._All_Table_size_key = 'All_Table_size'

    def setup(self, timeout):
        # self._dope_detection_pub = rospy.Publisher('/original_grasploc_poses', DopeActionResult, queue_size=10)
        self._marker_sub = rospy.Subscriber('/dope/markers', MarkerArray, self.initial_callback)
        self._dope_detection_pub = rospy.Publisher('/dope/markers', MarkerArray, queue_size=10)
        self._set_model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        return True

    def initial_callback(self, marker):
        self._marker = marker

    def update(self):
        self._model_state = py_trees.blackboard.Blackboard().get('model_state')
        self._grocery_box_size = py_trees.blackboard.Blackboard().get('grocery_box_size')
        self._conveyor_belt_size = py_trees.blackboard.Blackboard().get('conveyor_belt_size')
        self._obj_bboxes = dict()
        object_name = []
        object_index = []
        # cracker gelatin meat mustard soup sugar
        All_offset = deepcopy(py_trees.blackboard.Blackboard().get(self._All_object_offset_key))

        All_size = deepcopy(py_trees.blackboard.Blackboard().get(self._All_object_size_key))

        Table_size = deepcopy(py_trees.blackboard.Blackboard().get(self._All_Table_size_key))
        table_size_z = 0.01

        All_name = deepcopy(py_trees.blackboard.Blackboard().get(self._All_object_name_key))

        dope_result = DopeResult()

        plane_bboxes = []
        grocery_bboxes = []
        conveyor_belt_bboxes = []
        grocery_bbox_names = []
        if self._model_state is None:
            return py_trees.Status.FAILURE

        for name in self._model_state.name:
            if name == "ground_plane" or 'wall' in name:
                continue
            elif "table" in name:
                table_bbox = BoundingBox3D()

                table_pose = self._model_state.pose[self._model_state.name.index(name)]
                orientation_table_world = R.from_quat([
                    table_pose.orientation.x,
                    table_pose.orientation.y,
                    table_pose.orientation.z,
                    table_pose.orientation.w])
                orientation_fetch = R.from_quat([
                    self._fetch_pose.orientation.x,
                    self._fetch_pose.orientation.y,
                    self._fetch_pose.orientation.z,
                    self._fetch_pose.orientation.w])

                transformed_base_link_orientation = (orientation_fetch.inv() * orientation_table_world).as_quat()
                transformed_base_link_position = orientation_fetch.inv().apply([-self._fetch_pose.position.x + table_pose.position.x, -self._fetch_pose.position.y + table_pose.position.y, -self._fetch_pose.position.z + table_pose.position.z])
                transformed_base_link_pose = Pose()
                transformed_base_link_pose.orientation = Quaternion(transformed_base_link_orientation[0], transformed_base_link_orientation[1], transformed_base_link_orientation[2], transformed_base_link_orientation[3])
                transformed_base_link_pose.position = Point(transformed_base_link_position[0], transformed_base_link_position[1], transformed_base_link_position[2])


                # table_bbox.center = self._model_state.pose[self._model_state.name.index(name)]
                table_bbox.center = transformed_base_link_pose
                if 'lower' in name:
                    table_bbox.size = Table_size[1]
                else:
                    table_bbox.size = Table_size[0]
                # specialized for table using now
                table_bbox.center.position.z = table_bbox.center.position.z + table_bbox.size.z
                # specialized for gazebo
                # table_bbox.center.position.y = 0
                table_bbox.size.z = table_size_z
                plane_bboxes.append(table_bbox)
                continue
            elif name == "fetch":
                self._fetch_pose = self._model_state.pose[self._model_state.name.index(name)]
                continue
            elif "conveyor" in name:
                conveyor_belt_pose = self._model_state.pose[self._model_state.name.index(name)]
                orientation_conveyor_belt_world = R.from_quat([
                    conveyor_belt_pose.orientation.x,
                    conveyor_belt_pose.orientation.y,
                    conveyor_belt_pose.orientation.z,
                    conveyor_belt_pose.orientation.w])
                orientation_fetch = R.from_quat([
                    self._fetch_pose.orientation.x,
                    self._fetch_pose.orientation.y,
                    self._fetch_pose.orientation.z,
                    self._fetch_pose.orientation.w])

                transformed_base_link_orientation = (orientation_fetch.inv() * orientation_conveyor_belt_world).as_quat()
                transformed_base_link_position = orientation_fetch.inv().apply([-self._fetch_pose.position.x + conveyor_belt_pose.position.x, -self._fetch_pose.position.y + conveyor_belt_pose.position.y, -self._fetch_pose.position.z + conveyor_belt_pose.position.z])
                transformed_base_link_pose = Pose()
                transformed_base_link_pose.orientation = Quaternion(transformed_base_link_orientation[0], transformed_base_link_orientation[1], transformed_base_link_orientation[2], transformed_base_link_orientation[3])
                transformed_base_link_pose.position = Point(transformed_base_link_position[0], transformed_base_link_position[1], transformed_base_link_position[2])

                conveyor_belt_bbox = BoundingBox3D()
                conveyor_belt_bbox.center = transformed_base_link_pose
                conveyor_belt_bbox.center.position.z = conveyor_belt_bbox.center.position.z + self._conveyor_belt_size.z / 2
                conveyor_belt_bbox.size = self._conveyor_belt_size
                conveyor_belt_bboxes.append(conveyor_belt_bbox)
                continue
            elif "box" in name:
                grocery_box_pose = self._model_state.pose[self._model_state.name.index(name)]
                orientation_grocery_box_world = R.from_quat([
                    grocery_box_pose.orientation.x,
                    grocery_box_pose.orientation.y,
                    grocery_box_pose.orientation.z,
                    grocery_box_pose.orientation.w])
                orientation_fetch = R.from_quat([
                    self._fetch_pose.orientation.x,
                    self._fetch_pose.orientation.y,
                    self._fetch_pose.orientation.z,
                    self._fetch_pose.orientation.w])

                transformed_base_link_orientation = (orientation_fetch.inv() * orientation_grocery_box_world).as_quat()
                transformed_base_link_position = orientation_fetch.inv().apply([-self._fetch_pose.position.x + grocery_box_pose.position.x, -self._fetch_pose.position.y + grocery_box_pose.position.y, -self._fetch_pose.position.z + grocery_box_pose.position.z])
                transformed_base_link_pose = Pose()
                transformed_base_link_pose.orientation = Quaternion(transformed_base_link_orientation[0], transformed_base_link_orientation[1], transformed_base_link_orientation[2], transformed_base_link_orientation[3])
                transformed_base_link_pose.position = Point(transformed_base_link_position[0], transformed_base_link_position[1], transformed_base_link_position[2])

                grocery_bbox = BoundingBox3D()
                # grocery_bbox.center = self._model_state.pose[self._model_state.name.index(name)]
                grocery_bbox.center = transformed_base_link_pose
                # specialized for the grocery using now
                grocery_bbox.center.position.z = grocery_bbox.center.position.z + self._grocery_box_size.z / 2
                grocery_bbox.size = self._grocery_box_size
                grocery_bboxes.append(grocery_bbox)
                grocery_bbox_names.append(name)
                continue
            else:
                continue_if = True
                if 'test_' in name:
                    self._object_name_compensate_num = 6
                elif 'test' in name:
                    self._object_name_compensate_num = 4
                else:
                    self._object_name_compensate_num = 0
                for name_index in range(len(All_name)):
                    if All_name[name_index] in name:
                        continue_if = False
                        if All_name[name_index] + '_test' == name or All_name[name_index] == name:
                            object_name.append(All_name[name_index] + '_0')
                            object_index.append(name_index + 1)
                        else:
                            object_name.append(All_name[name_index] + '_' + str(int(name[len(All_name[name_index]) + self._object_name_compensate_num:]) + 1))
                            object_index.append(name_index + 1)

            dope_result.detections.append(Detection3D())
            index_cur = len(dope_result.detections) - 1
            dope_result.detections[index_cur].results.append(ObjectHypothesisWithPose())
            dope_result.detections[index_cur].results[0].id = object_index[len(object_index) - 1]
            dope_result.detections[index_cur].results[0].score = 0.9

            pose_object = self._model_state.pose[self._model_state.name.index(name)]
            pose_ori = R.from_quat([
                pose_object.orientation.x,
                pose_object.orientation.y,
                pose_object.orientation.z,
                pose_object.orientation.w])
            pose_rotated = R.from_quat([0.5, 0.5 ,0.5 ,0.5])

            pose_real = pose_ori * pose_rotated
            pose_real_quat = pose_real.as_quat()
            transformed_pose = Pose()
            size_object = Vector3()
            transformed_pose.orientation.x = pose_real_quat[0]
            transformed_pose.orientation.y = pose_real_quat[1]
            transformed_pose.orientation.z = pose_real_quat[2]
            transformed_pose.orientation.w = pose_real_quat[3]
            offset = pose_ori.apply([All_offset[object_index[len(object_index) - 1] - 1].x, All_offset[object_index[len(object_index) - 1] - 1].y, All_offset[object_index[len(object_index) - 1] - 1].z])
            transformed_pose.position.x = pose_object.position.x + offset[0]
            transformed_pose.position.y = pose_object.position.y + offset[1]
            transformed_pose.position.z = pose_object.position.z + offset[2]

            size_object.x = All_size[object_index[len(object_index) - 1] - 1].y
            size_object.y = All_size[object_index[len(object_index) - 1] - 1].z
            size_object.z = All_size[object_index[len(object_index) - 1] - 1].x

            orientation_obejct_world = R.from_quat([
                transformed_pose.orientation.x,
                transformed_pose.orientation.y,
                transformed_pose.orientation.z,
                transformed_pose.orientation.w])
            orientation_fetch = R.from_quat([
                self._fetch_pose.orientation.x,
                self._fetch_pose.orientation.y,
                self._fetch_pose.orientation.z,
                self._fetch_pose.orientation.w])

            transformed_base_link_orientation = (orientation_fetch.inv() * orientation_obejct_world).as_quat()
            transformed_base_link_position = orientation_fetch.inv().apply([-self._fetch_pose.position.x + transformed_pose.position.x, -self._fetch_pose.position.y + transformed_pose.position.y, -self._fetch_pose.position.z + transformed_pose.position.z])
            transformed_base_link_pose = Pose()
            transformed_base_link_pose.orientation = Quaternion(transformed_base_link_orientation[0], transformed_base_link_orientation[1], transformed_base_link_orientation[2], transformed_base_link_orientation[3])
            transformed_base_link_pose.position = Point(transformed_base_link_position[0], transformed_base_link_position[1], transformed_base_link_position[2])

            dope_result.detections[index_cur].results[0].pose = transformed_base_link_pose
            dope_result.detections[index_cur].bbox.center = transformed_base_link_pose
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
            marker_temp.header.frame_id = "base_link"

            marker_temp.header.stamp.secs = 2672
            marker_temp.header.stamp.nsecs = 673000000
            marker_temp.header.stamp=rospy.Time.now()
            marker_temp.header.seq = sequence
            marker_temp.ns = 'bboxes'
            marker_temp.id = i + 1
            marker_temp.type = 1
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

        self._dope_detection_pub.publish(test_markerarray)
        print('plane_bboxes: ', plane_bboxes)
        py_trees.blackboard.Blackboard().set('plane_bboxes', plane_bboxes)
        py_trees.blackboard.Blackboard().set('grocery_bboxes', grocery_bboxes)
        py_trees.blackboard.Blackboard().set('conveyor_belt_bboxes', conveyor_belt_bboxes)
        py_trees.blackboard.Blackboard().set('grocery_bbox_names', grocery_bbox_names)
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
        self._All_object_offset_key = 'All_object_offset'
        self._All_object_name_key = 'All_object_name'


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
        self._obj_index = self._model_state.name.index(object_move_name)

        pose_first = self._free_space_poses[0]

        orientation_initial = self._model_state.pose[self._obj_index].orientation

        pose_rotated = R.from_quat([0.5, 0.5 ,0.5 ,0.5])

        All_offset = deepcopy(py_trees.blackboard.Blackboard().get(self._All_object_offset_key))

        All_name = deepcopy(py_trees.blackboard.Blackboard().get(self._All_object_name_key))

        pose_real = R.from_quat([
                orientation_initial.x,
                orientation_initial.y,
                orientation_initial.z,
                orientation_initial.w])

        object_name_index = All_name.index(self._obj_name[0:-2])
        offset = pose_real.apply([All_offset[object_name_index].x, All_offset[object_name_index].y, All_offset[object_name_index].z])

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
        self._All_object_offset_key = 'All_object_offset'
        self._All_object_name_key = 'All_object_name'
        self._All_object_size_key = 'All_object_size'

    def setup(self, timeout):
        self._set_model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        return True


    def update(self):
        self._model_state = py_trees.blackboard.Blackboard().get(self._model_state_key)
        test_grocery_box_size = py_trees.blackboard.Blackboard().get("grocery_box_size")

        self._relation = py_trees.blackboard.Blackboard().get(self._relation_key)
        self._free_space_poses = py_trees.blackboard.Blackboard().get(self._free_space_poses_key)
        if self._obj_name == 'table':
            return py_trees.Status.SUCCESS
        object_move_name = None

        object_move_name = self._obj_name[0:-1] + 'test_' + str(int(self._obj_name[-1]) - 1)
        if self._obj_name[-1] == '0':
            object_move_name = object_move_name[0:-3]

        self._obj_index = self._model_state.name.index(object_move_name)

        orientation_initial = self._model_state.pose[self._obj_index].orientation

        pose_rotated = R.from_quat([0.5, 0.5 ,0.5 ,0.5])

        All_size = deepcopy(py_trees.blackboard.Blackboard().get(self._All_object_size_key))
        All_offset = deepcopy(py_trees.blackboard.Blackboard().get(self._All_object_offset_key))
        All_name = deepcopy(py_trees.blackboard.Blackboard().get(self._All_object_name_key))

        pose_real = R.from_quat([
                orientation_initial.x,
                orientation_initial.y,
                orientation_initial.z,
                orientation_initial.w])

        object_name_index = All_name.index(self._obj_name[0:-2])

        offset = pose_real.apply([All_offset[object_name_index].x, All_offset[object_name_index].y, All_offset[object_name_index].z])

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

        dist = []
        for i in range(len(poses_3)):
            dist.append(srp_md.pose_difference(poses_3[i], current_pose))
        self._set_model_state.pose.orientation = poses_3[dist.index(min(dist))].orientation

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
            self._set_model_state.pose.position.y = self._free_space_poses[0].position.y + All_size[object_name_index].z / 4 * 3
            self._set_model_state.twist.angular.x = 0.23

        elif self._mode == 3:
            self._set_model_state.twist = self._model_state.twist[self._obj_index]
            self._set_model_state.pose = self._model_state.pose[self._obj_index]

            self._set_model_state.twist.linear = Vector3()
            self._set_model_state.twist.angular = Vector3()
            self._set_model_state.pose.orientation.x = 0.5
            self._set_model_state.pose.orientation.y = -0.5
            self._set_model_state.pose.orientation.z = 0.5
            self._set_model_state.pose.orientation.w = 0.5

        self._set_model_state_pub.publish(self._set_model_state)
        time.sleep(0.05)

        for name in self._model_state.name:
            if 'grocery' in name:
                box_index = self._model_state.name.index(name)
                set_box_state = ModelState()
                set_box_state.model_name = name
                set_box_state.twist = self._model_state.twist[box_index]
                set_box_state.pose = self._model_state.pose[box_index]
                print('set_box_state: ', set_box_state)
                dist_temp = []
                poses_3_temp = deepcopy(poses_3)
                for i in range(len(poses_3_temp)):
                    poses_3_temp[i].position = set_box_state.pose.position
                    if i == 0:
                        print('poses_3_temp[i]: ', poses_3_temp[i])
                    dist_temp.append(srp_md.pose_difference(poses_3_temp[i], set_box_state.pose))
                set_box_state.pose.orientation = poses_3_temp[dist_temp.index(min(dist_temp))].orientation
                print('dist_temp: ', dist_temp)
                # set_box_state.pose.orientation.x = 0
                # set_box_state.pose.orientation.y = 0
                # set_box_state.pose.orientation.z = 0
                # set_box_state.pose.orientation.w = 1
                self._set_model_state_pub.publish(set_box_state)
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
        self._conveyor_belt_size_key = 'conveyor_belt_size'

    def setup(self, timeout):
        self._set_model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        return True


    def update(self):
        self._model_state = py_trees.blackboard.Blackboard().get(self._model_state_key)

        self._obj_name = self._model_state.name

        conveyor_belt_index = None
        for name in self._obj_name:
            if 'belt' in name:
                conveyor_belt_index = self._obj_name.index(name)
                break
        # Assume the orientation is identity
        conveyor_belt_position = self._model_state.pose[conveyor_belt_index].position

        conveyor_belt_size = deepcopy(py_trees.blackboard.Blackboard().get(self._conveyor_belt_size_key))

        conveyor_belt_minx = conveyor_belt_position.x - conveyor_belt_size.x / 2
        conveyor_belt_maxx = conveyor_belt_position.x + conveyor_belt_size.x / 2
        conveyor_belt_miny = conveyor_belt_position.y - conveyor_belt_size.y / 2
        conveyor_belt_maxy = conveyor_belt_position.y + conveyor_belt_size.y / 2
        self._y_limit = conveyor_belt_maxy - 0.12

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
                dis_temp = np.sqrt((fetch_position.x - obj_position.x) * (fetch_position.x - obj_position.x) + (fetch_position.y - obj_position.y) * (fetch_position.y - obj_position.y))
                if obj_position.x <= conveyor_belt_maxx and obj_position.x >= conveyor_belt_minx and obj_position.y <= conveyor_belt_maxy and obj_position.y >= conveyor_belt_miny:
                    moving_object_name.append(self._obj_name[i])
                    dis.append(dis_temp)
                    y_value.append(obj_position.y)

        if (len(dis) == 0):
            return py_trees.Status.SUCCESS
        while (min(dis) > self._distance_limit and max(y_value) < self._y_limit):
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
                # time.sleep(self._sleep_time)
            time.sleep(self._sleep_time * 8)

        return py_trees.Status.SUCCESS

def callback_test(data):
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

        self._obj_name = self._model_state.name

        fetch_index = self._obj_name.index('fetch')
        fetch_pose = self._model_state.pose[fetch_index]
        fetch_twist = self._model_state.twist[fetch_index]

        set_box_state = ModelState()
        set_box_state.model_name = 'fetch'
        set_box_state.pose = fetch_pose
        # set_box_state.pose.position.x = set_box_state.pose.position.x - 1
        set_box_state.twist = fetch_twist
        set_box_state.twist.angular.z = 0.3
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

                self._set_model_state_pub.publish(set_box_state)
                break

            time.sleep(self._sleep_time)

        self._model_state = py_trees.blackboard.Blackboard().get(self._model_state_key)
        self._obj_name = self._model_state.name

        fetch_index = self._obj_name.index('fetch')
        fetch_pose = self._model_state.pose[fetch_index]
        fetch_twist = self._model_state.twist[fetch_index]
        set_box_state = ModelState()
        set_box_state.model_name = 'fetch'
        set_box_state.pose = fetch_pose
        # set_box_state.pose.position.x = set_box_state.pose.position.x - 1
        set_box_state.twist = fetch_twist
        set_box_state.twist.angular.z = 5
        self._set_model_state_pub.publish(set_box_state)
        time.sleep(self._sleep_time)

        return py_trees.Status.SUCCESS

class GroceryBoxInitializationAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, grocery_box_num=3, lower_table_size_key='All_Table_size', lower_table_name='lower_table', model_state_key='model_state'):
        super(GroceryBoxInitializationAct, self).__init__(name)
        self._set_model_state = None
        self._set_model_state_pub = None
        self._sleep_time = 0.005
        self._grocery_box_num = grocery_box_num
        self._spawn_model_client = None
        self._set_model_state_pub = None
        self._lower_table_size_key = lower_table_size_key
        self._lower_table_size = None
        self._lower_table_pose = None
        self._lower_table_name = lower_table_name
        self._model_state_key = model_state_key

    def setup(self, timeout):
        rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=timeout)
        self._spawn_model_client = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self._set_model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        return True


    def update(self):
        model_state = py_trees.blackboard.Blackboard().get(self._model_state_key)
        self._lower_table_size = py_trees.blackboard.Blackboard().get(self._lower_table_size_key)
        self._lower_table_pose = model_state.pose[model_state.name.index(self._lower_table_name)]
        lower_one = None
        min_z = 100
        for size in self._lower_table_size:
            if size.z < min_z:
                min_z = size.z
                lower_one = size
        self._lower_table_size = lower_one

        req = SpawnModelRequest()
        req.model_name = 'grocery_box'
        f = open(srp_md_path + '/model/grocery_models/grocery_box/model.sdf','r')
        req.model_xml = f.read()
        req.initial_pose.orientation = Quaternion(0,0,0.707,0.707)
        req.initial_pose.position.x = self._lower_table_pose.position.x
        req.initial_pose.position.y = self._lower_table_pose.position.y
        req.initial_pose.position.z = self._lower_table_size.z
        req.reference_frame = 'world'

        self._spawn_model_client.call(req)

        if self._grocery_box_num > 1:
            for i in range(self._grocery_box_num - 1):
                req = SpawnModelRequest()
                req.model_name = 'grocery_box' + '_' + str(i)
                f = open(srp_md_path + '/model/grocery_models/grocery_box/model.sdf','r')
                req.model_xml = f.read()
                req.initial_pose.orientation = Quaternion(0,0,0.707,0.707)
                req.initial_pose.position.x = self._lower_table_pose.position.x
                req.initial_pose.position.y = self._lower_table_pose.position.y
                req.initial_pose.position.z = self._lower_table_size.z + (i + 1) * 0.03
                req.reference_frame = 'world'

                self._spawn_model_client.call(req)

        # set_box_state = ModelState()
        # set_box_state.model_name = 'grocery_box_1'
        # set_box_state.pose.position.x = self._lower_table_pose.position.x
        # set_box_state.pose.position.y = self._lower_table_pose.position.y
        # set_box_state.pose.position.z = 0 + self._lower_table_size.z
        # set_box_state.pose.orientation = Quaternion(0,0,0.707,0.707)
        # self._set_model_state_pub.publish(set_box_state)
        # time.sleep(self._sleep_time)

        # set_box_state = ModelState()
        # set_box_state.model_name = 'grocery_box_0'
        # set_box_state.pose.position.x = self._lower_table_pose.position.x
        # set_box_state.pose.position.y = self._lower_table_pose.position.y
        # set_box_state.pose.position.z = 0.03 + self._lower_table_size.z
        # set_box_state.pose.orientation = Quaternion(0,0,0.707,0.707)
        # self._set_model_state_pub.publish(set_box_state)
        # time.sleep(self._sleep_time)

        # set_box_state = ModelState()
        # set_box_state.model_name = 'grocery_box'
        # set_box_state.pose.position.x = self._lower_table_pose.position.x
        # set_box_state.pose.position.y = self._lower_table_pose.position.y
        # set_box_state.pose.position.z = 0.06 + self._lower_table_size.z
        # set_box_state.pose.orientation = Quaternion(0,0,0.707,0.707)
        # self._set_model_state_pub.publish(set_box_state)
        # time.sleep(self._sleep_time)

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
        self._conveyor_belt_size_key = 'conveyor_belt_size'

    def setup(self, timeout):
        self._set_model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        return True


    def update(self):
        self._model_state = py_trees.blackboard.Blackboard().get(self._model_state_key)

        self._obj_name = self._model_state.name

        conveyor_belt_index = None
        for name in self._obj_name:
            if 'belt' in name:
                conveyor_belt_index = self._obj_name.index(name)
                break
        if conveyor_belt_index == None:
            return py_trees.Status.SUCCESS

        # Assume the orientation is identity
        conveyor_belt_position = self._model_state.pose[conveyor_belt_index].position

        conveyor_belt_size = deepcopy(py_trees.blackboard.Blackboard().get(self._conveyor_belt_size_key))
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
        poses_filtered_half = list(filter(lambda x: (x.position.z - offset + offset_upper_offset)<= cracker_box.center.position.z + cracker_box.size.y / 2,
                                offset_poses)
                             )
        poses_filtered_offset = list(filter(lambda x: x.position.z - offset >= cracker_box.center.position.z - cracker_box.size.y / 2 + offset_bottom_offset,
                                     poses_filtered_half)
                             )
        if self._debug:
            orig = PoseArray()
            new = PoseArray()
            orig.header.frame_id = 'base_link'
            new.header.frame_id = 'base_link'
            orig.poses = poses
            new.poses = offset_poses
            self._orig_pub.publish(orig)
            self._new_pub.publish(new)

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
        MoveToFirstPoseAct('act_{}_pick_grasploc'.format(name), poses_key=poses_key, pick_if=True)
    ])
    return root


class ChooseGrasplocObjAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, bbox_key='obj_bboxes', object_index=None, crop_box_key='crop_box',
                 inhand_collision_object_key='inhand_collision_object', obj_dim_key='object_dim'):
        super(ChooseGrasplocObjAct, self).__init__(name)
        self._bbox_key = bbox_key
        self._crop_box_key = crop_box_key
        self._object_index = object_index
        self._inhand_collision_object_key = inhand_collision_object_key
        self._name = name
        self._obj_dim_key = obj_dim_key

    def update(self):
        blackboard = py_trees.blackboard.Blackboard()
        bboxes = blackboard.get(self._bbox_key)
        if bboxes is None or len(bboxes) == 0:
            return py_trees.Status.FAILURE

        blackboard.set(self._crop_box_key, bboxes[self._object_index])
        size_box = bboxes[self._object_index].size
        collision_box_list = ['inhand_collision_object', bboxes[self._object_index].center, [size_box.x, size_box.y, size_box.z]]

        blackboard.set(self._inhand_collision_object_key, collision_box_list)
        blackboard.set(self._obj_dim_key, bboxes[self._object_index].size)
        return py_trees.Status.SUCCESS


class SetPlanValueAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, bbox_key='obj_bboxes', relative_object_index=None,
                 relation='None', crop_box_key='crop_box',
                 relative_obj_bbox_key='relative_bbox', relation_key='relation', distance_key='distance'):
        super(SetPlanValueAct, self).__init__(name)
        self._bbox_key = bbox_key
        self._crop_box_key = crop_box_key
        self._relative_obj_bbox_key = relative_obj_bbox_key
        self._distance_key = distance_key
        self._distance = 0.03
        self._relation_key = relation_key
        self._relation = None
        self._relative_object_index = relative_object_index
        self._relation = relation
        self._name = name
        self._points = None

    def update(self):
        blackboard = py_trees.blackboard.Blackboard()
        bboxes = blackboard.get(self._bbox_key)
        self._points = py_trees.blackboard.Blackboard().get('depth_downsampled')
        print("self._points: ", self._points.header.stamp)
        print("ros::time(): ", rospy.Time.now())
        self._points.header.stamp = rospy.Time.now()
        if bboxes is None or len(bboxes) == 0:
            return py_trees.Status.FAILURE

        if self._relation is None or self._relation == 'Random' or self._relative_object_index == 'table':
            self._relation = 'None'
            self._distance = 0
        elif self._relation == 'proximity':
            pass

        if not self._relative_object_index == 'table':
            blackboard.set(self._relative_obj_bbox_key, bboxes[self._relative_object_index])
        blackboard.set(self._relation_key, self._relation)
        blackboard.set(self._distance_key, self._distance)

        return py_trees.Status.SUCCESS


class PushPoseGeneration(py_trees.behaviour.Behaviour):
    def __init__(self, name, grocery_bboxes_key='grocery_bboxes', push_pose_key='push_pose',
                 grocery_box_index_key='grocery_box_index'):
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
        self._grocery_box_index = blackboard.get(self._grocery_box_index_key)
        # if not len(self._grocery_bboxes) == 0:
        #     dis = []
        #     for grocery_bbox in self._grocery_bboxes:
        #         grocery_position = grocery_bbox.center.position
        #         dis_temp = np.sqrt(grocery_position.x * grocery_position.x + grocery_position.y * grocery_position.y)
        #         dis.append(dis_temp)
        #     self._grocery_bbox = self._grocery_bboxes[dis.index(min(dis))]
        #     self._grocery_box_index = dis.index(min(dis))

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
        # blackboard.set(self._grocery_box_index_key, self._grocery_box_index)
        return py_trees.Status.SUCCESS

# Choose Grocery Box on the lower table
class ChooseGroceryBoxGrab(py_trees.behaviour.Behaviour):
    def __init__(self, name, grocery_bboxes_key='grocery_bboxes', grocery_box_index_key='grocery_box_index', model_state_key='model_state', lower_table_name='lower_table', lower_table_size_key='All_Table_size'):
        super(ChooseGroceryBoxGrab, self).__init__(name)
        self._grocery_bboxes_key = grocery_bboxes_key
        self._grocery_bboxes = None
        self._grocery_bbox = None
        self._grocery_box_index = None
        self._grocery_box_index_key = grocery_box_index_key
        self._lower_table_pose = None
        self._lower_table_size = None
        self._lower_table_size_key = lower_table_size_key
        self._model_state_key = model_state_key
        self._lower_table_name = lower_table_name
        self._fetch_pose = None

    def update(self):
        blackboard = py_trees.blackboard.Blackboard()
        self._grocery_bboxes = deepcopy(blackboard.get(self._grocery_bboxes_key))
        model_state = deepcopy(blackboard.get(self._model_state_key))
        self._lower_table_pose = model_state.pose[model_state.name.index(self._lower_table_name)]
        self._lower_table_size = deepcopy(blackboard.get(self._lower_table_size_key))
        self._fetch_pose = model_state.pose[model_state.name.index('fetch')]
        lower_one = None
        min_z = 100
        for size in self._lower_table_size:
            if size.z < min_z:
                min_z = size.z
                lower_one = size
        self._lower_table_size = lower_one

        lower_table_y_min = self._lower_table_pose.position.y - self._lower_table_size.y / 2
        lower_table_y_max = self._lower_table_pose.position.y + self._lower_table_size.y / 2
        lower_table_x_min = self._lower_table_pose.position.x - self._lower_table_size.x / 2
        lower_table_x_max = self._lower_table_pose.position.x + self._lower_table_size.x / 2

        box_on_lower_table_index = []
        box_on_lower_table = []
        chosen_box_index = None
        max_z = -1
        if not len(self._grocery_bboxes) == 0:
            for grocery_bbox in self._grocery_bboxes:
                grocery_position_fetch = grocery_bbox.center.position

                orientation_grocery_box_world = R.from_quat([
                    grocery_bbox.center.orientation.x,
                    grocery_bbox.center.orientation.y,
                    grocery_bbox.center.orientation.z,
                    grocery_bbox.center.orientation.w])
                orientation_fetch = R.from_quat([
                    self._fetch_pose.orientation.x,
                    self._fetch_pose.orientation.y,
                    self._fetch_pose.orientation.z,
                    self._fetch_pose.orientation.w])

                transformed_world_orientation = (orientation_fetch * orientation_grocery_box_world).as_quat()
                transformed_world_position = orientation_fetch.apply([grocery_position_fetch.x, grocery_position_fetch.y, grocery_position_fetch.z])

                grocery_position = Point(self._fetch_pose.position.x + transformed_world_position[0], self._fetch_pose.position.y + transformed_world_position[1], self._fetch_pose.position.z + transformed_world_position[2])

                print('grocery_position: ', grocery_position)
                if grocery_position.x <= lower_table_x_max and grocery_position.x >= lower_table_x_min\
                and grocery_position.y <= lower_table_y_max and grocery_position.y >= lower_table_y_min:
                    if grocery_position.z > max_z:
                        max_z = grocery_position.z
                        chosen_box_index = self._grocery_bboxes.index(grocery_bbox)
        self._grocery_box_index = chosen_box_index

        print('self._grocery_box_index: ', self._grocery_box_index)
        print('self._grocery_bboxes: ', self._grocery_bboxes[self._grocery_box_index])

        blackboard.set(self._grocery_box_index_key, self._grocery_box_index)
        return py_trees.Status.SUCCESS

class ChooseGroceryBoxPush(py_trees.behaviour.Behaviour):
    def __init__(self, name, grocery_bboxes_key='grocery_bboxes', grocery_box_index_key='grocery_box_index', model_state_key='model_state', table_name='table', table_size_key='All_Table_size'):
        super(ChooseGroceryBoxPush, self).__init__(name)
        self._grocery_bboxes_key = grocery_bboxes_key
        self._grocery_bboxes = None
        self._grocery_bbox = None
        self._grocery_box_index = None
        self._grocery_box_index_key = grocery_box_index_key
        self._table_pose = None
        self._table_size = None
        self._table_size_key = table_size_key
        self._model_state_key = model_state_key
        self._table_name = table_name
        self._fetch_pose = None

    def update(self):
        blackboard = py_trees.blackboard.Blackboard()
        self._grocery_bboxes = deepcopy(blackboard.get(self._grocery_bboxes_key))
        model_state = deepcopy(blackboard.get(self._model_state_key))
        self._table_pose = model_state.pose[model_state.name.index(self._table_name)]
        self._table_size = deepcopy(blackboard.get(self._table_size_key))
        self._fetch_pose = model_state.pose[model_state.name.index('fetch')]
        higher_one = None
        max_z = -100
        for size in self._table_size:
            if size.z > max_z:
                max_z = size.z
                higher_one = size
        self._table_size = higher_one

        table_y_min = self._table_pose.position.y - self._table_size.y / 2
        table_y_max = self._table_pose.position.y + self._table_size.y / 2
        table_x_min = self._table_pose.position.x - self._table_size.x / 2
        table_x_max = self._table_pose.position.x + self._table_size.x / 2

        box_on_table_index = []
        box_on_table = []
        chosen_box_index = None
        min_distance_to_fetch = 100
        if not len(self._grocery_bboxes) == 0:
            for grocery_bbox in self._grocery_bboxes:
                grocery_position_fetch = grocery_bbox.center.position

                orientation_grocery_box_world = R.from_quat([
                    grocery_bbox.center.orientation.x,
                    grocery_bbox.center.orientation.y,
                    grocery_bbox.center.orientation.z,
                    grocery_bbox.center.orientation.w])
                orientation_fetch = R.from_quat([
                    self._fetch_pose.orientation.x,
                    self._fetch_pose.orientation.y,
                    self._fetch_pose.orientation.z,
                    self._fetch_pose.orientation.w])

                transformed_world_orientation = (orientation_fetch * orientation_grocery_box_world).as_quat()
                transformed_world_position = orientation_fetch.apply([grocery_position_fetch.x, grocery_position_fetch.y, grocery_position_fetch.z])

                grocery_position = Point(self._fetch_pose.position.x + transformed_world_position[0], self._fetch_pose.position.y + transformed_world_position[1], self._fetch_pose.position.z + transformed_world_position[2])

                if grocery_position.x <= table_x_max and grocery_position.x >= table_x_min\
                and grocery_position.y <= table_y_max and grocery_position.y >= table_y_min:
                    distance_temp = np.sqrt((grocery_position.x - self._fetch_pose.position.x) * (grocery_position.x - self._fetch_pose.position.x)\
                        + (grocery_position.y - self._fetch_pose.position.y) * (grocery_position.y - self._fetch_pose.position.y))
                    if distance_temp < min_distance_to_fetch:
                        min_distance_to_fetch = distance_temp
                        chosen_box_index = self._grocery_bboxes.index(grocery_bbox)
        self._grocery_box_index = chosen_box_index

        print('self._grocery_box_index: ', self._grocery_box_index)
        print('self._grocery_bboxes: ', self._grocery_bboxes[self._grocery_box_index])

        blackboard.set(self._grocery_box_index_key, self._grocery_box_index)
        return py_trees.Status.SUCCESS

class PushBoxPoseGeneration(py_trees.behaviour.Behaviour):
    def __init__(self, name, grocery_bboxes_key='grocery_bboxes', push_pose_key='push_pose', grocery_box_index_key='grocery_box_index'):
        super(PushBoxPoseGeneration, self).__init__(name)
        self._grocery_bboxes_key = grocery_bboxes_key
        self._grocery_bboxes = None
        self._push_pose_key = push_pose_key
        self._grocery_bbox = None
        self._grocery_box_index = None
        self._grocery_box_index_key = grocery_box_index_key
        self._relative_cartesian_move_key = 'relative_cartesian_move_key'

    def update(self):
        blackboard = py_trees.blackboard.Blackboard()
        self._grocery_bboxes = blackboard.get(self._grocery_bboxes_key)
        self._grocery_box_index = blackboard.get(self._grocery_box_index_key)

        model_state = deepcopy(blackboard.get('model_state'))
        self._fetch_pose = model_state.pose[model_state.name.index('fetch')]

        test_gripper_poses = []

        self._grocery_bbox = self._grocery_bboxes[self._grocery_box_index]

        grocery_bbox_orientation = self._grocery_bbox.center.orientation

        tf = R.from_quat([
                grocery_bbox_orientation.x,
                grocery_bbox_orientation.y,
                grocery_bbox_orientation.z,
                grocery_bbox_orientation.w])
        # gripper_ori_relative = R.from_quat([0, 0.707, 0, 0.707])
        # gripper_ori_relative = R.from_quat([0.5, 0.5, -0.5, 0.5])
        gripper_ori_relative = R.from_quat([0, 0, 0, 1])
        offset = tf.apply([- self._grocery_bbox.size.x / 2,  self._grocery_bbox.size.y / 2 - 0.11, 0])
        # offset = tf.apply([self._grocery_bbox.size.x / 2, self._grocery_bbox.size.y / 2 - 0.05, 0])
        gripper_ori = (tf * gripper_ori_relative).as_quat()

        gripper_pose = Pose()
        # gripper_pose.orientation = Quaternion(0, 0.707, 0, 0.707)
        gripper_pose.orientation = Quaternion(gripper_ori[0], gripper_ori[1], gripper_ori[2], gripper_ori[3])

        # gripper_pose.position.x = self._grocery_bbox.center.position.x - self._grocery_bbox.size.x / 2
        # gripper_pose.position.y = self._grocery_bbox.center.position.y - self._grocery_bbox.size.y / 2
        gripper_pose.position.x = self._grocery_bbox.center.position.x + offset[0]
        gripper_pose.position.y = self._grocery_bbox.center.position.y + offset[1]
        gripper_pose.position.z = self._grocery_bbox.center.position.z + self._grocery_bbox.size.z / 2 - 0.02
        test_gripper_poses.append(gripper_pose)

        blackboard.set(self._push_pose_key, test_gripper_poses)

        relative_cartesian_move_dis = 0.4
        tf_1 = R.from_quat([
                self._fetch_pose.orientation.x,
                self._fetch_pose.orientation.y,
                self._fetch_pose.orientation.z,
                self._fetch_pose.orientation.w])
        offset_1 = (tf_1.inv()).apply([0, relative_cartesian_move_dis, 0])

        # print('offset_1: ', offset_1)

        to_push_tf_1 = TransformStamped()
        to_push_tf_1.header.frame_id = "base_link"
        # to_push_tf.transform.translation.x = pre_grasp_offset * 1.2
        to_push_tf_1.transform.translation.x = offset_1[0]
        to_push_tf_1.transform.translation.y = offset_1[1]
        to_push_tf_1.transform.rotation.w = 1.0
        blackboard.set(self._relative_cartesian_move_key, to_push_tf_1)
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
        self._grocery_box_index = blackboard.get(self._grocery_box_index_key)
        test_gripper_poses = []

        self._grocery_bbox = self._grocery_bboxes[self._grocery_box_index]

        grocery_bbox_orientation = self._grocery_bbox.center.orientation

        tf = R.from_quat([
                grocery_bbox_orientation.x,
                grocery_bbox_orientation.y,
                grocery_bbox_orientation.z,
                grocery_bbox_orientation.w])
        # gripper_ori_relative = R.from_quat([0, 0.707, 0, 0.707])
        gripper_ori_relative = R.from_quat([0.5, 0.5, -0.5, 0.5])
        offset = tf.apply([- self._grocery_bbox.size.x / 2, - self._grocery_bbox.size.y / 2 + 0.05, 0])
        # offset = tf.apply([self._grocery_bbox.size.x / 2, self._grocery_bbox.size.y / 2 - 0.05, 0])
        gripper_ori = (tf * gripper_ori_relative).as_quat()

        gripper_pose = Pose()
        # gripper_pose.orientation = Quaternion(0, 0.707, 0, 0.707)
        gripper_pose.orientation = Quaternion(gripper_ori[0], gripper_ori[1], gripper_ori[2], gripper_ori[3])

        # gripper_pose.position.x = self._grocery_bbox.center.position.x - self._grocery_bbox.size.x / 2
        # gripper_pose.position.y = self._grocery_bbox.center.position.y - self._grocery_bbox.size.y / 2
        gripper_pose.position.x = self._grocery_bbox.center.position.x + offset[0]
        gripper_pose.position.y = self._grocery_bbox.center.position.y + offset[1]
        gripper_pose.position.z = self._grocery_bbox.center.position.z + self._grocery_bbox.size.z / 2 - 0.02
        test_gripper_poses.append(gripper_pose)

        blackboard.set(self._grab_pose_key, test_gripper_poses)
        # blackboard.set(self._grocery_box_index_key, self._grocery_box_index)
        return py_trees.Status.SUCCESS

class GrabBoxPlacePoseGeneration(py_trees.behaviour.Behaviour):
    def __init__(self, name, grocery_bboxes_key='grocery_bboxes', grab_pose_key='grab_pose',
        grocery_box_index_key='grocery_box_index', model_state_key='model_state', table_size_key='All_Table_size', table_bbox_key='plane_bboxes'):
        super(GrabBoxPlacePoseGeneration, self).__init__(name)
        self._grocery_bboxes_key = grocery_bboxes_key
        self._grocery_bboxes = None
        self._grab_pose_key = grab_pose_key
        self._grocery_bbox = None
        self._grocery_box_index = None
        self._grocery_box_index_key = grocery_box_index_key
        self._table_size = None
        self._table_size_key = table_size_key
        self._model_state_key = model_state_key
        self._model_state = None
        self._fetch_pose = None
        self._table_bbox = None
        self._table_bbox_key = table_bbox_key


    def update(self):
        blackboard = py_trees.blackboard.Blackboard()
        self._grocery_bboxes = deepcopy(blackboard.get(self._grocery_bboxes_key))
        self._grocery_box_index = deepcopy(blackboard.get(self._grocery_box_index_key))
        self._model_state = deepcopy(blackboard.get(self._model_state_key))
        self._table_size = deepcopy(blackboard.get(self._table_size_key))
        self._fetch_pose =self._model_state.pose[self._model_state.name.index('fetch')]
        self._table_bbox = deepcopy(blackboard.get(self._table_bbox_key))
        test_gripper_poses = []

        max_z = -1
        table_bbox = None
        for bbox in self._table_bbox:
            if bbox.center.position.z > max_z:
                max_z = bbox.center.position.z
                table_bbox = bbox
        self._table_bbox = table_bbox

        self._grocery_bbox = self._grocery_bboxes[self._grocery_box_index]

        grocery_bbox_orientation = self._grocery_bbox.center.orientation

        tf = R.from_quat([
                self._table_bbox.center.orientation.x,
                self._table_bbox.center.orientation.y,
                self._table_bbox.center.orientation.z,
                self._table_bbox.center.orientation.w])
        # gripper_ori_relative = R.from_quat([0, 0.707, 0, 0.707])
        gripper_ori_relative = R.from_quat([0.5, 0.5, -0.5, 0.5])
        # offset = tf.apply([- self._table_bbox.size.x / 2 + self._grocery_bbox.size.x / 2, - self._table_bbox.size.y / 2 + self._grocery_bbox.size.y / 2, 0])
        offset = tf.apply([- self._table_bbox.size.x / 2 + 0.02, -self._table_bbox.size.y / 2 + 0.05, 0])
        gripper_ori = (tf * gripper_ori_relative).as_quat()

        gripper_pose = Pose()
        # gripper_pose.orientation = Quaternion(0, 0.707, 0, 0.707)
        gripper_pose.orientation = Quaternion(gripper_ori[0], gripper_ori[1], gripper_ori[2], gripper_ori[3])

        # gripper_pose.position.x = self._grocery_bbox.center.position.x - self._grocery_bbox.size.x / 2
        # gripper_pose.position.y = self._grocery_bbox.center.position.y - self._grocery_bbox.size.y / 2
        gripper_pose.position.x = self._table_bbox.center.position.x + offset[0]
        gripper_pose.position.y = self._table_bbox.center.position.y + offset[1]
        gripper_pose.position.z = self._table_bbox.center.position.z + self._grocery_bbox.size.z
        test_gripper_poses.append(gripper_pose)

        blackboard.set(self._grab_pose_key, test_gripper_poses)
        # blackboard.set(self._grocery_box_index_key, self._grocery_box_index)
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
        for i in range(len(test_gripper_poses)):
            norm = np.array([x_axises[i][0], x_axises[i][1], x_axises[i][2]])
            norm = norm / np.linalg.norm(norm)
            if np.dot(norm, self._center_axis) > self._min_cos_theta or np.dot(norm, self._center_axis) < -self._min_cos_theta:
                test_gripper_poses_filtered.append(test_gripper_poses[i])
                poses_z_value.append(test_gripper_poses[i].position.z)
            else:
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
        self.action_goal.bot_dim = self._relative_object_bbox.size
        self.action_goal.dim = py_trees.blackboard.Blackboard().get(self._obj_dim_key)
        self._object_dim = self.action_goal.dim
        self._relation = py_trees.blackboard.Blackboard().get(self._relation_key)

    def update(self):
        grasp_poses = deepcopy(py_trees.blackboard.Blackboard().get('filtered_grasploc'))
        pose_index_parformed = deepcopy(py_trees.blackboard.Blackboard().get('pose_index_parformed'))
        
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
            result.pose.orientation = grasp_poses[pose_index_parformed].orientation
            result.pose.position.y = result.pose.position.y
            result.pose.position.z = result.pose.position.z + self.action_goal.dim.y / 3.5
            # result.pose.position.z = result.pose.position.z + self.action_goal.dim.y
            upper_poses = []
            upper_poses.append(result.pose)
            # upper_poses[0].position.y = upper_poses[0].position.y - self._object_dim.y / 4
            py_trees.blackboard.Blackboard().set('free_space_poses', upper_poses)
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = self.override_feedback_message_on_running
            return py_trees.Status.RUNNING

class FreeSpaceFinderAct(py_trees_ros.actions.ActionClient):
    def __init__(self, name, obj_bbox_key='crop_box', plane_bboxes_key='plane_bboxes', relative_bbox_key = 'relative_bbox',
                 relation_key='relation', grocery_box_size_key='grocery_box_size',
                 grocery_bboxes_key='grocery_bboxes', distance_key='distance',
                 object_index=None, use_grocery_box='True',
                 pose_index_parformed_key='pose_index_parformed',
                 grasp_pose_key='grasp_pose', *argv, **kwargs):
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
        self._use_grocery_box = use_grocery_box
        self._plane_size_z = 0.01
        self._pose_index_parformed_key = pose_index_parformed_key
        self._pose_index_parformed = None

    def initialise(self):
        self.action_goal.points = deepcopy(py_trees.blackboard.Blackboard().get('depth_downsampled'))
        self._obj_bbox = deepcopy(py_trees.blackboard.Blackboard().get(self._obj_bbox_key))
        self._grocery_bboxes = deepcopy(py_trees.blackboard.Blackboard().get(self._grocery_bboxes_key))
        self._obj_dim = deepcopy(self._obj_bbox.size)
        grasp_poses = deepcopy(py_trees.blackboard.Blackboard().get('filtered_grasploc'))
        self.action_goal.distance = deepcopy(py_trees.blackboard.Blackboard().get(self._distance_key))
        self.action_goal.relative_obj_bbox = deepcopy(py_trees.blackboard.Blackboard().get(self._relative_bbox_key))
        if self.action_goal.relative_obj_bbox == None:
            self.action_goal.relative_obj_bbox = BoundingBox3D()
        self._relation = deepcopy(py_trees.blackboard.Blackboard().get(self._relation_key))
        self._grocery_box_size = deepcopy(py_trees.blackboard.Blackboard().get(self._grocery_box_size_key))
        self._pose_index_parformed = deepcopy(py_trees.blackboard.Blackboard().get(self._pose_index_parformed_key))
        self._grasp_pose = grasp_poses[self._pose_index_parformed]
        if self._obj_bbox is None:
            self.action_goal.obj_bbox = BoundingBox3D()
            return py_trees.Status.FAILURE
        else:
            self.action_goal.obj_bbox = self._obj_bbox
        self.action_goal.relation = 'test'
        self._plane_bboxes = deepcopy(py_trees.blackboard.Blackboard().get(self._plane_bboxes_key))
        # self._grocery_bbox.size = Vector3(self._grocery_box_size.x, self._grocery_box_size.y, self._grocery_box_size.z)
        self._grocery_bbox.size = deepcopy(self._grocery_box_size)
        if not self._use_grocery_box:
            if len(self._plane_bboxes) == 0:
                # self.action_goal.plane_bbox = BoundingBox3D()
                return py_trees.Status.FAILURE
            else:
                plane_distance_min = 100
                min_dis_index = -1
                for plane_index in range(len(self._plane_bboxes)):
                    distance_temp = self._plane_bboxes[plane_index].center.x * self._plane_bboxes[plane_index].center.x\
                                    + self._plane_bboxes[plane_index].center.y * self._plane_bboxes[plane_index].center.y
                    if distance_temp < plane_distance_min:
                        plane_distance_min = distance_temp
                        min_dis_index = plane_index
                self.action_goal.plane_bbox = deepcopy(self._plane_bboxes[min_dis_index])
                # self._grocery_bbox.center = deepcopy(self._plane_bboxes[min_dis_index].center)
                self._plane_bbox = self._plane_bboxes[min_dis_index]

        if self._use_grocery_box:
            if len(self._grocery_bboxes) == 0:
                return py_trees.Status.FAILURE
            dis = []
            for grocery_bbox in self._grocery_bboxes:
                grocery_position = grocery_bbox.center.position
                dis_temp = np.sqrt(grocery_position.x * grocery_position.x + grocery_position.y * grocery_position.y)
                dis.append(dis_temp)
            self._grocery_bbox = deepcopy(self._grocery_bboxes[dis.index(min(dis))])
            print('before: ', self._grocery_bbox)
            self._grocery_bbox.center.position.z = self._grocery_bbox.center.position.z - self._grocery_bbox.size.z / 2
            self._grocery_bbox.size.z = self._plane_size_z
            self.action_goal.plane_bbox = self._grocery_bbox

        grocery_box_manual_deduction = Vector3(0.0001, 0, 0)

        self.action_goal.plane_bbox.center.position.x = self.action_goal.plane_bbox.center.position.x
        self.action_goal.plane_bbox.center.position.y = self.action_goal.plane_bbox.center.position.y - grocery_box_manual_deduction.y

        self.action_goal.plane_bbox.size.x = self._grocery_box_size.x - grocery_box_manual_deduction.x * 2
        self.action_goal.plane_bbox.size.y = self._grocery_box_size.y - grocery_box_manual_deduction.y * 2

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
            self.action_client.send_goal(self.action_goal)
            self.sent_goal = True
            self.feedback_message = "sent goal to the action server"
            return py_trees.Status.RUNNING
        self.feedback_message = self.action_client.get_goal_status_text()
        if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
                                              actionlib_msgs.GoalStatus.PREEMPTED]:
            return py_trees.Status.FAILURE
        result = self.action_client.get_result()

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
                pose = deepcopy(result.pose[i])
                pose.orientation = self._grasp_pose.orientation
                pose.position.z = self._grasp_pose.position.z
                if pose.position.z - self.action_goal.plane_bbox.center.position.z > self._obj_bbox.size.y * 1.5:
                    pose.position.z = self.action_goal.plane_bbox.center.position.z + self._obj_bbox.size.y * 1.1
                # poses.append(pose)
                poses_temp.append(pose)

                distance_ori.append((pose.position.x - ori_position.x) * (pose.position.x - ori_position.x)
                                    + (pose.position.y - ori_position.y) * (pose.position.y - ori_position.y))
            if 'Away' in self._relation:
                # if 'Away' in self._relation:
                print("relation is Away")
                for i in range(len(poses_temp)):
                    pose_index = distance_ori.index(max(distance_ori))
                    x_rand = poses_temp[pose_index].position.x
                    y_rand = poses_temp[pose_index].position.y
                    add_if = True
                    for object_index in range(len(obj_bboxes)):
                        distance = np.sqrt((obj_bboxes_value[object_index].center.position.x - x_rand) * (obj_bboxes_value[object_index].center.position.x - x_rand)
                                           + (obj_bboxes_value[object_index].center.position.y - y_rand) * (obj_bboxes_value[object_index].center.position.y - y_rand))
                        if distance < Away_limit_dis:
                            add_if = False
                    if add_if:
                        poses.append(poses_temp[distance_ori.index(max(distance_ori))])
                    distance_ori[distance_ori.index(max(distance_ori))] = -1
            elif 'proximity' in self._relation and 'meat' in self._object_index:
                for i in range(len(poses_temp)):
                    if i > len(poses_temp) / 3:
                        pose_index = distance_ori.index(max(distance_ori))
                        poses.append(poses_temp[pose_index])
                        distance_ori[pose_index] = -1
            elif 'proximity' in self._relation:
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
        self.action_goal.in_pc = deepcopy(blackboard.get(self._in_pc_key))
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
        self.action_goal.in_pc = deepcopy(blackboard.get(self._in_pc_key))

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
        self._pc = deepcopy(blackboard.get(self._in_pc_key))

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
                 grocery_box_offset_key = 'grocery_box_offset', plane_bbox_key = 'plane_bboxes',
                 grocery_bboxes_key = 'grocery_bboxes', conveyor_belt_bboxes_key='conveyor_belt_bboxes'):
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
        self._plane_bbox_key = plane_bbox_key
        self._plane_bboxes = None
        self._conveyor_belt_bboxes_key = conveyor_belt_bboxes_key
        self._conveyor_belt_bboxes = None


    def setup(self, timeout):
        self._pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=1)
        return True

    def update(self):
        # Make the box
        blackboard = py_trees.blackboard.Blackboard()

        bboxes = blackboard.get(self._bbox_key)
        self._grocery_bboxes = deepcopy(blackboard.get(self._grocery_bboxes_key))
        self._grocery_box_size = deepcopy(blackboard.get(self._grocery_box_size_key))
        self._plane_bboxes = deepcopy(blackboard.get(self._plane_bbox_key))
        # print('len(self._plane_bboxes): ',self._plane_bboxes)
        self._conveyor_belt_bboxes = deepcopy(blackboard.get(self._conveyor_belt_bboxes_key))

        grocery_collision_box_name = blackboard.get('grocery_collision_box_name')

        planning_scene = PlanningScene()
        planning_scene.is_diff = True

        grocery_collsion_box_bboxes = []

        for grocery_bbox in self._grocery_bboxes:
            grocery_box_position = deepcopy(grocery_bbox.center.position)
            grocery_box_orientation = grocery_bbox.center.orientation
            grocery_box_size = grocery_bbox.size

            orientation_box = R.from_quat([
                    grocery_box_orientation.x,
                    grocery_box_orientation.y,
                    grocery_box_orientation.z,
                    grocery_box_orientation.w])

            left_bbox = BoundingBox3D()
            left_bbox.center = Pose()
            left_bbox.center.orientation = grocery_box_orientation

            left_offset = orientation_box.apply([0, grocery_box_size.y / 2, 0])

            left_bbox.center.position.z = grocery_box_position.z
            # left_bbox.center.position.x = grocery_box_position.x
            # left_bbox.center.position.y = grocery_box_position.y + grocery_box_size.y / 2
            left_bbox.center.position.x = grocery_box_position.x + left_offset[0]
            left_bbox.center.position.y = grocery_box_position.y + left_offset[1]
            left_bbox.size.x = self._grocery_box_size.x
            left_bbox.size.z = self._grocery_box_size.z
            left_bbox.size.y = 0.002

            right_bbox = BoundingBox3D()
            right_bbox.center = Pose()
            right_bbox.center.orientation = grocery_box_orientation
            right_offset = orientation_box.apply([0, -grocery_box_size.y / 2, 0])
            right_bbox.center.position.z = grocery_box_position.z
            right_bbox.center.position.x = grocery_box_position.x + right_offset[0]
            right_bbox.center.position.y = grocery_box_position.y + right_offset[1]
            right_bbox.size.x = self._grocery_box_size.x
            right_bbox.size.z = self._grocery_box_size.z
            right_bbox.size.y = 0.002

            front_bbox = BoundingBox3D()
            front_bbox.center = Pose()
            front_bbox.center.orientation = grocery_box_orientation
            front_offset = orientation_box.apply([-grocery_box_size.x / 2, 0, 0])
            front_bbox.center.position.z = grocery_box_position.z - self._grocery_box_size.z / 4
            front_bbox.center.position.x = grocery_box_position.x + front_offset[0]
            front_bbox.center.position.y = grocery_box_position.y + front_offset[1]
            front_bbox.size.y = self._grocery_box_size.y
            front_bbox.size.z = self._grocery_box_size.z / 2
            front_bbox.size.x = 0.002

            back_bbox = BoundingBox3D()
            back_bbox.center = Pose()
            back_bbox.center.orientation = grocery_box_orientation
            back_offset = orientation_box.apply([grocery_box_size.x / 2, 0, 0])
            back_bbox.center.position.z = grocery_box_position.z
            back_bbox.center.position.x = grocery_box_position.x + back_offset[0]
            back_bbox.center.position.y = grocery_box_position.y + back_offset[1]
            back_bbox.size.y = self._grocery_box_size.y
            back_bbox.size.z = self._grocery_box_size.z
            back_bbox.size.x = 0.002

            grocery_collsion_box_bboxes.append(left_bbox)
            grocery_collsion_box_bboxes.append(right_bbox)
            grocery_collsion_box_bboxes.append(front_bbox)
            grocery_collsion_box_bboxes.append(back_bbox)

        for i in range(len(grocery_collsion_box_bboxes)):
            self._box_name = grocery_collision_box_name[i % 4] + '_' + str(int(i // 4))
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

        for bbox in self._conveyor_belt_bboxes:
            plane_position = bbox.center.position
            plane_orientation = bbox.center.orientation
            plane_size = bbox.size

            self._box_name = 'conveyor_belt' + '_' + str(self._conveyor_belt_bboxes.index(bbox))
            self._box_pose = deepcopy(bbox.center)
            box_size = deepcopy(plane_size)
            box_size.z = plane_position.z
            self._box_size = [box_size.x + 0.01, box_size.y + 0.01, box_size.z + 0.01]
            self._box_pose.position.z = self._box_pose.position.z / 2
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

        for bbox in self._plane_bboxes:
            plane_position = bbox.center.position
            plane_orientation = bbox.center.orientation
            plane_size = bbox.size

            self._box_name = 'table_box' + '_' + str(self._plane_bboxes.index(bbox))
            self._box_pose = deepcopy(bbox.center)
            box_size = deepcopy(plane_size)
            box_size.z = plane_position.z
            self._box_size = [box_size.x + 0.01, box_size.y + 0.01, box_size.z + 0.01]
            self._box_pose.position.z = self._box_pose.position.z / 2
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

        for i in range(len(bboxes)):
            self._box_name = bboxes.keys()[i]
            self._box_pose = bboxes[self._box_name].center
            print('self._box_pose: ', self._box_pose)
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
            planning_scene.world.collision_objects.append(obj)

        # grocery box collision box

        self._pub.publish(planning_scene)
        return py_trees.Status.SUCCESS


class RemoveAllCollisionBoxAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, bbox_key='obj_bboxes', grocery_bboxes_key='grocery_bboxes', plane_bbox_key = 'plane_bboxes', conveyor_belt_bboxes_key='conveyor_belt_bboxes'):
        super(RemoveAllCollisionBoxAct, self).__init__(name)
        self._pub = None
        self._bbox_key = bbox_key
        self._box_name = None
        self._grocery_bboxes_key = grocery_bboxes_key
        self._grocery_bboxes = None
        self._plane_bbox_key = plane_bbox_key
        self._plane_bbox = None
        self._conveyor_belt_bboxes = None
        self._conveyor_belt_bboxes_key = conveyor_belt_bboxes_key

    def setup(self, timeout):
        self._pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=1)
        return True

    def update(self):
        # Make the box
        blackboard = py_trees.blackboard.Blackboard()
        bboxes = blackboard.get(self._bbox_key)
        self._grocery_bboxes = blackboard.get(self._grocery_bboxes_key)
        self._conveyor_belt_bboxes = blackboard.get(self._conveyor_belt_bboxes_key)
        grocery_collision_box_name = blackboard.get('grocery_collision_box_name')
        self._plane_bbox = blackboard.get(self._plane_bbox_key)
        planning_scene = PlanningScene()
        planning_scene.is_diff = True

        for i in range(len(self._plane_bbox)):
            obj = CollisionObject()
            obj.id = 'table_box' + '_' + str(i)
            obj.operation = obj.REMOVE

            planning_scene.world.collision_objects.append(obj)

        for i in range(len(self._conveyor_belt_bboxes)):
            obj = CollisionObject()
            obj.id = 'conveyor_belt' + '_' + str(i)
            obj.operation = obj.REMOVE

            planning_scene.world.collision_objects.append(obj)

        for i in range(len(self._grocery_bboxes)):
            print(i)
            obj = CollisionObject()
            obj.id = grocery_collision_box_name[0] + '_' + str(i)
            obj.operation = obj.REMOVE

            planning_scene.world.collision_objects.append(obj);

            obj = CollisionObject()
            obj.id = grocery_collision_box_name[1] + '_' + str(i)
            obj.operation = obj.REMOVE

            planning_scene.world.collision_objects.append(obj);

            obj = CollisionObject()
            obj.id = grocery_collision_box_name[2] + '_' + str(i)
            obj.operation = obj.REMOVE

            planning_scene.world.collision_objects.append(obj);

            obj = CollisionObject()
            obj.id = grocery_collision_box_name[3] + '_' + str(i)
            obj.operation = obj.REMOVE

            planning_scene.world.collision_objects.append(obj)

        for i in range(len(bboxes)):
            self._box_name = bboxes.keys()[i]

            obj = CollisionObject()
            obj.id = self._box_name
            obj.operation = obj.REMOVE

            planning_scene.world.collision_objects.append(obj)

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
        link_name='link_1', relative_object_name='fetch', relative_link_name='wrist_roll_link', grocery_bbox_names_key = 'grocery_bbox_names'):
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
        self._grocery_bbox_names_key = grocery_bbox_names_key
        self._grocery_bbox_names = None

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
        self._grocery_bbox_names = py_trees.blackboard.Blackboard().get(self._grocery_bbox_names_key)
        grocery_bbox = self._grocery_bboxes[self._grocery_box_index]
        if self._grocery_box_index > 0:
            grocery_box_name = 'grocery_box' + '_' + str(self._grocery_box_index - 1)
        elif self._grocery_box_index == 0:
            grocery_box_name = 'grocery_box'

        grocery_box_name = self._grocery_bbox_names[self._grocery_box_index]

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
            if 'test' in self._model_state.name[i] and self._model_state.pose[i].position.x > x_min\
            and self._model_state.pose[i].position.x < x_max and self._model_state.pose[i].position.y > y_min and self._model_state.pose[i].position.y < y_max:
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
        link_name='link_1', relative_object_name='fetch', relative_link_name='wrist_roll_link', grocery_bbox_names_key='grocery_bbox_names'):
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
        self._grocery_bbox_names_key = grocery_bbox_names_key
        self._grocery_bbox_names = None

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
        self._grocery_bbox_names = py_trees.blackboard.Blackboard().get(self._grocery_bbox_names_key)
        grocery_bbox = self._grocery_bboxes[self._grocery_box_index]
        if self._grocery_box_index > 0:
            grocery_box_name = 'grocery_box' + '_' + str(self._grocery_box_index - 1)
        elif self._grocery_box_index == 0:
            grocery_box_name = 'grocery_box'

        grocery_box_name = self._grocery_bbox_names[self._grocery_box_index]

        req = AttachRequest()
        req.model_name_1 = grocery_box_name
        req.link_name_1 = self._link_name
        req.model_name_2 = self._relative_object_name
        req.link_name_2 = self._relative_link_name

        self._link_attacher_client.call(req)

        x_min = grocery_bbox.center.position.x - grocery_bbox.size.x / 2
        x_max = grocery_bbox.center.position.x + grocery_bbox.size.x / 2
        y_min = grocery_bbox.center.position.y - grocery_bbox.size.y / 2
        y_max = grocery_bbox.center.position.y + grocery_bbox.size.y / 2

        for i in range(len(self._model_state.name)):
            if 'test' in self._model_state.name[i] and self._model_state.pose[i].position.x > x_min and self._model_state.pose[i].position.x < x_max\
            and self._model_state.pose[i].position.y > y_min and self._model_state.pose[i].position.y < y_max:
                req_object = AttachRequest()
                req_object.model_name_1 = self._model_state.name[i]
                req_object.link_name_1 = self._link_name
                req_object.model_name_2 = grocery_box_name
                req_object.link_name_2 = 'link_1'
                self._link_attacher_client.call(req_object)
                time.sleep(0.05)

        return py_trees.Status.SUCCESS

class SpawnRandomModelAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, model_state_key='model_state', spawn_range_y=1, range_use=True):
        super(SpawnRandomModelAct, self).__init__(name)
        self._model_state_key = model_state_key
        self._model_state = None
        self._model_state_pose = None
        self._spawn_model_client = None
        self._conveyor_belt_size_key = 'conveyor_belt_size'
        self._conveyor_belt_pose = None
        self._object_name_compensate_num = 6
        self._spawn_range_y = spawn_range_y
        self._range_use = range_use

    def setup(self, timeout):
        rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=timeout)
        self._spawn_model_client = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

        return True

    def initialise(self):
        self._model_state = deepcopy(py_trees.blackboard.Blackboard().get(self._model_state_key))
        self._conveyor_belt_pose = self._model_state.pose[self._model_state.name.index('conveyor_belt')]
        self._model_state_pose = deepcopy(self._model_state.pose)
        self._model_state_name = deepcopy(self._model_state.name)

    def update(self):
        conveyor_belt_size = deepcopy(py_trees.blackboard.Blackboard().get(self._conveyor_belt_size_key))
        All_object_name = ['cracker', 'gelatin', 'meat', 'mustard', 'soup', 'sugar']

        object_num = {'cracker': 0, 'gelatin': 0,'meat': 0,'mustard': 0,'soup': 0, 'sugar': 0}
        for name in self._model_state_name:
            if 'test_' in name:
                self._object_name_compensate_num = 6
            elif 'test' in name:
                self._object_name_compensate_num = 5
            else:
                self._object_name_compensate_num = 0
            for object_name in All_object_name:
                if object_name in name:
                    if object_name + '_test' == name or object_name == name:
                        if object_num[object_name] < 1:
                            object_num[object_name] = 1
                    elif int(name[len(object_name) + self._object_name_compensate_num:]) + 2 >= object_num[object_name]:
                        object_num[object_name] = int(name[len(object_name) + self._object_name_compensate_num:]) + 2
        All_object_file = []
        All_object_file.append(srp_md_path + '/model/grocery_models/cracker_test/model.sdf')
        All_object_file.append(srp_md_path + '/model/grocery_models/gelatin_test/model.sdf')
        All_object_file.append(srp_md_path + '/model/grocery_models/meat_test/model.sdf')
        All_object_file.append(srp_md_path + '/model/grocery_models/mustard_test/model.sdf')
        All_object_file.append(srp_md_path + '/model/grocery_models/soup_test/model.sdf')
        All_object_file.append(srp_md_path + '/model/grocery_models/sugar_test/model.sdf')
        dis_to_edge_x = {'cracker': 0.152, 'gelatin': 0.09,'meat': 0.092,'mustard': 0.08,'soup': 0.034, 'sugar': 0.097}
        dis_to_edge_y = {'cracker': 0.207, 'gelatin': 0.097,'meat': 0.05,'mustard': 0.06,'soup': 0.034, 'sugar': 0.174}
        object_radius = {'cracker': 0.2644 / 2, 'gelatin': 0.135 - 0.013, 'meat': 0.108 + 0.15, 'mustard': 0.2147 / 2 + 0.005,'soup': 0.113 + 0.02, 'sugar': 0.204 / 2}
        conveyor_belt_position = self._conveyor_belt_pose.position
        for loop_times in range(20000):
            object_index = random.randint(-1,5)
            if object_index == -1:
                object_index = 0
            # if object_index == 6:
            #     object_index = 5
            # object_index = 5
            x_rand = random.uniform(conveyor_belt_position.x - conveyor_belt_size.x / 2 + dis_to_edge_x[All_object_name[object_index]], conveyor_belt_position.x + conveyor_belt_size.x / 2 - dis_to_edge_x[All_object_name[object_index]] / 2)
            y_rand = random.uniform(conveyor_belt_position.y - conveyor_belt_size.y / 2 + dis_to_edge_y[All_object_name[object_index]], conveyor_belt_position.y + conveyor_belt_size.y / 2 - dis_to_edge_y[All_object_name[object_index]] / 2)
            if self._range_use:
                y_rand = random.uniform(conveyor_belt_position.y - conveyor_belt_size.y / 2 + dis_to_edge_y[All_object_name[object_index]], conveyor_belt_position.y - conveyor_belt_size.y / 2 + dis_to_edge_y[All_object_name[object_index]] / 2 + self._spawn_range_y)

            dis_min_to_objects = 100
            for i in range(len(self._model_state_pose)):
                All_object_name_index = -1
                for j in range(len(All_object_name)):
                    if All_object_name[j] in self._model_state_name[i]:
                        All_object_name_index = j
                if All_object_name_index == -1:
                    continue
                dis_to_objects = np.sqrt((x_rand - self._model_state_pose[i].position.x) * (x_rand - self._model_state_pose[i].position.x) + (y_rand - self._model_state_pose[i].position.y) * (y_rand - self._model_state_pose[i].position.y))\
                - object_radius[All_object_name[All_object_name_index]] - object_radius[All_object_name[object_index]]
                if dis_to_objects < dis_min_to_objects:
                    dis_min_to_objects = dis_to_objects

            if dis_min_to_objects <= 0:
                continue
            req = SpawnModelRequest()
            if object_num[All_object_name[object_index]] == 0:
                req.model_name = All_object_name[object_index] + '_test'
            elif object_num[All_object_name[object_index]] > 0:
                req.model_name = All_object_name[object_index] + '_test_' + str(object_num[All_object_name[object_index]] - 1)
            f = open(All_object_file[object_index],'r')
            req.model_xml = f.read()
            req.initial_pose.orientation.w = 1
            req.initial_pose.position.x = x_rand
            req.initial_pose.position.y = y_rand
            req.initial_pose.position.z = 0.65
            req.reference_frame = 'world'
            if 'cracker' in All_object_name[object_index] or 'sugar' in All_object_name[object_index]:
                req.initial_pose.orientation = Quaternion(0.5, -0.5, 0.5, 0.5)
                req.initial_pose.position.y = req.initial_pose.position.y + dis_to_edge_y[All_object_name[object_index]] / 2
            self._spawn_model_client.call(req)
            if 'cracker' in All_object_name[object_index] or 'sugar' in All_object_name[object_index]:
                req.initial_pose.position.y = req.initial_pose.position.y - dis_to_edge_y[All_object_name[object_index]] / 2
            self._model_state_pose.append(deepcopy(req.initial_pose))
            self._model_state_name.append(deepcopy(req.model_name))
            object_num[All_object_name[object_index]] = object_num[All_object_name[object_index]] + 1

        return py_trees.Status.SUCCESS

class DeleteModelAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, model_state_key='model_state'):
        super(DeleteModelAct, self).__init__(name)
        self._model_state_key = model_state_key
        self._model_state = None
        self._delete_model_client = None

    def setup(self, timeout):
        rospy.wait_for_service('/gazebo/delete_model', timeout=timeout)
        self._delete_model_client = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        return True

    def initialise(self):
        self._model_state = deepcopy(py_trees.blackboard.Blackboard().get(self._model_state_key))
        self._model_state_name = deepcopy(self._model_state.name)

    def update(self):
        All_object_name = ['cracker', 'gelatin', 'meat', 'mustard', 'soup', 'sugar']
        for name in self._model_state_name:
            req = DeleteModelRequest()
            if_delete = False
            for j in range(len(All_object_name)):
                if All_object_name[j] in name:
                    if_delete = True
                    break
            if not if_delete:
                continue
            req.model_name = name
            self._delete_model_client.call(req)

        return py_trees.Status.SUCCESS


class SpawnStaticModelAct(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(SpawnStaticModelAct, self).__init__(name)
        # self._pub = None
        self._spawn_model_client = None
        # self._allow = allow
        # self._object_name = object_name



    def setup(self, timeout):
        rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=timeout)
        self._spawn_model_client = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

        return True

    def initialise(self):
        pass

    def update(self):

        req = SpawnModelRequest()
        req.model_name = 'table'
        f = open(srp_md_path + '/model/grocery_models/table_1/model.sdf','r')
        req.model_xml = f.read()
        req.initial_pose.orientation.w = 1
        req.initial_pose.position.x = 1
        req.reference_frame = 'world'

        self._spawn_model_client.call(req)

        req = SpawnModelRequest()
        req.model_name = 'lower_table'
        f = open(srp_md_path + '/model/grocery_models/table_2/model.sdf','r')
        req.model_xml = f.read()
        req.initial_pose.orientation.w = 1
        req.initial_pose.position.y = 1
        req.reference_frame = 'world'

        self._spawn_model_client.call(req)

        req = SpawnModelRequest()
        req.model_name = 'conveyor_belt'
        f = open(srp_md_path + '/model/grocery_models/conveyor_belt/model.sdf','r')
        req.model_xml = f.read()
        req.initial_pose.orientation.w = 1
        req.initial_pose.position.x = 1
        req.initial_pose.position.y = -3.25
        req.reference_frame = 'world'

        self._spawn_model_client.call(req)

        req = SpawnModelRequest()
        req.model_name = 'wall'
        f = open(srp_md_path + '/model/grocery_models/wall/model.sdf','r')
        req.model_xml = f.read()
        req.initial_pose.orientation.w = 1
        req.initial_pose.position.x = 0
        req.initial_pose.position.y = 0
        req.reference_frame = 'world'

        self._spawn_model_client.call(req)

        # req = SpawnModelRequest()
        # req.model_name = 'cracker_test_1'
        # f = open(srp_md_path + '/model/grocery_models/cracker_test/model.sdf','r')
        # req.model_xml = f.read()
        # req.initial_pose.orientation = Quaternion(0.5, -0.5, 0.5, 0.5)
        # # req.initial_pose.orientation.w = 1
        # req.initial_pose.position.x = 2.5
        # req.initial_pose.position.y = 0
        # req.initial_pose.position.z = 0.5
        # req.reference_frame = 'world'

        # self._spawn_model_client.call(req)

        return py_trees.Status.SUCCESS

class FetchMoveAct(py_trees_ros.actions.ActionClient):
    def __init__(self, name, position, orientation, frame_id, *argv, **kwargs):
        super(FetchMoveAct, self).__init__(
            name=name,
            action_spec=move_base_msgs.MoveBaseAction,
            action_goal=move_base_msgs.MoveBaseGoal(),
            action_namespace='/move_base',
            *argv,
            **kwargs
        )
        self._frame_id = frame_id
        self.action_goal.target_pose.header.frame_id = self._frame_id
        self._position = position
        self.action_goal.target_pose.pose.position = self._position
        self._orientation = orientation
        self.action_goal.target_pose.pose.orientation = self._orientation

    def initialise(self):
        super(FetchMoveAct, self).initialise()
        # Get goal from blackboard
        blackboard = py_trees.blackboard.Blackboard()
        # test_move_base = move_base_msgs.MoveBaseGoal()
        self.action_goal.target_pose.header.stamp=rospy.Time.now()
        self.action_goal.target_pose.header.seq = 1
        # self.action_goal.target_pose.header.frame_id = "map"
        # self.action_goal.target_pose.pose.orientation.w = 1
        # self.action_goal.target_pose.pose.position.y = -3
        # self.action_goal.target_pose.pose.position.x = 3

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
            # py_trees.blackboard.Blackboard().set(self._out_pc_key, result.out_pc)
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = self.override_feedback_message_on_running
            return py_trees.Status.RUNNING

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
        rospy.wait_for_service('/link_attacher_node/attach', timeout=timeout)
        self._link_attacher_client = rospy.ServiceProxy("/link_attacher_node/attach", Attach)

        return True

    def initialise(self):
        pass

    def update(self):
        object_move_name = None
        object_move_name = self._object_name[0:-1] + 'test_' + str(int(self._object_name[-1]) - 1)
        if self._object_name[-1] == '0':
            object_move_name = object_move_name[0:-3]
        req = AttachRequest()
        req.model_name_1 = object_move_name
        req.link_name_1 = self._link_name
        req.model_name_2 = self._relative_object_name
        req.link_name_2 = self._relative_link_name
        self._link_attacher_client.call(req)
        return py_trees.Status.SUCCESS

class LinkDettachingAct(py_trees.behaviour.Behaviour):
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
        rospy.wait_for_service('/link_attacher_node/detach', timeout=timeout)
        self._link_dettacher_client = rospy.ServiceProxy("/link_attacher_node/detach", Attach)

        return True

    def initialise(self):
        pass

    def update(self):
        object_move_name = None
        object_move_name = self._object_name[0:-1] + 'test_' + str(int(self._object_name[-1]) - 1)
        if self._object_name[-1] == '0':
            object_move_name = object_move_name[0:-3]
        req = AttachRequest()
        req.model_name_1 = object_move_name
        req.link_name_1 = self._link_name
        req.model_name_2 = self._relative_object_name
        req.link_name_2 = self._relative_link_name
        self._link_dettacher_client.call(req)
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
        # AddAllObjectCollisionBoxAct('act_add_all_object_collision_box'),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        ChooseGroceryBoxPush('ChooseGroceryBoxPush'),

        FetchMoveAct('FetchMoveAct', position=Point(-1,-1,0), orientation=Quaternion(0, 0, 0, 1), frame_id='map'),
        FetchMoveAct('FetchMoveAct', position=Point(0.15,0,0), orientation=Quaternion(0, 0, 0, 1), frame_id='map'),
        py_trees_ros.subscribers.ToBlackboard(
            name='act_get_groundtruth',
            topic_name='/gazebo/model_states',
            topic_type=ModelStates,
            blackboard_variables={'model_state': None},
            clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE),
        GetFakeDopeSnapshotAct('act_get_dope_snapshot'),
        AddAllObjectCollisionBoxAct('act_add_all_object_collision_box'),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),

        PushBoxPoseGeneration('PushPoseGeneration'),
        MoveToRelativePoseAct('act_{}_move_pre_grasp'.format(name), pre_grasp),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        # OffsetPoses('act_offset_place_poses',
        #             offset=[-(gripper_length + pre_grasp_offset), 0, 0],
        #             in_poses_key='push_pose',
        #             out_poses_key='offset_place_pose', debug=True),
        OffsetPoses('act_offset_place_poses',
                    offset=[-gripper_length, -(pre_grasp_offset), 0],
                    in_poses_key='push_pose',
                    out_poses_key='offset_place_pose', debug=True),
        MoveToFirstPoseAct('act_{}_pick_grasploc'.format(name), poses_key='offset_place_pose'),
        # MoveToFirstPoseAct('act_{}_pick_grasploc'.format(name), poses_key='push_pose'),
        SleepBehavior('act_sleep_a_smidge', duration=1.5),
        # RelativeCartesianMoveAct('act_move_to_grasp_pose', pose_diff_msg=to_grasp_full_tf),
        RelativeCartesianMoveAct('act_move_to_grasp_pose', pose_diff_msg=to_y_grasp_full_tf),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        GroceryLinkAttachingAct('GroceryLinkAttachingAct'),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        SetAllowGripperCollisionAct('act_ignore_gripper_collision', allow=True),
        # RelativeCartesianMoveAct('act_move_to_grasp_pose', pose_diff_msg=to_push_tf),
        RelativeCartesianMoveBlackboardAct('RelativeCartesianMoveBlackboardAct'),
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
        RelativeCartesianMoveAct('act_move_to_grasp_pose', pose_diff_msg=to_y_grasp_full_back_tf),
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
        RemoveAllCollisionBoxAct('RemoveAllCollisionBoxAct'),
        FetchMoveAct('FetchMoveAct', position=Point(-1,-1,0), orientation=Quaternion(0, 0, 0, 1), frame_id='map'),
        FetchMoveAct('FetchMoveAct', position=Point(0,0.4,0), orientation=Quaternion(0, 0, 0.707, 0.707), frame_id='map'),
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
        ChooseGroceryBoxGrab('ChooseGroceryBoxGrab'),
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
        CloseGripperAct('act_close_gripper'),
        RelativeCartesianMoveAct('act_move_to_grasp_pose', pose_diff_msg=to_grab_box_back_tf),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        RelativeCartesianMoveAct('act_move_to_grasp_pose', pose_diff_msg=to_move_away_tf_1),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        RelativeCartesianMoveAct('act_move_to_grasp_pose', pose_diff_msg=to_move_away_tf_2),
        RemoveAllCollisionBoxAct('RemoveAllCollisionBoxAct'),
        FetchMoveAct('FetchMoveAct', position=Point(-0.5,-1,0), orientation=Quaternion(0, 0, 0, 1), frame_id='map'),
        FetchMoveAct('FetchMoveAct', position=Point(0.25,-0.75,0), orientation=Quaternion(0, 0, 0, 1), frame_id='map'),


        py_trees_ros.subscribers.ToBlackboard(
            name='act_get_groundtruth',
            topic_name='/gazebo/model_states',
            topic_type=ModelStates,
            blackboard_variables={'model_state': None},
            clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        GetFakeDopeSnapshotAct('act_get_dope_snapshot'),
        GrabBoxPlacePoseGeneration('GrabBoxPlacePoseGeneration'),
        OffsetPoses('act_offset_place_poses',
                    offset=[-(gripper_length + pre_grasp_offset), 0, 0],
                    in_poses_key='grab_pose',
                    out_poses_key='offset_place_pose', debug=True),
        # MoveToRelativePoseAct('act_{}_move_pre_grasp'.format(name), pre_grasp),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        MoveToFirstPoseAct('act_{}_pick_grasploc'.format(name), poses_key='offset_place_pose'),
        SleepBehavior('act_sleep_a_smidge', duration=1),
        RelativeCartesianMoveAct('act_move_to_grasp_pose', pose_diff_msg=to_grasp_tf),
        # RelativeCartesianMoveAct('act_move_to_grasp_pose', pose_diff_msg=to_grab_box_tf),
        #

        # SleepBehavior('act_sleep_a_smidge', duration=0.5),
        # RelativeCartesianMoveAct('act_move_to_grasp_pose', pose_diff_msg=to_grab_box_tf_append),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        OpenGripperAct('OpenGripperAct'),
        GroceryLinkDettachingAct('GroceryLinkDettachingAct'),
        # SleepBehavior('act_sleep_a_smidge', duration=0.5),
        # RelativeCartesianMoveAct('act_move_to_grasp_pose', pose_diff_msg=to_grab_box_back_tf),
        SetAllowGripperCollisionAct('act_ignore_gripper_collision', allow=False),
        # SleepBehavior('act_sleep_a_smidge', duration=0.5),
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
        RemoveAllCollisionBoxAct('RemoveAllCollisionBoxAct'),
    ])
    return root


def PickAct(name, obj):
    """
    Composite pick action for an object
    """
    # Initialize the root as sequence node
    root = py_trees.composites.Sequence(
        name='{}_pick_{}'.format(name, obj),
        children=None)

    # Add steps to execute pick action

    object_index = obj

    grasploc_grasp_fall = py_trees.composites.Selector('grasploc_grasp_fall')
    grasploc_grasp_seq = py_trees.composites.Sequence('grasploc_grasp_seq')
    grasploc_grasp_seq.add_children([
        SetAllowGripperCollisionAct('act_ignore_gripper_collision', allow=True),
        SleepBehavior('act_sleep_a_smidge', duration=1.5),
        RelativeCartesianMoveAct('act_move_to_grasp_pose', pose_diff_msg=to_grasp_tf),
        LinkAttachingAct('LinkAttachAct', object_name=object_index, link_name='link_1'),
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
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        AttachObjectAct('act_attach_object', object_name='inhand_collision_object', to_attach=True),
        RelativeCartesianMoveAct('act_move_up', pose_diff_msg=up_tf),
        SleepBehavior('act_sleep_a_smidge', duration=0.5),
        SetAllowGripperCollisionAct('act_ignore_gripper_collision', allow=False)
    ])

    grasploc_grasp_fall.add_children([
        grasploc_grasp_seq,
        py_trees.meta.success_is_failure(SetAllowGripperCollisionAct)(name='act_ignore_gripper_collision', allow=False)
    ])

    root.add_children([
        SetValueToBlackBoardAct('SetValueToBlackBoardAct'),
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
        TFPCAct('act_tf_pc', 'in_pc', 'base_link', 'tfed_pc'),
        CropPCAct('act_test_crop_pc', 'tfed_pc', 'crop_box', 'croped_pc'),
        PCPubAct('act_pub_croped_pc', 'croped_pc', 'croped_pc'),

        AttachObjectAct('act_dettach_object', object_name='inhand_collision_object', to_attach=False),

        # GetDopeSnapshotAct('act_get_dope_snapshot', object_name=object_index),
        GetFakeDopeSnapshotAct('act_get_dope_snapshot'),
        RemoveCollisionBoxAct('act_remove_inhand_collision_object', box_name='inhand_collision_object'),
        # GetTableAct('act_get_table'),


        ChooseGrasplocObjAct('act_choose_grasploc_obj', object_index=object_index),
        AddAllObjectCollisionBoxAct('act_add_all_object_collision_box'),
        GraspPoseGeneration('generate_gripper_pose'),
        FilterGrasplocPoints('act_filt_grasploc'),
        OffsetPoses('act_offset_grasploc_poses',
                    offset=[-(gripper_length + pre_grasp_offset), 0, 0],
                    in_poses_key='filtered_grasploc',
                    out_poses_key='offset_grasploc', debug=True), # The gripper origin is in the wrist so move out along x
        # GetTableAct('act_get_table'),
        MoveToStartAct('act_grasploc_pick', 'offset_grasploc'),
        grasploc_grasp_fall,
        # PCPubAct('act_pub_croped_pc', 'depth_downsampled', 'depth_downsampled'),

    ])
    return root


def PlaceAct(name, obj, surface, relation='None'):
    """
    Composite place action for an object
    """

    # Initialize the root as sequence node
    root = py_trees.composites.Sequence(
        name='{}_place_{}_from_{}'.format(name, obj, surface, relation),
        children=None)
    object_index = obj
    relative_object_index = surface

    place_act_fall = py_trees.composites.Selector('place_act_fall')
    place_act_seq = py_trees.composites.Sequence('place_act_seq')
    place_act_seq.add_children([
        SetPlanValueAct('SetPlanValueAct', relation=relation, relative_object_index=relative_object_index),
        # PCPubAct('act_pub_croped_pc', 'depth_downsampled', 'depth_downsampled'),
        FreeSpaceFinderAct('act_find_free_space', object_index=object_index),
        PCPubAct('act_pub_croped_pc', 'depth_downsampled', 'depth_downsampled'),
        GetStackPoseAct('act_get_stack_pose'),
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
        # self._srp._plans[0] = []
        # plan_temp = []
        # plan_temp.append('pick_from_surface')
        # plan_temp.append('meat_0')
        # self._srp._plans[0].append(deepcopy(plan_temp))
        # plan_temp = []
        # plan_temp.append('place_on_surf_proximity_to')
        # plan_temp.append('meat_0')
        # plan_temp.append('cracker_1')
        self._srp._plans[0].append(deepcopy(plan_temp))
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
        self._plan_root = py_trees.composites.Sequence('seq_{}_plan_root'.format(self.name))
        for action in plan:
            if action[0] == 'pick_from_surface':
                obj = action[1]
                self._plan_root.add_child(PickAct('act_pick_from_surface_{}'.format(obj), obj=obj))
            elif action[0] == 'place_on_surface':
                obj = action[1]
                self._plan_root.add_child(PlaceAct('act_place_on_surface_{}'.format(obj), obj=obj, surface='table'))
            elif action[0] == 'place_on_stack':
                obj = action[1]
                to_obj = action[2]
                self._plan_root.add_child(PlaceAct('act_place_on_stack_{}_{}'.format(obj, to_obj), obj=obj, surface=to_obj, relation='Stacking'))
            elif action[0] == 'pick_from_stack':
                obj = action[1]
                from_obj = action[2]
                self._plan_root.add_child(PickAct('act_pick_from_stack_{}_{}'.format(obj, from_obj), obj=obj))
            elif action[0] == 'place_on_surf_proximity_to':
                obj = action[1]
                to_obj = action[2]
                self._plan_root.add_child(PlaceAct('act_place_on_stack_{}_{}'.format(obj, to_obj), obj=obj, surface=to_obj, relation='proximity'))
            elif action[0] == 'place_on_obj_proximity_to':
                obj = action[1]
                other_obj = action[2]
                bot_obj = action[3]
                self._plan_root.add_child(py_trees.behaviours.Failure(
                    'act_place_on_obj_proximity_to_{}_{}_{}'.format(obj, other_obj, bot_obj)))
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
    root = py_trees.composites.Sequence('seq_{}_root'.format(name))
    dummy = py_trees.composites.Sequence('seq_{}_dummy_node'.format(name))
    root.add_children([
        _ExecutePlanAct(name, tree, dummy.id),
        dummy
    ])
    return root
