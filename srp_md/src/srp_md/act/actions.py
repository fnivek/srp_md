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

import numpy as np

from moveit_msgs.msg import PlanningScene, CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from control_msgs.msg import GripperCommandGoal, GripperCommandAction
from srp_md_msgs.msg import *
from dope_msgs.msg import DopeAction, DopeGoal
from geometry_msgs.msg import Pose, PoseStamped, Transform, PoseArray, TransformStamped, Vector3
from sensor_msgs.msg import CameraInfo, Image as ImageSensor_msg, PointCloud2
from vision_msgs.msg import BoundingBox3D
import actionlib_msgs.msg as actionlib_msgs
from grasploc_wrapper_msgs.msg import GrasplocAction, GrasplocGoal, GrasplocResult
from scipy.spatial.transform import Rotation as R
import srp_md

import fetch_manipulation_pipeline.msg
from behavior_manager.conditions.arm_tucked_condition import ArmTuckedCondition
from behavior_manager.interfaces.manipulation_behavior_new import (TuckWithCondBehavior, HeadMoveBehavior,
                                                                   FullyCollapseTorso, FullyExtendTorso)
from behavior_manager.interfaces.fetch_manipulation_behavior import *


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
    def __init__(self, name, poses_key, *argv, **kwargs):
        super(MoveToFirstPoseAct, self).__init__(
            name=name,
            action_spec=MoveToFirstPoseAction,
            action_goal=MoveToFirstPoseGoal(),
            action_namespace='move_to_first_pose',
            *argv,
            **kwargs
        )
        self._poses_key = poses_key

    def initialise(self):
        super(MoveToFirstPoseAct, self).initialise()
        self.action_goal.poses = py_trees.blackboard.Blackboard().get(self._poses_key)

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

class GetDopeSnapshotAct(py_trees_ros.actions.ActionClient):
    def __init__(self, name, *argv, **kwargs):
        super(GetDopeSnapshotAct, self).__init__(
            name=name,
            action_spec=DopeAction,
            action_goal=DopeGoal(),
            action_namespace='dope',
            *argv,
            **kwargs
        )

        self._timeout = 5
        self._listener = tf.TransformListener()

        self.image_sub = message_filters.Subscriber(
            '/head_camera/rgb/image_raw',
            ImageSensor_msg
        )
        self.info_sub = message_filters.Subscriber(
            '/head_camera/rgb/camera_info',
            CameraInfo
        )
        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.info_sub], 100)
        self.ts.registerCallback(self.image_callback)

    def image_callback(self, image, info):
        self.action_goal = DopeGoal()
        self.action_goal.image = image
        self.action_goal.cam_info = info

    def update(self):
        self.logger.debug("{0}.update()".format(self.__class__.__name__))
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
            obj_bboxes_prev = py_trees.blackboard.Blackboard().get('obj_bboxes')
            obj_bboxes_post = {}
            uuid = 0
            class_ids = rospy.get_param("/dope/class_ids")
            class_names = {class_id: name for name, class_id in class_ids.iteritems()}

            for detection in result.detections:
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = '/head_camera_rgb_optical_frame' # shouldn't this be same as self.imsage_sub?
                pose_stamped.header.stamp = rospy.Time(0)
                pose_stamped.pose = detection.bbox.center
                try:
                    tfed_pose_stamped = self._listener.transformPose('/base_link', pose_stamped)
                    detection.bbox.center = tfed_pose_stamped.pose
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
                    print 'Failed to transform from /head_camera_rgb_optical_frame to /base_link: {}'.format(e)
                    return None

                obj_bboxes_post[class_names[detection.results[0].id] + '_' + str(uuid)] = detection.bbox
                uuid += 1

            # print 'obj_bboxes_post: {}'.format(obj_bboxes_post)

            if obj_bboxes_prev is None:
                obj_bboxes = obj_bboxes_post
            else:
                obj_bboxes = obj_bboxes_prev
                for obj_prev_key in obj_bboxes_prev.keys():
                    closest_obj_name = None
                    closest_dist = 10000
                    for obj_post_key in obj_bboxes_post.keys():
                        dist = srp_md.pose_difference(obj_bboxes_prev[obj_prev_key].center, obj_bboxes_post[obj_post_key].center)
                        if dist < closest_dist:
                            closest_dist = dist
                            closest_obj_name = obj_post_key

                    # print 'Closest distance {}, with prev object {} and post object {}'.format(closest_dist, obj_prev_key, closest_obj_name)
                    if closest_dist <= 0.01:
                        obj_bboxes[obj_prev_key] = obj_bboxes_post[closest_obj_name]
                        del obj_bboxes_post[closest_obj_name]
                    else:
                        print 'No closest object found. Defaulting with previously seen object bbox value for {}'.format(obj_prev_key)

            py_trees.blackboard.Blackboard().set('obj_bboxes', obj_bboxes)
            print "obj bboxes: ", obj_bboxes

            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = self.override_feedback_message_on_running
            return py_trees.Status.RUNNING

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

    def initialise(self):
        super(GrasplocAct, self).initialise()

        # Get the point cloud
        blackboard = py_trees.blackboard.Blackboard()
        self._pc = blackboard.get(self._in_pc_key)
        self.action_goal.input_pc = self._pc

        # Location of head depth frame when head fully extended
        # TODO(...): Actually get the pose of the depth frame from tf
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
        MoveToFirstPoseAct('act_{}_pick_grasploc'.format(name), poses_key=poses_key)
    ])
    return root

class ChooseGrasplocObjAct(py_trees.behaviour.Behaviour):
    def __init__(self, name, bbox_key='obj_bboxes', crop_box_key='crop_box'):
        super(ChooseGrasplocObjAct, self).__init__(name)
        self._bbox_key = bbox_key
        self._crop_box_key = crop_box_key

    def update(self):
        blackboard = py_trees.blackboard.Blackboard()
        bboxes = blackboard.get(self._bbox_key)
        if bboxes is None or len(bboxes) == 0:
            return py_trees.Status.FAILURE

        # TODO(Kevin): Get which object to grab from the plan
        blackboard.set(self._crop_box_key, bboxes.values()[0])
        return py_trees.Status.SUCCESS

class FilterGrasplocPoints(py_trees.behaviour.Behaviour):
    def __init__(self, name, grasp_points_key='grasploc', filtered_grasp_points_key='filtered_grasploc'):
        super(FilterGrasplocPoints, self).__init__(name)
        self._grasp_points_key = grasp_points_key
        self._filtered_grasp_points_key = filtered_grasp_points_key
        # self._center_axis = np.array([-1, 0, 1]) / np.sqrt(2) # 45deg from vertical towards robot
        # self._min_cos_theta = np.cos(np.pi / 4) # cos(x) -> +- x rads
        self._center_axis = np.array([0, 0, 1]) # 0deg from vertical towards robot
        self._min_cos_theta = np.cos(np.pi / 8) # cos(x) -> +- x rads

    def update(self):
        blackboard = py_trees.blackboard.Blackboard()
        grasploc = blackboard.get(self._grasp_points_key)
        if grasploc is None:
            return py_trees.Status.FAILURE

        filtered_grasploc = GrasplocResult()
        filtered_grasploc.graspable_points.header = grasploc.graspable_points.header
        for i, ros_norm in enumerate(grasploc.normal):
            norm = np.array([ros_norm.x, ros_norm.y, ros_norm.z])
            norm = norm / np.linalg.norm(norm)
            if np.dot(norm, self._center_axis) > self._min_cos_theta:
                filtered_grasploc.graspable_points.poses.append(grasploc.graspable_points.poses[i])
                filtered_grasploc.normal.append(grasploc.normal[i])
                filtered_grasploc.principal.append(grasploc.principal[i])

        # TODO(Kevin): Get which object to grab from the plan
        blackboard.set(self._filtered_grasp_points_key, filtered_grasploc.graspable_points.poses)
        return py_trees.Status.SUCCESS

class GetTableAct(py_trees_ros.actions.ActionClient):
    def __init__(self, name, *argv, **kwargs):
        super(GetTableAct, self).__init__(
            name=name,
            action_spec=GetTableAction,
            action_goal=GetTableGoal(),
            action_namespace='get_table',
            *argv,
            **kwargs
        )

    def initialise(self):
        self.action_goal.points = py_trees.blackboard.Blackboard().get('depth_downsampled')
        cropped_pc_pub = rospy.Publisher('cropped_pc', PointCloud2, queue_size=10)
        cropped_pc_pub.publish(self.action_goal.points)

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

class FreeSpaceFinderAct(py_trees_ros.actions.ActionClient):
    def __init__(self, name, obj_dim=None, *argv, **kwargs):
        super(FreeSpaceFinderAct, self).__init__(
            name=name,
            action_spec=FreeSpaceFinderAction,
            action_goal=FreeSpaceFinderGoal(),
            action_namespace='free_space_finder',
            *argv,
            **kwargs
        )
        self._obj_dim = obj_dim

    def initialise(self):
        self.action_goal.points = py_trees.blackboard.Blackboard().get('depth_downsampled')
        if self._obj_dim is None:
            self.action_goal.obj_dim = Vector3()
        else:
             self.action_goal.obj_dim = self._obj_dim
        plane_bboxes = py_trees.blackboard.Blackboard().get('plane_bboxes')
        if len(plane_bboxes) == 0:
            self.action_goal.plane_bbox = BoundingBox3D()
        else:
            self.action_goal.plane_bbox = plane_bboxes[0]

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
            py_trees.blackboard.Blackboard().set('free_space_pose', result.pose)
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

def AddAllCollisionBoxesAct(name):
    """
    Add all collision boxes
    """
    # Initialize the root as sequence node
    root = py_trees.composites.Sequence(
        name='{}_add_all_collision_boxes'.format(name),
        children=None)

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

    # Add each collision boxes to the root node
    root.add_children([
        AddCollisionBoxAct('act_add_left_wall', box_name='left_wall', box_pose=left_wall_pose, box_size=[10, 0.1, 10]),
        AddCollisionBoxAct('act_add_right_wall', box_name='right_wall', box_pose=right_wall_pose, box_size=[10, 0.1, 10]),
        AddCollisionBoxAct('act_add_bot_wall', box_name='bot_wall', box_pose=bot_wall_pose, box_size=[0.5, 0.5, 0.01]),
        AddCollisionBoxAct('act_add_table', box_name='table', box_pose=table_pose, box_size=[0.8, 4, table_pose.position.z * 2]),
    ])

    return root

def MoveToStartAct(name):
    """
    Bring the robot to initial position for experiment
    """
    # Initialize the root as sequence node
    root = py_trees.composites.Sequence(
        name='{}_move_to_start'.format(name),
        children=None)
    root.add_children([
        FullyExtendTorso('act_{}_extend_torso'.format(name)),
        TuckWithCondBehavior('act_{}_tuck_arm'.format(name), tuck_pose='tuck'),
        # HeadMoveBehavior('act_{}_look_strait'.format(name), 'MoveStraight'),
        OpenGripperAct('act_{}_open_gripper'.format(name))
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
    root.add_children([
        GetDopeSnapshotAct('act_get_dope_snapshot_{}'.format(obj)),
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

    # Add steps to execute pick action
    # root.add_children([
    # ])
    return root

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
