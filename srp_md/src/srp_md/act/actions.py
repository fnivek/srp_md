""" Actions.

Define the actions used in srp_md.

"""
from copy import deepcopy

import rospy
import actionlib
import py_trees
import py_trees_ros
from control_msgs.msg import GripperCommandGoal, GripperCommandAction
from srp_md_msgs.msg import *
from geometry_msgs.msg import Pose, Transform

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

class MoveToRelativePoseAct(py_trees_ros.actions.ActionClient):
    def __init__(self, name, transform=Transform(), *argv, **kwargs):
        super(MoveToRelativePoseAct, self).__init__(
            name=name,
            action_spec=MoveToRelativePoseAction,
            action_goal=MoveToRelativePoseGoal(),
            action_namespace='relative_move',
            *argv,
            **kwargs
        )
        self.action_goal.transform = transform

def PickAct(name):
    """
    Pick
    - Input: Detected objects' poses from DOPE
    - Children:
    -   GrasplocPickBehavior: From the bounding box, get the intersection point cloud and find graspable pose for grip using grasploc?
    -   GrasplocPickBehavior: Pick the best graspable pose by pruning undesired approach angles with dot product of z-axis?
    -   GrasplocPickBehavior: Move the gripper to the pose with trajectory planning (move somewhere to the top, go down to the offset and go to the object)
    -   GrasplocPickBehavior: Pick up the object by actuating gripper
    -   CartesianBehavior: Lift it up in world z (base-link z)
    """
    # Set up blackboard variable for GrasplocPickBehavior?
    obj_name, obj_bbox = py_trees.blackboard.Blackboard().get('obj_key')
    obj_pose = obj_bbox

    # Generate a pose that is higher up from obj_pose. Not sure if this needs to be the gripper pose?
    up_pose = deepcopy(obj_pose)
    up_pose.position.z += 0.5

    # Initialize the root as sequence node
    root = py_trees.composites.Sequence(
        name='seq_{}'.format(name),
        children=None)

    # Add steps to execute pick action
    root.add_children([
        GrasplocPickBehavior(name='grasplocPick', bbox=obj_bbox),
        CartesianBehavior(name='act_cartesian_pick', action_name='move', wplist=up_pose, max_try=3)
    ])
    return root

class GrasplocPickBehavior(py_trees_ros.actions.ActionClient):
    def __init__(self, name, input_key='grasploc', centroid_key='centroid_3d_point', filter_off=False, *argv, **kwargs):
        super(GrasplocPickBehavior, self).__init__(
            name=name,
            action_spec=fetch_manipulation_pipeline.msg.GrasplocPickAction,
            action_goal=fetch_manipulation_pipeline.msg.GrasplocPickGoal(),
            action_namespace='grasploc_pick',
            *argv,
            **kwargs
        )
        self._input_key = input_key
        self._filter_off = filter_off
        self._centroid_key = centroid_key
        loadPredefined("/Prepare_Grasploc_Pick")

    def initialise(self):
        super(GrasplocPickBehavior, self).initialise()
        # Get centroid and shelf id from blackboard
        blackboard = py_trees.blackboard.Blackboard()
        poseArray = blackboard.get(self._input_key)
        centroid = blackboard.get(self._centroid_key)

        if poseArray is None:
            rospy.logerr('PoseArray was not defined for pick behavior')
            self.action_goal = fetch_manipulation_pipeline.msg.GrasplocPickGoal()
            return
        self.action_goal.grasp_poses = poseArray.graspable_points.poses
        self.action_goal.normal = poseArray.normal
        self.action_goal.principal = poseArray.principal
        self.action_goal.filter_off = self._filter_off
        self.action_goal.centroid = centroid
        rospy.loginfo('Grasploc Pick Goal Constructed.')

def PlaceAct(obj_name, des_pose):
    """
    Place
    - Input: Desired pose of the object to be placed
    - Children:
    -   Go to that position from the top, using trajectory planning, move down in same orientation as it grasped the object
    -   ControlGripperBehavior: Actuate the gripper to let it go~
    -   CartesianBehvaior: Move side, up and out
    -   TuckWithCondBehavior: Move the arm to default position
    """
    # Generate a pose that is higher up from des_pose. Not sure if this needs to be the gripper pose?
    back_pose = deepcopy(des_pose)
    back_pose.position.x += 0.5 # probably sure this is not the way to do it. General moving out how to do it?

    up_pose = deepcopy(back_pose)
    up_pose.position.z += 0.5

    # Initialize the root as sequence node
    root = py_trees.composites.Sequence(
        name='place_{}'.format(obj_name),
        children=None)

    # Add steps to execute pick action
    root.add_children([
        LoadAndExecuteTrajectoryBehavior(obj_name, traj_name='PlaceItem'),
        ControlGripperBehavior(name, 1.0),
        CartesianBehavior(name='act_cartesian_place', action_name='move', wplist=[back_pose, up_pose], max_try=3),
        TuckWithCondBehavior(obj_name, 'unknown_3')
    ])
    return root

def GetDesiredPose(obj_poses, cur_obj, surface):
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
    # If desired surface is table, do:
    if surface == "table":
        table = Table_Detector()
        rand_points = Gen_Rand_Points_On_Table(table)
        point = Max_Min_Distance(obj_poses, rand_points)
    # If desired surface is on an object, do:
    else:
        des_pose = obj_poses[surface]
        des_pose.position.z += cur_obj.height / 2 + 0.5 # add buffer
    return des_pose

"""
Table_Detector

- Get point cloud from depth image and prune the points according to z-range
- Run ransac algorithm to find plane and prune the non-horizontal ones with dot product. Pick the lowest / largest plane
- Use min / max in x & y axis to find the extreme points (corners) of the point cloud
- Fit minimum all-inclusive polygon (likely rectangle) using the extreme points
"""
