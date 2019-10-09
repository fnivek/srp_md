""" Actions.

Define the actions used in srp_md.

"""
import py_trees

from behavior_manager.interfaces.manipulation_behavior_new import (TuckWithCondBehavior, HeadMoveBehavior,
                                                                   FullyExtendTorso)


def Reset(name):
    """ Bring robot back to default pose.

    Lift the torso, tuck the arm and look strait.

    """
    root = py_trees.composites.Parallel(
        name='pal_{}'.format(name),
        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL,
        synchronize=True,
        allow_failure=False)
    root.add_children([
        FullyExtendTorso('act_{}_lift_torso'.format(name)),
        TuckWithCondBehavior('act_{}_tuck_arm'.format(name), tuck_pose='tuck'),
        HeadMoveBehavior('act_{}_look_strait'.format(name), 'MoveStraight')
    ])
    return root
