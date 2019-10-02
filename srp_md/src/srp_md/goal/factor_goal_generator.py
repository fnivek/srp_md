# Python
from __future__ import absolute_import
import logging
import sys
from random import choice
import itertools
import copy

# Ros
import rospy

# Project
from . import goal_generator
from srp_md.srv import GetGoal, GetGoalRequest
from srp_md.msg import Factor
import srp_md


class FactorGraphGoalGenerator(goal_generator.BaseGoalGenerator):
    def __init__(self):
        super(FactorGraphGoalGenerator, self).__init__()
        self._allowed_config_keys.extend(['goal_client', 'use_consistency', 'use_commonsense'])
        self._logger = logging.getLogger(__name__)
        self._goal_client = None
        self._goal_client_name = '/get_goal'
        self._goal_client_changed = True
        self.use_consistency = True
        self.use_commonsense = False

    @property
    def goal_client(self):
        return self._goal_client_name

    @goal_client.setter
    def goal_client(self, name):
        self._goal_client_name = name
        self._goal_client_changed = True

    def connect_goal_client(self):
        # This can raise a rospy.ServiceException
        rospy.wait_for_service(self._goal_client_name, timeout=1)
        self._goal_client = rospy.ServiceProxy(self._goal_client_name, GetGoal)
        self._goal_client_changed = False

    def make_prior_knowledge_msg(self):
        prior_knowledge = []
        if self.use_consistency:
            prior_knowledge.append(GetGoalRequest.CONSISTENCY_PRIOR)
        if self.use_commonsense:
            prior_knowledge.append(GetGoalRequest.COMMON_SENSE_PRIOR)
        return prior_knowledge

    def generate_goal(self, factors, obs):
        self._logger.debug('Generating goal')
        self._logger.debug('Took factors %s', factors.keys())

        # Fill in the request
        req = GetGoalRequest()
        req.prior_knowledge = self.make_prior_knowledge_msg()
        req.objects = obs.get_obj_names()
        # There are no relations because there is only one object therfore return that object
        if len(req.objects) <= 1:
            self._logger.warn('Only one object in the scene graph, maybe we should just disallow this case')
            return None
        # TODO(?): We need to figure out how to represent this, we are not using common sense knowledge in libDAI so it
        #          should not matter for now
        req.classes = [
            choice([GetGoalRequest.CLASS_PROP, GetGoalRequest.CLASS_CONTAINER, GetGoalRequest.CLASS_SUPPORTER])
            for _ in req.objects
        ]
        # TODO(Kevin): If the objects always have one state then there is no need to have this (I leave it for now
        #              because it might be useful if we take noisy observations)
        # Objects will have one state because they have been observed and therfore are in one possible state
        req.num_states = [obj.num_states for obj in obs.objs]

        # Generate and fill in all factors
        #   For each type of factor generate all possible combinations
        #   If there are not enough vars for the factor the factor is skipped
        for factor_type, handler in factors.iteritems():
            ros_factor = None
            if len(obs.objs) < factor_type[0] or len(obs.relations) < factor_type[1]:
                self._logger.debug(
                    'Can not make a factor of type {} because there are {} objects and {} relations'.format(
                        factor_type, len(obs.objs), len(obs.relations)))
                continue
            # Generate all combinations of objects
            for objects in itertools.combinations(obs.objs, factor_type[0]):
                # Generate all possible relationship vars and assign a uuid
                pairs = []
                new_uuid = obs.get_new_uuid()
                for pair in itertools.combinations(objects, 2):
                    relation = srp_md.Relation(list(pair), uuid=new_uuid + len(pairs))
                    pairs.append(relation)
                # Use the LearnedFactor to generate a ros_factor for each combination
                ros_factor = handler.generate_factor(objects + tuple(pairs)).to_ros_factor()
                req.factors.append(ros_factor)

        self._logger.debug('Get goal request is:\n{}'.format(req))

        # Get the response
        resp = None
        try:
            if self._goal_client_changed:
                self.connect_goal_client()
            resp = self._goal_client(req)
        except rospy.ServiceException as e:
            self._logger.error('Failed when calling /get_goal service: {}'.format(e))
            raise

        self._logger.debug('/get_goal response:\n{}'.format(resp))

        # Turn the response into scene graph
        goal = copy.deepcopy(obs)
        for i in range(len(resp.relation)):
            id_list = [resp.object1[i][resp.object1[i].find('_') + 1:], resp.object2[i][resp.object2[i].find('_') + 1:]]
            id_list.sort()
            rel_name = 'R_' + id_list[0] + '_' + id_list[1]
            for relation in goal.relations:
                if relation.name == rel_name:
                    relation.value = resp.relation[i]
        self._logger.debug('What is resp? %s', resp)
        self._logger.debug('What are object names? %s', goal.get_obj_names())
        self._logger.debug('What are relation names? %s', goal.get_rel_names())
        self._logger.debug('What are relation values? %s', goal.get_rel_values())
        self._logger.debug('What are property values? %s', goal.get_prop_values('color'))
        self._logger.debug('Is this scene graph consistent? %s', goal.check_consistency("block"))
        return goal


# Register the goal generator
goal_generator.goal_generators['factor_graph_goal_generator'] = FactorGraphGoalGenerator
