# Python imports
from __future__ import absolute_import
import logging
import itertools

# SRP MD imports
from . import learn
import srp_md


class FactorGraphLearner(learn.BaseLearner):
    def __init__(self):
        self._logger = logging.getLogger(__name__)

    def learn(self, obs):
        """ Learn.

        Transforms a set of scene graphs into learned factors that can be used to build scene graphs. Each factor in the
        input scene graph can be maped to a particular type of factor e.g., a object unary potential. Duplicates in the
        graph may exist and these are treated as the same factor and learned as one. Additionally there may be duals,
        where a dual is a pair of factors that are related and one can be derived from the other. An example of a  dual
        is a factor between a relation and an object1 and a factor between relation and object 2 because if object 1 and
        2 are swaped, the relation just flips to its dual value (ON <--> SUPPORT). Duals are treated as one factor and
        learned together with a dual function to generate the dual factor.

        Input:
          obs - a set of scene graphs in the form TODO(Kevin)

        Returns:
          A dictionary of factor types to dictionaries of variable state to real [0, inf) i.e.:
            {'xx': {x1=A,x2=B: 10, ...}, 'x': {x1=A: 10, x2=B: 0.01}, 'rrr': {r1=ON, r2=ON, r3=DISJOINT: 3.8}}

        """
        self._logger.debug('Learn')

        for example in obs:
            self.learn_single_obs(example)

        return None

    def learn_single_obs(self, obs):
        # Determine which factors exist in observation by counting number of objects
        factors = []
        num_objs = 3  # TODO(Kevin)
        num_relations = int(srp_md.ncr(num_objs, 2))
        # No objects therfor no factors to learn
        if num_objs == 0:
            return
        factor_gen = itertools.product(range(num_objs + 1), range(num_relations + 1))
        next(factor_gen)  # Remove (0, 0) becuase there is no such factor
        for x, y in factor_gen:
            factors.append(Factor(x, y))

        # Increment seen assignments
        # TODO(Kevin): When objects or relations has duplicate objects they are counted as different objects. I need to
        #              think about the assignment of the variable vs the number of variables
        objs = ['a', 'b', 'c']  # TODO(Kevin): Get actuall objects
        relations = ['r{}'.format(i) for i in range(num_relations)]  # TODO(Kevin): Get actuall relations
        obj_combos = {i: list(itertools.combinations(objs, i)) for i in range(1, num_objs + 1)}
        obj_combos[0] = []
        relation_combos = {i: list(itertools.combinations(relations, i)) for i in range(1, num_relations + 1)}
        relation_combos[0] = []
        for factor in factors:
            # Get all possible combinations of objects and relations from observations and increment that dictionary key
            keys = None
            if factor.num_objs == 0:
                keys = relation_combos[factor.num_relations]
            elif factor.num_relations == 0:
                keys = obj_combos[factor.num_objs]
            else:
                keys = []
                prod = itertools.product(obj_combos[factor.num_objs], relation_combos[factor.num_relations])
                for elem in prod:
                    keys.append(elem[0] + elem[1])
            for key in keys:
                print(key)
                if key in factor.map:
                    factor.map[key] += 1
                else:
                    factor.map[key] = 1
            print(factor)

        return factors


class Factor:
    def __init__(self, num_objs, num_relations):
        self.num_objs = num_objs
        self.num_relations = num_relations
        self.map = {}

    def __str__(self):
        return '({}, {})\n{}'.format(self.num_objs, self.num_relations, self.map)


class Var:
    pass


class Object:
    pass


class Relation:
    pass


# Register the learner
learn.learners['factor_graph_learner'] = FactorGraphLearner
