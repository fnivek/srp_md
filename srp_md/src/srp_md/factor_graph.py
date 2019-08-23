from __future__ import absolute_import
from builtins import range
import itertools

# srp_md imports
from srp_md.msg import Factor as RosFactor
from srp_md.msg import ObjectPair


class FactorGraph(object):
    def __init__(self, variables=None, factors=None):
        if variables is None:
            variables = list()
        if factors is None:
            factors = list()
        self._vars = variables
        self._factors = factors

    def num_vars(self):
        return len(self._vars)

    def num_factors(self):
        return len(self._factors)

    def add_factor(self, factor):
        self._factors.append(factor)

    def add_var(self, var):
        self._vars.append(var)

    def get_objects(self):
        return [var for var in self._vars if var.type == "object"]

    def get_relations(self):
        return [var for var in self._vars if var.type == "relation"]

    def gen_input_factors(self, configs=[]):
        for config in configs:
            for obj_combo in itertools.combinations(self.get_objects(), config[0]):
                for rel_combo in itertools.combinations(self.get_relations(), config[1]):
                    yield Factor(variables=obj_combo + rel_combo)


class Factor:
    def __init__(self, variables=None, probs=None):
        if variables is None:
            self.vars = []
        else:
            self.vars = variables
        self.probs = probs

    def connect_var(self, var):
        self.vars.append(var)

    def to_ros_factor(self):
        ros_factor = RosFactor()
        ros_factor.objects = [var.name for var in self.vars if var.type == 'object']
        for var in [var for var in self.vars if var.type == 'relation']:
            pair = ObjectPair()
            pair.object1 = var.properties['object1'].name
            pair.object2 = var.properties['object2'].name
            ros_factor.pairs.append(pair)
        ros_factor.probs = self.probs
        return ros_factor


class Var:
    def __init__(self, name, var_type='object', factors=None, value=None, properties=None, num_states=1):
        self.name = name
        self.value = value
        self.factors = factors
        if factors is None:
            self.factors = []
        self.properties = properties
        if properties is None:
            self.properties = {}
        self.type = var_type
        self.num_states = num_states

    def connect_factor(self, factor):
        self.factors.append(factor)

    def return_objects(self):
        if self.type == "relation":
            return [int(self.name[self.name.find('_', 1) + 1:self.name.find('_', 2)]),
                    int(self.name[self.name.find('_', 2) + 1:])]
        else:
            raise TypeError("This method should only be used with relation variables")
