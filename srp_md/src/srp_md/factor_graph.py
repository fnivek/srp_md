from __future__ import absolute_import
from builtins import range
import itertools

# srp_md imports
import srp_md
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
        self._rev_rel_dict = {
            "disjoint": "disjoint",
            "proximity": "proximity",
            "on": "support",
            "support": "on",
            "in": "contain",
            "contain": "in"
        }

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

    def get_var_names(self):
        return [var.name for var in self._vars]

    def get_var_from_name(self, name):
        for var in self._vars:
            if var.name == name:
                return var
        else:
            return None

    def gen_input_factors(self, configs=[]):
        for config in configs:
            for obj_combo in itertools.combinations(self.get_objects(), config[0]):
                for rel_combo in itertools.combinations(self.get_relations(), config[1]):
                    yield Factor(variables=obj_combo + rel_combo)

    def gen_ordered_factors(self, configs=[]):
        # For each (obj, rel) config, do:
        for config in configs:
            # If the number of relations do not match with corresponding number of objects, just skip
            if config[1] != srp_md.tri_num(config[0] - 1):
                continue
            # For each combination of objects and relations, do:
            for obj_permu in itertools.permutations(self.get_objects(), config[0]):
                # Initialize variable list as the permutated list of objects
                var_list = list(obj_permu)
                # For each pair of objects in the list, do:
                for obj_pair in itertools.combinations(obj_permu, 2):
                    # Get the relation of those pair (in order)
                    rel_name = "R_" + str(obj_pair[0].return_id()) + "_" + str(obj_pair[1].return_id())
                    relation = self.get_var_from_name(rel_name)
                    # If such relation does not exist, then it's reverse should exist
                    if relation is None:
                        rev_name = "R_" + str(obj_pair[1].return_id()) + "_" + str(obj_pair[0].return_id())
                        rev_var = self.get_var_from_name(rev_name)
                        # If the reverse doesn't exist print error
                        if rev_var is None:
                            raise ValueError("Something is seriously wrong here!")
                        # Otherwise, make and add the relation into variable list
                        else:
                            rev_val = self._rev_rel_dict[rev_var.assignment['value']]
                            temp_var = Var(rel_name, var_type='relation', value=rev_val)
                            var_list.append(temp_var)
                    # Else if the relation exists, just add it to the list
                    else:
                        var_list.append(relation)
                # Finally return the factor of this variables list
                yield Factor(variables=var_list)


class Factor:
    """
    Factor.

    Represents a factor in a factor graph.

    WARNING: Vars must be ordered such that all objects are first and are followed by any relations
             additionally the probabilities are sorted such that states are as follows:
             [[0, 0, ..., 0], [1, 0, ..., 0], ..., [n1, 0, ..., 0], [0, 1, ..., 0], [1, 1, ..., 0], ...,
             [n1, 1, ..., 0], ..., [n1, n2, ..., nm]]
             i.e. index = sum[i = 0 -> n-1](state(vi) * prod[j = 0 -> i - 1](num_states(vi)))
             see https://staff.fnwi.uva.nl/j.m.mooij/libDAI/doc/namespacedai.html#a750c3807e7375265c3fb410a1f1223fe
             calcLinearState and calcState for more details.

    """
    def __init__(self, variables=None, probs=None):
        if variables is None:
            self.vars = []
        else:
            self.vars = variables
        self.probs = probs

    def connect_vars(self, vars):
        self.vars.extend(vars)

    def to_ros_factor(self):
        ros_factor = RosFactor()
        ros_factor.objects = [var.name for var in self.vars if var.type == 'object']
        for var in [var for var in self.vars if var.type == 'relation']:
            pair = ObjectPair()
            pair.object1 = var.object1
            pair.object2 = var.object2
            ros_factor.pairs.append(pair)
        ros_factor.probs = self.probs
        return ros_factor


class Var:
    def __init__(self, name, var_type='object', factors=None, value=None, assignment=None, num_states=1):
        self.name = name
        self.factors = factors
        if factors is None:
            self.factors = []
        self.assignment = assignment
        if assignment is None:
            self.assignment = {}
        self.assignment['value'] = value
        self.type = var_type
        self.num_states = num_states

    def connect_factors(self, factors):
        self.factors.extend(factors)

    def return_id(self):
        if self.type == "object":
            return int(self.name[self.name.find('_') + 1:])
        else:
            raise TypeError("This method should only be used with object variables")

    def return_objects(self):
        # Returns objects connected to single relation
        if self.type == "relation":
            return [int(self.name[self.name.find('_', 1) + 1:self.name.find('_', 2)]),
                    int(self.name[self.name.find('_', 2) + 1:])]
        else:
            raise TypeError("This method should only be used with relation variables")
