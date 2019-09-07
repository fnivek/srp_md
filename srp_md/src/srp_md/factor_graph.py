from __future__ import absolute_import

# srp_md imports


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

    def get_var_names(self):
        return [var.name for var in self._vars]

    def get_var_from_name(self, name):
        for var in self._vars:
            if var.name == name:
                return var
        else:
            return None

    def get_vars(self):
        return self._vars

    def get_new_uuid(self):
        return max([var.uuid for var in self._vars]) + 1

    # This function uses scene graph's function variables... may need to change?
    # def gen_input_factors(self, configs=[]):
    #     for config in configs:
    #         for obj_combo in itertools.combinations(self.get_objects(), config[0]):
    #             for rel_combo in itertools.combinations(self.get_relations(), config[1]):
    #                 yield Factor(variables=obj_combo + rel_combo)


class Factor(object):
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

    def __eq__(self, other):
        if isinstance(other, Factor):
            return len(self.vars) == len(other.vars) and \
                   len(self.vars) == sum([1 for var in other.vars if var in self.vars])
        return NotImplemented

    def __ne__(self, other):
        x = self.__eq__(other)
        if x is not NotImplemented:
            return not x
        return NotImplemented

    def __hash__(self):
        return hash(tuple(self.vars))

    def connect_vars(self, vars):
        self.vars.extend(vars)


class Var(object):
    def __init__(self, name=None, uuid=0):
        self.name = name
        self.uuid = uuid

    def __eq__(self, other):
        if isinstance(other, Var):
            return self.uuid == other.uuid
        return NotImplemented

    def __ne__(self, other):
        x = self.__eq__(other)
        if x is not NotImplemented:
            return not x
        return NotImplemented

    def __hash__(self):
        return hash(self.uuid)

    def __str__(self):
        return '{}[{}]'.format(self.name, str(self.uuid))
