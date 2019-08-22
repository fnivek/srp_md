from builtins import range
import itertools


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

    def gen_all_possible_factors(self):
        for i in range(1, self.num_vars() + 1):
            for combo in itertools.combinations(self._vars, i):
                yield Factor(variables=combo)

    def gen_input_factors(self, configs=[]):
        for config in configs:
            for obj_combo in itertools.combinations(self.get_objects(), config[0]):
                for rel_combo in itertools.combinations(self.get_relations(), config[1]):
                    yield Factor(variables=obj_combo + rel_combo)


class Factor:
    def __init__(self, variables=[]):
        self.vars = variables
        self.map = {}

    def connect_var(self, var):
        self.vars.append(var)


class Var:
    def __init__(self, name, var_type='object', factors=[], value=None, properties={}):
        self.name = name
        self.value = value
        self.factors = factors
        self.properties = properties
        self.type = var_type

    def connect_factor(self, factor):
        self.factors.append(factor)

    def return_objects(self):
        if self.type == "relation":
            return [int(self.name[self.name.find('_', 1) + 1:self.name.find('_', 2)]),
                    int(self.name[self.name.find('_', 2) + 1:])]
        else:
            raise TypeError("This method should only be used with relation variables")
