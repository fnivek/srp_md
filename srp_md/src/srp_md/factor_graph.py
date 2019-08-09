from builtins import range
import itertools


class FactorGraph:
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

    def gen_all_possible_factors(self):
        for i in range(1, self.num_vars() + 1):
            for combo in itertools.combinations(self._vars, i):
                yield Factor(variables=combo)


class Factor:
    def __init__(self, variables=[]):
        self.vars = variables
        self.map = {}

    def connect_var(self, var):
        self.vars.append(var)


class Var:
    def __init__(self, name, factors=[], value=None):
        self.name = name
        self.value = value
        self.factors = factors

    def connect_factor(self, factor):
        self.factors.append(factor)
