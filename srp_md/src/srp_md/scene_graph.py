import srp_md

import itertools


class SceneGraph(srp_md.FactorGraph):
    # Class constants
    RELATIONS = ["disjoint", "in", "on", "contain", "support", "proximity"]

    def __init__(self, objs=None):
        super(SceneGraph, self).__init__()
        if objs is None:
            objs = list()
        self.objs = objs
        self._vars.extend(self.objs)
        self.relations = []
        self.generate_relations()

    def num_objs(self):
        return len(self.objs)

    def num_relations(self):
        return len(self.relations)

    def get_obj_names(self):
        return [obj.name for obj in self.objs]

    def generate_relations(self, reset=True):
        """ Generate all relation variables

        Generates all possible combinations of relations between objects.

        """
        # Reset all relations
        # TODO(?): Keep existing relations
        if reset:
            self._vars = [var for var in self._vars if var not in self.relations]
            self.relations = []

        # Generate all combos of objects
        for pair in itertools.combinations(self.objs, 2):
            # Make a new relationship var
            var = srp_md.Var(name='R_{}_{}'.format(pair[0].name, pair[1].name))
            self.relations.append(var)
            self._vars.append(var)
