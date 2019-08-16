from .factor_graph import FactorGraph, Var
from builtins import str
import itertools


class SceneGraph(FactorGraph):
    # Class constants
    RELATIONS = ['disjoint', 'on', 'support', 'proximity']

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

    def get_obj_values(self):
        return [obj.value for obj in self.objs]

    def get_rel_names(self):
        return [rel.name for rel in self.relations]

    def get_rel_values(self):
        return [rel.value for rel in self.relations]

    def get_rel_value_from_name(self, name):
        for rel in self.relations:
            if rel.name == name:
                return rel.value
        else:
            return None

    def check_consistency(self):
        # Initialize
        relations = list(self.relations)

        # While the relation list is not empty, do:
        while len(relations) != 0:
            # Get an existing relation and variables in this relation
            rel_1 = relations.pop()

            # Compare with other relations
            for rel_2 in relations:
                # If not consistent with rel_2, then return False
                if not self.is_consistent(rel_1, rel_2):
                    return False

        # If all relations are consistent with each other, then return True
        return True

    def is_consistent(self, rel_1, rel_2):
        consistency = True

        # Get the object id's of rel_1 and rel_2
        var_i = rel_1.name[rel_1.name.find('_', 1) + 1:rel_1.name.find('_', 2)]
        var_j = rel_1.name[rel_1.name.find('_', 2) + 1:]
        var_k = rel_2.name[rel_2.name.find('_', 1) + 1:rel_2.name.find('_', 2)]
        var_l = rel_2.name[rel_2.name.find('_', 2) + 1:]

        # Put the object ids into a set
        ids = list(set([int(var_i), int(var_j), int(var_k), int(var_l)]))

        # If rel_1 and rel_2 have common object id, do:
        if len(ids) == 3:
            # Make sure that rel_1 and rel_2 and rel_3 are in order: R_i_j, R_j_k, R_i_k, where i < j < k
            ids.sort()

            # Get existing relation values, and one or more doesn't exist yet, it is consistent except few cases
            values = []
            values.append(self.get_rel_value_from_name('R_' + str(ids[0]) + '_' + str(ids[1])))
            values.append(self.get_rel_value_from_name('R_' + str(ids[1]) + '_' + str(ids[2])))
            values.append(self.get_rel_value_from_name('R_' + str(ids[0]) + '_' + str(ids[2])))

            # Do if statements for each cases
            # Should work for now, try reducing the code later
            if values[0] == 'disjoint':
                if values[1:] in [['on', 'on'],
                                  ['support', 'support']]:
                    consistency = False

            elif values[0] == 'proximity':
                if values[1:] in [['on', 'proximity'], ['on', 'on'],
                                  ['support', 'proximity'], ['support', 'support'],
                                  ['proximity', 'on'], ['proximity', 'support']]:
                    consistency = False

            elif values[0] == 'on':
                if values[1] == 'support':
                    consistency = False
                elif values[1:] in [['disjoint', 'on'],
                                    ['on', 'proximity'], ['on', 'on'], ['on', 'support'],
                                    ['proximity', 'proximity'], ['proximity', 'on'], ['proximity', 'support']]:
                    consistency = False

            elif values[0] == 'support':
                if values[1] == 'on':
                    consistency = False
                elif values[1:] in [['disjoint', 'support'],
                                    ['support', 'proximity'], ['support', 'on'], ['support', 'support'],
                                    ['proximity', 'proximity'], ['proximity', 'on'], ['proximity', 'support']]:
                    consistency = False

            elif values[0] is None:
                if values[1:] in [['on', 'on'],
                                  ['support', 'support']]:
                    consistency = False

        # If they have more than 1 common object id, then something is wrong
        elif len(ids) <= 2:
            raise IndexError('Something is wrong!')

        return consistency

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
            id_1 = pair[0].name[pair[0].name.find('_') + 1:]
            id_2 = pair[1].name[pair[1].name.find('_') + 1:]
            var = Var(name='R_{}_{}'.format(id_1, id_2))
            self.relations.append(var)
            self._vars.append(var)
