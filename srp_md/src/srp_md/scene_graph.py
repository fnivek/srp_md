# Python
from __future__ import absolute_import
from builtins import str
import itertools

# Project
import srp_md


class SceneGraph(srp_md.FactorGraph):
    # Class constants
    RELATION_STRS = ['disjoint', 'in', 'on', 'contain', 'support', 'proximity']
    REV_RELATION_DICT = {
        "disjoint":  "disjoint",
        "proximity": "proximity",
        "on":        "support",
        "support":   "on",
        "in":        "contain",
        "contain":   "in"
    }

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
        return [obj.assignment['value'] for obj in self.objs]

    def get_rel_names(self):
        return [rel.name for rel in self.relations]

    def get_rel_values(self):
        return [rel.assignment['value'] for rel in self.relations]

    def get_prop_values(self, prop=None):
        if prop is None:
            raise ValueError("Property needs to be specified")
        else:
            return [obj.assignment[prop] for obj in self.objs]

    def get_rel_value_from_name(self, name):
        for rel in self.relations:
            if rel.name == name:
                return rel.assignment['value']
        else:
            return None

    def get_objects(self):
        return [var for var in self._vars if var.type == "object"]

    def get_relations(self):
        return [var for var in self._vars if var.type == "relation"]

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
                            rev_val = SceneGraph.REV_RELATION_DICT[rev_var.assignment['value']]
                            temp_var = srp_md.Var(rel_name, var_type='relation', value=rev_val)
                            var_list.append(temp_var)
                    # Else if the relation exists, just add it to the list
                    else:
                        var_list.append(relation)
                # Finally return the factor of this variables list
                yield srp_md.Factor(variables=var_list)

    def check_consistency(self, world=None):
        # If no world specified, just return True
        if world is None:
            return None

        # Initialize
        relations = list(self.relations)

        # While the relation list is not empty, do:
        while len(relations) != 0:
            # Get an existing relation and variables in this relation
            rel_1 = relations.pop()

            # Compare with other relations
            for rel_2 in relations:
                # Depending on the world, pick function
                if world == "block":
                    # If not consistent with rel_2, then return False
                    if not self.block_consistent(rel_1, rel_2):
                        return False
                elif world == "pen" or world == "book":
                    # If not consistent with rel_2, then return False
                    if not self.pen_book_consistent(rel_1, rel_2):
                        return False
                elif world == "abstract":
                    # If not consistent with rel_2, then return False
                    if not self.abstract_consistent(rel_1, rel_2):
                        return False
                else:
                    return True

        # If all relations are consistent with each other, then return True
        return True

    def block_consistent(self, rel_1, rel_2):
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

    def pen_book_consistent(self, rel_1, rel_2):
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
            if values[0] == 'left':
                if values[1:] == ['left', 'right']:
                    consistency = False

            elif values[0] == 'right':
                if values[1:] == ['right', 'left']:
                    consistency = False

        # If they have more than 1 common object id, then something is wrong
        elif len(ids) <= 2:
            raise IndexError('Something is wrong!')

        return consistency

    def abstract_consistent(self, rel_1, rel_2):
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
            if values[0] == 'smaller':
                if values[1:] == [['smaller', 'equal'], ['smaller', 'bigger'],
                                  ['equal', 'equal'], ['equal', 'bigger']]:
                    consistency = False

            elif values[0] == 'equal':
                if values[1:] == [['smaller', 'equal'], ['smaller', 'bigger'],
                                  ['equal', 'smaller'], ['equal', 'bigger'],
                                  ['bigger', 'smaller'], ['bigger', 'equal']]:
                    consistency = False

            elif values[0] == 'bigger':
                if values[1:] == [['equal', 'smaller'], ['equal', 'equal'],
                                  ['bigger', 'smaller'], ['bigger', 'equal']]:
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
            var = srp_md.Var(name='R_{}_{}'.format(id_1, id_2), var_type="relation")
            setattr(var, 'object1', pair[0].name)
            setattr(var, 'object2', pair[1].name)
            # TODO(Henry): Remove this magic number 6, it is the number of types of relations in scenegraph.cpp
            #              this number should come from something like a config file that specifies our domain.
            var.num_states = 6
            self.relations.append(var)
            self._vars.append(var)
