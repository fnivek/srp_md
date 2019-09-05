# Python
from __future__ import absolute_import
from builtins import str
import itertools
import logging
import copy

# Project
import srp_md

# Can not pickle an object (or deepcopy) with a logger
logger = logging.getLogger(__name__)


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

    def get_rel_by_objs(self, obj1, obj2):
        for relation in self.relations:
            if (relation.object1 == obj1) and (relation.object2 == obj2):
                return relation
            elif (relation.object1 == obj2) and (relation.object2 == obj1):
                return relation
        return None

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

    @classmethod
    def make_relation(cls, obj1, obj2):
        """ Generates a new relation from an object pair.

        WARNING: Does not give a uuid instead just 0. Function user must assign a uuid.

        """
        name = 'R_{}_{}'.format(obj1.name, obj2.name)
        relation = srp_md.Var(name, uuid=0, var_type='relation', num_states=len(SceneGraph.RELATION_STRS))
        setattr(relation, 'object1', obj1)
        setattr(relation, 'object2', obj2)
        return relation

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
                    # Get the relation of those obj_pair (in order)
                    relation = self.get_rel_by_objs(obj_pair[0], obj_pair[1])
                    if relation is None:
                        raise ValueError("Expected Var but got None!")
                    # If reversed flip value
                    if relation.object1 != obj_pair[0]:
                        relation = self.flip_rel(relation)
                    var_list.append(relation)
                # Finally return the factor of this variables list
                yield srp_md.Factor(variables=var_list)

    def flip_rel(self, rel):
        rel_cp = copy.deepcopy(rel)
        rel_cp.assignment['value'] = SceneGraph.REV_RELATION_DICT[rel.assignment['value']]
        return rel_cp

    def markov_blanket(self, vars):
        """ Get all vars in the markov blanket.

        Assumptions:
          * vars is a set of objects in the scene graph and all of there possible relations

        """
        # Split into objects and relations
        relations = []
        objs = []
        for var in vars:
            if var.type == 'relation':
                relations.append(var)
            else:
                objs.append(var)
        # Grab all relations and there objects that have a single common object
        mb = set()
        for relation in self.relations:
            if relation in relations:
                continue
            if relation.object1 in objs:
                mb.add(relation.object2)
                mb.add(relation)
            elif relation.object2 in objs:
                mb.add(relation.object1)
                mb.add(relation)

        # logger.debug('Markov blanket of {} is {}'.format([var.name for var in vars], [var.name for var in mb]))
        return mb

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
            var = srp_md.Var(name='R_{}_{}'.format(id_1, id_2), uuid=self.get_new_uuid(), var_type="relation")
            setattr(var, 'object1', pair[0])
            setattr(var, 'object2', pair[1])
            # TODO(Henry): Remove this magic number 6, it is the number of types of relations in scenegraph.cpp
            #              this number should come from something like a config file that specifies our domain.
            var.num_states = 6
            self.relations.append(var)
            self._vars.append(var)
