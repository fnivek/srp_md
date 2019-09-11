# Python
from __future__ import absolute_import
from builtins import str
import itertools
import logging
import copy

# Project
import srp_md
from srp_md.msg import Factor as RosFactor
from srp_md.msg import ObjectPair

# Can not pickle an object (or deepcopy) with a logger
logger = logging.getLogger(__name__)


class SceneGraph(srp_md.FactorGraph):
    # Class constants
    RELATION_STRS = ['disjoint', 'in', 'on', 'contain', 'support', 'proximity']
    REV_RELATION_DICT = {
        "disjoint": "disjoint",
        "proximity": "proximity",
        "on": "support",
        "support": "on",
        "in": "contain",
        "contain": "in"
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

    def print_objs(self):
        return {obj.name: [(ass, obj.assignment[ass]) for ass in obj.assignment.keys()] for obj in self.objs}

    def print_rels(self):
        return {rel.name: rel.value for rel in self.relations}

    def get_obj_names(self):
        return [obj.name for obj in self.objs]

    def get_rel_names(self):
        return [rel.name for rel in self.relations]

    def get_rel_values(self):
        return [rel.value for rel in self.relations]

    def get_rel_by_objs(self, obj1, obj2):
        for relation in self.relations:
            if (relation.obj1 == obj1) and (relation.obj2 == obj2):
                return relation
            elif (relation.obj1 == obj2) and (relation.obj2 == obj1):
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
                return rel.value
        else:
            return None

    def generate_relations(self):
        """ Generate all relation variables

        Generates all possible combinations of relations between objects.

        """
        # Generate all combos of objects
        for pair in itertools.combinations(self.objs, 2):
            relation = Relation(list(pair), uuid=self.get_new_uuid(), value=None)
            self.relations.append(relation)
            self._vars.append(relation)
            pair[0].add_relation(relation)
            pair[1].add_relation(relation)

    def reset_relations(self):
        # Reset all relations
        for relation in self.relations:
            relation.value = None

    def gen_ordered_factors(self, configs):
        # For each permutation of the objects, calculate all factors:
        for obj_permu in itertools.permutations(self.objs, self.num_objs()):
            # For each (obj, rel) type of factor
            for config in configs:
                # If the number of relations do not match with corresponding number of objects, yell
                if config[1] != srp_md.tri_num(config[0] - 1):
                    raise ValueError('Can not make factors of type ({} objs, {} rels)'.format(config[0], config[1]))
                # For each combination of config[0] objects get all relations between them
                for objs in itertools.combinations(obj_permu, config[0]):
                    # Get the relation for each pairing
                    rels = []
                    for obj_pair in itertools.combinations(objs, 2):
                        # Get the relation of those objs (in order)
                        relation = self.get_rel_by_objs(obj_pair[0], obj_pair[1])
                        # If reversed flip value
                        if relation.obj1 != obj_pair[0]:
                            relation = self.flip_rel(relation)
                        rels.append(relation)
                    # Finally yield the factor of these variables list
                    yield srp_md.SgFactor(variables=list(objs) + rels)

    def flip_rel(self, rel):
        rel_cp = copy.deepcopy(rel)
        rel_cp.value = SceneGraph.REV_RELATION_DICT[rel.value]
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
            if isinstance(var, Relation):
                relations.append(var)
            else:
                objs.append(var)
        # Grab all relations and there objects that have a single common object
        mb = set()
        for relation in self.relations:
            if relation in relations:
                continue
            if relation.obj1 in objs:
                mb.add(relation.obj2)
                mb.add(relation)
            elif relation.obj2 in objs:
                mb.add(relation.obj1)
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
                elif world == "block_strict":
                    # If not consistent with rel_2, then return False
                    if not self.block_strict_consistent(rel_1, rel_2):
                        return False
                else:
                    return True

        # If all relations are consistent with each other, then return True
        return True

    def block_strict_consistent(self, rel_1, rel_2):
        consistency = True

        # Get the object id's of rel_1 and rel_2
        objs = list(set(rel_1.get_objs() + rel_2.get_objs()))
        ids = [int(obj.id) for obj in objs]

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
        objs = list(set(rel_1.get_objs() + rel_2.get_objs()))
        ids = [int(obj.id) for obj in objs]

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
        objs = list(set(rel_1.get_objs() + rel_2.get_objs()))
        ids = [int(obj.id) for obj in objs]

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

    def block_consistent(self, rel_1, rel_2):
        consistency = True

        # Get the object id's of rel_1 and rel_2
        objs = list(set(rel_1.get_objs() + rel_2.get_objs()))
        ids = [int(obj.id) for obj in objs]

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
                if values[1:] in [['on', 'on'], ['on', 'support'],
                                  ['support', 'on'], ['support', 'support']]:
                    consistency = False

            elif values[0] == 'on':
                if values[1:] in [['on', 'support'], ['on', 'disjoint'],
                                  ['support', 'disjoint'],
                                  ['disjoint', 'on'], ['disjoint', 'support']]:
                    consistency = False

            elif values[0] == 'support':
                if values[1:] in [['on', 'disjoint'],
                                  ['support', 'on'], ['support', 'disjoint'],
                                  ['disjoint', 'on'], ['disjoint', 'support']]:
                    consistency = False

        # If they have more than 1 common object id, then something is wrong
        elif len(ids) <= 2:
            raise IndexError('Something is wrong!')

        return consistency


class Object(srp_md.Var):
    def __init__(self, id_num=0, uuid=0, assignment=None, relations=None):
        super(Object, self).__init__(name='X_{}'.format(id_num), uuid=uuid, assignment=assignment, num_states=1)
        self.id = id_num
        self.relations = relations
        if relations is None:
            self.relations = []

    def add_relation(self, relation):
        self.relations.append(relation)


class Relation(srp_md.Var):
    def __init__(self, objs, uuid=0, value=None):
        super(Relation, self).__init__(name=None, uuid=uuid, num_states=len(SceneGraph.RELATION_STRS))
        if len(objs) != 2 or any([not isinstance(obj, Object) for obj in objs]):
            raise TypeError('objs must be an itterable of 2 srp_md.Object classes')
        self.obj1 = objs[0]
        self.obj2 = objs[1]
        self.name = 'R_{}_{}'.format(self.obj1.id, self.obj2.id)
        self.value = value

    def get_objs(self):
        return [self.obj1, self.obj2]

    @property
    def value(self):
        return self.assignment['value']

    @value.setter
    def value(self, value):
        self.assignment['value'] = value


class SgFactor(srp_md.Factor):
    """
    Factor that corresponds to scene graph.
    """
    def __init__(self, variables=None, probs=None):
        self.vars = variables
        if variables is None:
            self.vars = []
        self.objs = [var for var in self.vars if isinstance(var, srp_md.Object)]
        self.relations = [var for var in self.vars if isinstance(var, srp_md.Relation)]
        self.probs = probs

    def connect_vars(self, vars):
        self.vars.extend(vars)
        self.objs.extend([var for var in self.vars if isinstance(var, srp_md.Object)])
        self.relations.extend([var for var in self.vars if isinstance(var, srp_md.Relation)])

    def to_ros_factor(self):
        ros_factor = RosFactor()
        ros_factor.objects = [obj.name for obj in self.objs]
        for relation in self.relations:
            pair = ObjectPair()
            pair.object1 = relation.obj1.name
            pair.object2 = relation.obj2.name
            ros_factor.pairs.append(pair)
        ros_factor.probs = self.probs
        return ros_factor
