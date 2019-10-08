""" Geometry.

This file is for represetning and manipulating geometries.

"""
import numpy

""" Representations.

Here are classes for representing geometries.

"""


class Sphere:
    def __init__(self, radius, center):
        self.radius = radius
        self.center = center


class Obb:
    def __init__(self, center, orientation, dimensions):
        self.center = center
        self.orientation = orientation
        self.dimensions = dimensions

    def bounding_sphere(self):
        return Sphere(self.center, numpy.linalg.norm(self.dimensions))


""" Algorithms.

Here are algorithms on geometry, e.g. collsion detection.

"""


def sphere_collision(spheres):
    collisions = []
    num_spheres = len(spheres)
    for i in range(num_spheres):
        for j in range(i + 1, num_spheres):
            dis = numpy.linalg.norm(spheres[i].center - spheres[j].center)
            collsion_dis = spheres[i].radius + spheres[j].radius
            if dis < collsion_dis:
                collisions.append((i, j))
    return collisions


def obb_collision(obbs):
    """ Collision test for oriented bounding boxes.
    """
    # Get a shortlist of oob pairs to check by doing sphere collision
    spheres = [oob.bounding_sphere() for oob in oobs]
    shortlist = sphere_collision(spheres)
    print shortlist


""" Tests.

Unit tests.

"""


def sphere_collision_test():
    spheres = [Sphere(1, numpy.mat([0, 0, 0])),
               Sphere(0.4, numpy.mat([0, 0, 0])),
               Sphere(100, numpy.mat([0, 0, -50])),
               Sphere(1, numpy.mat([10, 0, 0]))]
    print sphere_collision(spheres)


def obb_collision_test():
    center, orientation, dimensions
    obbs = [Obbs()]


def test():
    sphere_collision_test()
    obb_collision_test()


if __name__ == '__main__':
    test()
