from Geometry.Rectangle import Rectangle
from math import atan2, acos
import numpy

def get_flat_bounds_of_vectors(vectors):
    leftest = vectors[0][0]
    rightest = vectors[0][0]
    uppest = vectors[0][1]
    downest = vectors[0][1]

    for i in range(1, len(vectors)):
        if vectors[i][0] < leftest:
            leftest = vectors[i][0]
        if vectors[i][0] > rightest:
            rightest = vectors[i][0]
        if vectors[i][1] < downest:
            downest = vectors[i][1]
        if vectors[i][1] > uppest:
            uppest = vectors[i][1]
    return Rectangle(leftest, uppest, rightest-leftest, uppest-downest)

'''can take multi dimensional vectors, but only uses the indices listed in "indices", where the first index is treated like x, and the
second is treated like y'''
def get_flat_angle_intersecting_points(origin, compare, indices = (0,1)):
    dy = compare[indices[1]] - origin[indices[1]]
    dx = compare[indices[0]] - origin[indices[0]]
    return atan2(dy, dx)

def dot_angle_between(v1, v2):
    return acos(numpy.dot(v1,v2)/(numpy.linalg.norm(v1) * numpy.linalg.norm(v2)))

def vec_proj(projection_vector, base_vector):
    return (numpy.dot(projection_vector, base_vector)/numpy.linalg.norm(base_vector)**2)*base_vector

def vec_reject(rejection_vector, base_vector):
    return rejection_vector - vec_proj(rejection_vector, base_vector)

def scalar_proj(projection_vector, base_vector):
    return numpy.dot(projection_vector, base_vector/numpy.linalg.norm(base_vector))

'''cannot return a negative scalar rejection'''
def scalar_reject(rejection_vector, base_vector):
    orthog_vector = vec_reject(rejection_vector, base_vector)
    return scalar_proj(rejection_vector, orthog_vector)
