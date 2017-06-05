import numpy
import VectorOps.VectorMath as VectorMath

'''
line is a tuple of length two containing the first and second point, in that order.
Contain is whether or not to return a point if the line does not contain the point between its two points
(i.e. setting to false means it can return a point outside of the segment but along the same line)
'''
def get_x_of_line_at_y(line, y, contain):
    if contain and not line_contains_y(line, y):
        return None

    line_subtract = line[1]-line[0]
    y_ratio = float(y - line[0][1])/float(line_subtract[1])
    intersect_point = line[0] + (y_ratio * line_subtract)
    return intersect_point[0]

'''
does not return true if y rests on top of a corner
'''
def line_contains_y(line, y):
    return  ((line[0][1] > y and line[1][1] < y) or (line[0][1] < y and line[1][1] > y))

def distance_between_lines(line1, line2, bounded):
    t_line_1, t_line_2 = get_points_of_minimum_distance_between_lines_3d(line1, line2, bounded)
    return numpy.linalg.norm(t_line_2 - t_line_1)

def distance_from_segment_to_point(segment, point, bounded = True):
    point_sub = point - segment[0]
    segment_mag = numpy.linalg.norm(segment[1] - segment[0])
    vec_rejection = VectorMath.vec_reject(point_sub, segment[1]-segment[0])

    segment_unit = (segment[1]-segment[0])/numpy.linalg.norm(segment[1]-segment[0])
    vec_projection_t = numpy.dot(point_sub, segment_unit)
    mag_vec_rejection = numpy.linalg.norm(vec_rejection)
    if bounded:
        if vec_projection_t > segment_mag:
            return numpy.linalg.norm(point - segment[1])
        elif vec_projection_t < 0:
            return numpy.linalg.norm(point_sub)
    return mag_vec_rejection


def point_of_segment_at_t(segment, t):
    return segment[0] + (segment[1]-segment[0]) * t

def get_intersection_between_segments(segment1, segment2, bounded = True, minimum_distance = 0.05):
    point1, point2 = get_points_of_minimum_distance_between_lines_3d(segment1, segment2, bounded)
    if numpy.linalg.norm(point2-point1) > minimum_distance:
        return None
    '''will just return midpoint if they are minisculely close'''
    return (point1 + point2)/2.0


def get_points_of_minimum_distance_between_lines_3d(line1, line2, bounded):
    a1 = line1[0]
    a2 = line1[1]
    a_sub = a2-a1
    b1 = line2[0]
    b2 = line2[1]
    b_sub = b2-b1

    t_of_line1 = get_t_of_point_closest_to_line(line1, line2, bounded)
    t_of_line2 = get_t_of_point_closest_to_line(line2, line1, bounded)


    '''if the value of t for either line is outside of the range it can be to be within either line, return None'''
    if bounded:
        if t_of_line1 > 1:
            t_of_line1 = 1
        elif t_of_line1 < 0:
            t_of_line1 = 0
        if t_of_line2 > 1:
            t_of_line2 = 1
        elif t_of_line2 < 0:
            t_of_line2 = 0


    point_on_line1 = a1 + a_sub*t_of_line1
    point_on_line2 = b1 + b_sub*t_of_line2

    return point_on_line1, point_on_line2

'''returns the value of t in v0 + (v1-v0)t such that the point it returns is the closest point to the other line being compared to.
If looking for perfect'''
def get_t_of_point_closest_to_line(base_line, compare_line, bounded, exact_intersect = False):
    a1 = compare_line[0]
    a2 = compare_line[1]
    a_sub = a2-a1
    b1 = base_line[0]
    b2 = base_line[1]
    b_sub = b2-b1

    cross_a_with_b = numpy.cross(a_sub, b_sub)
    '''checks if the two lines exactly intersect each other'''
    if exact_intersect:
        if not (numpy.dot(cross_a_with_b, a1) == numpy.dot(cross_a_with_b, b1)):
            return None

    cross_a_with_cross_a_with_b = numpy.cross(a_sub, cross_a_with_b)
    solve_constant = numpy.dot(a1, cross_a_with_cross_a_with_b)
    t_of_base_line = (solve_constant - cross_a_with_cross_a_with_b.dot(b1))/(cross_a_with_cross_a_with_b.dot(b_sub))
    if bounded:
        if t_of_base_line > 1:
            t_of_base_line = 1
        elif t_of_base_line < 0:
            t_of_base_line = 0
    return t_of_base_line
