import numpy

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
    '''sub_line1 = line1[1] - line1[0]
    sub_line2 = line2[1] - line2[0]

    sub_line1line2_cross = numpy.cross(sub_line1, sub_line2)

    distance = numpy.linalg.norm((sub_line1line2_cross/numpy.linalg.norm(sub_line1line2_cross)).dot(line1[0]-line2[0]))'''



def point_of_segment_at_t(segment, t):
    return segment[0] + (segment[1]-segment[0]) * t

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
        if (t_of_line1 < 0 or t_of_line1 > 1) or (t_of_line2 < 0 or t_of_line2 > 1):
            return None, None

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

'''
def get_intersection_between_lines_3d2(line1, line2, bounded):
    a1 = line1[0]
    a2 = line1[1]
    mag_a1a2 = numpy.linalg.norm(a2-a1)
    b1 = line2[0]
    b2 = line2[1]
    mag_b1b2 = numpy.linalg.norm(b2-b1)
    t_of_b = get_intersection_between_lines_3d(line1, line2, bounded)
    t_of_a = get_intersection_between_lines_3d(line2, line1, bounded)
    point_on_a = a1 + (a2-a1) * t_of_a
    point_on_b = b1 + (b2-b1) * t_of_b

    shortest_dist_between_lines = numpy.linalg.norm((numpy.cross(a2-a1, b2-b1)/numpy.linalg.norm(numpy.cross(a2-a1, b2-b1))).dot(a1-b1))
    print("shortest_dist_between_lines: ", shortest_dist_between_lines)
    print("point on a: ", point_on_a)
    print("point on b: ", point_on_b)
    print("mag between point a and b: ", numpy.linalg.norm(point_on_a - point_on_b))

def get_intersection_between_lines_3d(line1, line2, bounded):
    a1 = line1[0]
    a2 = line1[1]
    mag_a1a2 = numpy.linalg.norm(a2-a1)
    b1 = line2[0]
    b2 = line2[1]
    mag_b1b2 = numpy.linalg.norm(b2-b1)

    cross = numpy.cross((a2-a1), (b2-b1))
    a_constant = numpy.dot(cross, a1)
    b_constant = numpy.dot(cross, b1)
    print("a constant == b constant? ", (a_constant == b_constant))
    cross_a_with_cross = numpy.cross(a2-a1, cross)
    second_cross_constant = numpy.dot(a1, cross_a_with_cross)
    #cross_a_with_cross . (b1 + (b2-b1)t) = second_cross_constant
    #cross_a_with_cross . b1 + cross_a_with_cross . (b2-b1)t = second_cross_constant
    #cross_a_with_cross . (b2-b1)t = second_cross_constant - cross_a_with_cross . b1
    #t = (second_cross_constant - cross_a_with_cross . b1)/(cross_a_with_cross . (b2-b1))
    t_of_b = (second_cross_constant - cross_a_with_cross.dot(b1))/(cross_a_with_cross.dot(b2-b1))

    point_of_intersection = b1 + (b2-b1)*t_of_b
    return t_of_b

'''
