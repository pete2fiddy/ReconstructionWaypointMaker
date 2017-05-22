from math import sqrt
import numpy

def area_of_triangle(triangle_points):
    if len(triangle_points) != 3:
        return None
    v1 = triangle_points[2]
    v2 = triangle_points[0]

    origin = triangle_points[1]
    sub_v1 = v1-origin
    sub_v2 = v2-origin

    mag_sub_v1 = numpy.linalg.norm(sub_v1)
    mag_sub_v2 = numpy.linalg.norm(sub_v2)

    cos_angle_between = numpy.dot(sub_v1, sub_v2)/(mag_sub_v1 * mag_sub_v2)
    '''In one bizarre error-case, the above yielded 1, and when printed, printed 1.0. Yet, when it was subtracted from 1 in the square root
    below, the result was a very small NEGATIVE number, crashing the application. The below is to remedy this so that it cannot crash on
    lossy math'''

    if abs(cos_angle_between) > 1.0:
        cos_angle_between = (cos_angle_between/abs(cos_angle_between))
    sin_angle_between = sqrt(1.0 - cos_angle_between**2)
    area =  numpy.linalg.norm(numpy.cross(sub_v1, sub_v2))#sin_angle_between
    return area

def get_triangles_from_corners_to_point(corners, point):
    triangles = []
    for i in range(0, len(corners)):
        p1 = corners[i-1]
        p2 = corners[i]

        append_triangle = [p1, point, p2]
        triangles.append(append_triangle)
    return triangles
