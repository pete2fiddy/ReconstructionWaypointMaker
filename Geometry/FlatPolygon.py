import VectorOps.VectorMath as VectorMath
import Geometry.LineMath as LineMath
import numpy
import Geometry.AngleMath as AngleMath
from math import cos, sin, pi

class FlatPolygon:
    '''
    takes an array of n dimensional points, but only uses the first and second indices of each vector
    '''
    def __init__(self, points):
        self.points = points
        self.bounding_rect = VectorMath.get_flat_bounds_of_vectors(self.points)
        self.origin = numpy.average(self.points, axis = 0)

    '''may be using lists and numpy arrays interchangeably in some places...'''

    '''return polygon scaled by amount margin (positive or negative) so that this polygon will be scaled by an amount that has a constant
    margin of size margin between the old polygon and the new polygon'''
    def get_polygon_scaled_with_margin(self, margin):
        scaled_points = []
        points_subtracted_by_origin = numpy.subtract(self.points, self.origin)
        for i in range(0, len(points_subtracted_by_origin)):
            left_point, right_point = self.get_points_surrounding_point_index(points_subtracted_by_origin, i)
            left_sub_point = left_point - points_subtracted_by_origin[i]
            right_sub_point = right_point - points_subtracted_by_origin[i]
            cos_angle = VectorMath.dot_angle_between(left_sub_point, right_sub_point)
            left_point_angle = AngleMath.normalize_angle_to_0_and_2pi(VectorMath.get_flat_angle_intersecting_points(points_subtracted_by_origin[i], left_point))
            right_point_angle = AngleMath.normalize_angle_to_0_and_2pi(VectorMath.get_flat_angle_intersecting_points(points_subtracted_by_origin[i], right_point))
            halfway_angle = left_point_angle

            if AngleMath.angle_is_between_angles_radians(left_point_angle, right_point_angle, left_point_angle + (cos_angle/2.0)):
                halfway_angle += cos_angle / 2.0
            else:
                halfway_angle -= cos_angle / 2.0



            halfway_angle_vector = numpy.array([cos(halfway_angle), sin(halfway_angle)])

            if numpy.dot(halfway_angle_vector, points_subtracted_by_origin[i][:2]) > 0:
                halfway_angle_vector *= -1
                halfway_angle -= pi

            margin_radius_at_corner = margin/sin(cos_angle/2.0)
            new_point = numpy.array([points_subtracted_by_origin[i][0] + halfway_angle_vector[0] * margin_radius_at_corner, points_subtracted_by_origin[i][1] + halfway_angle_vector[1] * margin_radius_at_corner])
            scaled_points.append(new_point)
        scaled_points = numpy.array(scaled_points)
        return FlatPolygon(scaled_points)

    def get_points_surrounding_point_index(self, points, point_index):
        left_point = points[point_index-1]
        right_point = points[(point_index + 1) % len(points)]
        return left_point, right_point


    def get_floodfill_lines(self, dy):
        out_lines = []
        y = self.bounding_rect.y
        bottom_y = self.bounding_rect.y - self.bounding_rect.height

        while y > bottom_y:
            x_intersects_at_y = self.get_x_intersections_at_y(y, sort_x = True)
            x_lines_at_y = []

            for i in range(0, len(x_intersects_at_y)):
                if i%2 == 0 and i < len(x_intersects_at_y):
                    iter_line = (numpy.array([x_intersects_at_y[i], y]), numpy.array([x_intersects_at_y[i+1], y]))
                    x_lines_at_y.append(iter_line)
            out_lines.append(x_lines_at_y)
            y -= dy
        return out_lines

    def get_x_intersections_at_y(self, y, sort_x = True):
        x_intersects = []
        for i in range(0, len(self.points)):
            '''
            line is from self.points[i-1] to self.points[i].
            Uses i-1 because it will loop back to the last possible index if i is 0
            '''
            iter_line = (self.points[i-1], self.points[i])
            iter_line_x_intersect = LineMath.get_x_of_line_at_y(iter_line, y, True)
            if not (iter_line_x_intersect == None):
                x_intersects.append(iter_line_x_intersect)
        if sort_x:
            x_intersects.sort()
        return x_intersects

    '''ray must be only two points long'''
    def get_segment_intersections(self, segment_points):
        segment_sub = segment_points[1] - segment_points[0]
        segment_points2d = [segment_points[i][:2] for i in range(0, len(segment_points))]
        segment_points2d_sub = segment_points2d[1] - segment_points2d[0]
        v3 = numpy.array([-segment_points2d_sub[1], segment_points2d_sub[0]])

        intersection_points = []
        for i in range(0, len(self.points)):
            iter_polysegment = [self.points[i-1], self.points[i]]
            iter_polysegment_sub = iter_polysegment[1] - iter_polysegment[0]
            iter_polysegment2d = [self.points[i-1][:2], self.points[i][:2]]
            iter_polysegment2d_sub = iter_polysegment2d[1] - iter_polysegment2d[0]

            v1 = segment_points2d[0] - iter_polysegment2d[0]
            v2 = iter_polysegment2d[1] - iter_polysegment2d[0]

            multiple_of_segment = numpy.linalg.norm(numpy.cross(v2, v1))/(numpy.dot(v2, v3))
            multiple_of_polysegment = (numpy.dot(v1,v3))/(numpy.dot(v2, v3))

            if multiple_of_polysegment >= 0  and multiple_of_polysegment <= 1:
                intersection_points.append(iter_polysegment[0] + iter_polysegment_sub * multiple_of_polysegment)
        intersection_points.sort(key = lambda intersection: intersection[0])
        print("len intersection points: ", len(intersection_points))
        return intersection_points
