import numpy
import Geometry.PolyMath as PolyMath
from Geometry.FlatPolygon import FlatPolygon

class PolyPlane:
    '''when calculating if a point lies on the plane, you must dot it with the plane's normal after subtracting the origin from the
    point. If this is zero, the point lies on the unbounded plane. However, if there is anything lossy, then there is a chance that
    the dot product will not equal zero, but something very close to it. So long as the dot with the normal is accurate to the error
    margin, it will be considered within the bounded plane'''
    PLANE_DOT_PRODUCT_MARGIN_OF_ERROR = 0.001
    POINT_IN_POLYGON_BOUNDED_AREA_TOLERANCE_CONSTANT = 0.01
    '''polygon must be convex relative to the origin'''
    def __init__(self, bounding_points):
        self.bounding_points = bounding_points
        self.origin = numpy.average(self.bounding_points, axis = 0)
        self.init_unit_normal()
        self.init_bounded_area()
        self.init_plane_basis_vectors()
        self.init_flat_basis_polygon()

    def init_unit_normal(self):
        v1 = self.bounding_points[2]
        v2 = self.bounding_points[0]
        origin_v = self.bounding_points[1]

        sub_v1 = v1 - origin_v
        sub_v2 = v2 - origin_v
        cross_between = numpy.cross(sub_v1, sub_v2)
        self.unit_normal = cross_between/numpy.linalg.norm(cross_between)

    def init_bounded_area(self):
        '''is a little bit of a clumsy way of calculating the area of the polygon plane'''
        triangles = PolyMath.get_triangles_from_corners_to_point(self.bounding_points, self.origin)
        self.bounded_area = self.get_total_area_of_triangles(triangles)
        self.point_in_polygon_bounded_area_threshold = self.bounded_area*PolyPlane.POINT_IN_POLYGON_BOUNDED_AREA_TOLERANCE_CONSTANT + self.bounded_area

    def point_lies_in_bounded_plane(self, point):
        if not self.point_lies_in_unbounded_plane(point):
            return False
        triangles_to_point = PolyMath.get_triangles_from_corners_to_point(self.bounding_points, point)
        total_area = self.get_total_area_of_triangles(triangles_to_point)
        '''this calculation may be subject to the same problems that the dot product calculation could have -- where if there is lossyness
        in calculation, points that do lie within the bounded plane will not be counted as such. May have to include a ratio'd margin of
        error later'''
        return (total_area <= self.point_in_polygon_bounded_area_threshold)

    def line_lies_in_unbounded_plane(self, segment_points):
        for i in range(0, len(segment_points)):
            if not self.point_lies_in_unbounded_plane(segment_points[i]):
                return False
        return True


    '''inits a set of unit basis vectors where the first axis is determined by taking the unit vector of a single corner to
    the origin, the third is the normal vector, and the second is determiend through taking the cross product of the two.
    Is important to note that the set of vectors can be flipped to be reversed from what is expected'''
    def init_plane_basis_vectors(self):
        basis1_sub = self.bounding_points[0] - self.origin
        basis1 = basis1_sub / numpy.linalg.norm(basis1_sub)

        basis3 = self.unit_normal
        basis2_scaled = numpy.cross(basis1, basis3)
        basis2 = basis2_scaled / numpy.linalg.norm(basis2_scaled)
        self.basises = numpy.array([basis1, basis2, basis3])


    def init_flat_basis_polygon(self):
        flat_basis_bounds = [(self.bounding_points[i]-self.origin).dot(self.basises) for i in range(0, len(self.bounding_points))]
        self.flat_basis_polygon = FlatPolygon(flat_basis_bounds)

    def get_total_area_of_triangles(self, triangles):
        total_area = 0
        for i in range(0, len(triangles)):
            total_area += PolyMath.area_of_triangle(triangles[i])
        return total_area

    def point_lies_in_unbounded_plane(self, point):
        point_dot = numpy.dot(point - self.origin, self.unit_normal)
        return abs(point_dot) < PolyPlane.PLANE_DOT_PRODUCT_MARGIN_OF_ERROR

    def distance_to_point(self, point):
        return abs(numpy.dot(point - self.origin, self.unit_normal))


    def get_corners_sorted_by_distance_to_point(self, point, reverse = False):
        indexes = [i for i in range(0, len(self.bounding_points))]
        return sorted(indexes, key = lambda index: numpy.linalg.norm(self.bounding_points[index] - point), reverse = reverse)
        '''smallest_dist = numpy.linalg.norm(point - self.bounding_points[0])
        smallest_dist_index = 0
        for i in range(1, len(self.bounding_points)):
            iter_dist = numpy.linalg.norm(point - self.bounding_points[i])
            if iter_dist < smallest_dist:
                smallest_dist = iter_dist
                smallest_dist_index = i
        return smallest_dist_index'''

    def get_path_between_start_and_end_around_perimeter(self, start_point, end_point):
        corners_sorted_by_distance_to_start = self.get_corners_sorted_by_distance_to_point(start_point)
        start_index = corners_sorted_by_distance_to_start[0]

        corners_sorted_by_distance_to_end = self.get_corners_sorted_by_distance_to_point(end_point)
        end_index = corners_sorted_by_distance_to_end[0]

        step_positive_distance = self.get_distance_from_corner_index_to_corner_index_using_step(start_index, end_index, 1)
        step_negative_distance = self.get_distance_from_corner_index_to_corner_index_using_step(start_index, end_index, -1)
        step = 1 if step_positive_distance < step_negative_distance else -1

        i = start_index
        end_found = False
        corner_path = []
        while not end_found:
            corner_path.append(self.bounding_points[i])
            if i == end_index:
                end_found = True
            if step > 0:
                if i < len(self.bounding_points)-1:
                    i += step
                else:
                    i = 0
            else:
                if i > 0:
                    i += step
                else:
                    i = len(self.bounding_points)-1
        return corner_path

    def get_distance_from_corner_index_to_corner_index_using_step(self, start_index, end_index, step):
        end_found = False
        distance_sum = 0
        i = (start_index + step)%len(self.bounding_points)
        while not end_found:
            distance_sum += numpy.linalg.norm(self.bounding_points[i] - self.bounding_points[(i-step)%len(self.bounding_points)])
            if i == end_index:
                end_found = True
            if step > 0:
                if i < len(self.bounding_points)-1:
                    i += step
                else:
                    i = 0
            else:
                if i > 0:
                    i += step
                else:
                    i = len(self.bounding_points)-1
        return distance_sum
