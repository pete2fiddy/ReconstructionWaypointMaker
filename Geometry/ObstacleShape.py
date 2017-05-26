from abc import ABC, abstractmethod
from Geometry.PolyPlane import PolyPlane
import Geometry.AngleMath as AngleMath

class ObstacleShape(ABC):
    MAX_DISTANCE_TO_BE_DEEMED_INTERSECTING = 0.00001
    @abstractmethod
    def obstacle_points(self):
        pass

    @abstractmethod
    def init_cartesian_points(self, GEO_ORIGIN):
        pass

    @abstractmethod
    def intersect_with_plane(self, plane, resolution, bounded = True):
        pass

    def clip_points_with_plane(self, in_points, plane):
        unclipped_points_plane = PolyPlane(in_points)
        unclipped_basis_vectors = unclipped_points_plane.basises
        '''sorts all the points so that they go from 0 to 2pi counterclockwise when projected onto the plane's basises'''
        points = sorted(list(in_points), key = lambda point: unclipped_points_plane.get_angle_to_coplanar_point_from_first_basis(point))

        '''holds whether the point at index i is in or out of bounds of the clipping plane'''
        point_in_plane_arr = []
        '''holds all intersection points between the polygon and the plane'''
        clipped_intersections = []

        for i in range(0, len(points)):
            point_segment = [points[i-1], points[i]]
            point_in_bounds = plane.point_lies_in_bounded_plane(points[i])
            point_in_plane_arr.append(point_in_bounds)

            segment_intersections_with_plane_edges = plane.get_segment_intersections_with_edges(point_segment, ObstacleShape.MAX_DISTANCE_TO_BE_DEEMED_INTERSECTING)
            clipped_intersections.extend(segment_intersections_with_plane_edges)

        if True not in point_in_plane_arr:
            return []
        elif len(clipped_intersections) == 0:
            return points

        '''sets all the ranges of angles that do not contain points that are out of bounds of the clipping plane'''
        in_bounds_angle_ranges = self.get_angle_ranges(unclipped_points_plane, points, point_in_plane_arr)

        point_index = 0
        while point_index < len(points):
            point_in_bounds = plane.point_lies_in_bounded_plane(points[point_index])
            if point_in_bounds:
                point_index += 1
            else:
                del points[point_index]

        '''intersections are added to the points'''
        points.extend(clipped_intersections)

        '''all points in the clipping plane that are contained by this plane are added to fill the holes where points were deleted
        (i.e. if there was a corner that clipped the polygon, it would have to clip the polygon to that corner)'''
        for i in range(0, len(plane.bounding_points)):
            if unclipped_points_plane.point_lies_in_bounded_plane(plane.bounding_points[i]):
                points.append(plane.bounding_points[i])

        '''is a poly plane object that is created from the already clipped points only.
        May have to be instantiated earlier so that the appended, unsorted points do not cause it to fail?'''
        if len(points) > 2:
            clipped_points_plane = PolyPlane(points)
            '''sorts the points by order of angle relative to the clipped points plane basises'''
            points.sort(key = lambda point: clipped_points_plane.get_angle_to_coplanar_point_from_first_basis(point))
            return points
        return None


    def get_angle_ranges(self, unclipped_points_plane, points, point_in_plane_arr):
        in_bounds_angle_ranges = []
        print("point_in_plane_arr: ", point_in_plane_arr)
        for i in range(0, len(point_in_plane_arr)):
            prev_point_in_plane = point_in_plane_arr[i-1]
            this_point_in_plane = point_in_plane_arr[i]

            if not prev_point_in_plane and this_point_in_plane:
                '''finds the first case where a segment goes from being out of bounds to being in bounds.
                Next it searches the whole arr for a case where the array is False again, signifying the end
                of the angle range'''
                end_segment_index = i
                angle_to_start_point = unclipped_points_plane.get_angle_to_coplanar_point_from_first_basis(points[i])

                for j in range(i, len(point_in_plane_arr)+i):
                    j_index = j%len(point_in_plane_arr)
                    if not point_in_plane_arr[j_index]:
                        end_segment_index = j_index-1
                        break
                angle_to_end_point = unclipped_points_plane.get_angle_to_coplanar_point_from_first_basis(points[end_segment_index])
                append_range = (angle_to_start_point, angle_to_end_point)
                in_bounds_angle_ranges.append(append_range)
        return in_bounds_angle_ranges

    '''removal by angle range possibly not working as well as just testing for in-bound-ness'''
    def remove_points_outside_of_angle_ranges(self, unclipped_points_plane, points, in_bounds_angle_ranges):
        point_index = 0
        while point_index < len(points):
            angle_to_point = unclipped_points_plane.get_angle_to_coplanar_point_from_first_basis(points[point_index])
            angle_in_range = AngleMath.angle_in_angle_ranges(in_bounds_angle_ranges, angle_to_point)
            if angle_in_range:
                point_index += 1
            else:
                del points[point_index]
        return None
