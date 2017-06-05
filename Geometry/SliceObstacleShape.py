import numpy
from WaypointOps.WaypointSegment import WaypointSegment
from Geometry.PolyPlane import PolyPlane
import Geometry.LineMath as LineMath
import VectorOps.VectorMath as VectorMath
from math import exp

class SliceObstacleShape:

    def __init__(self, obstacle, obstacle_slice_polyplane):
        self.obstacle = obstacle
        self.obstacle_slice_polyplane = obstacle_slice_polyplane

        self.init_total_polyplane_distance()

    def init_total_polyplane_distance(self):
        self.total_slice_distance = 0
        for i in range(0, len(self.obstacle_slice_polyplane.bounding_points)):
            self.total_slice_distance += numpy.linalg.norm(self.obstacle_slice_polyplane.bounding_points[i] - self.obstacle_slice_polyplane.bounding_points[i-1])


    def path_around_obstacle(self, waypoint_segment, intersection_points):
        '''out_segments = []
        out_segments.append(WaypointSegment([waypoint_segment.segment[0], intersection_points[0]], waypoint_segment.path_plane))
        out_segments.append(WaypointSegment([intersection_points[0], intersection_points[1]], waypoint_segment.path_plane))
        out_segments.append(WaypointSegment([intersection_points[1], waypoint_segment.segment[1]], waypoint_segment.path_plane))
        return out_segments'''

        print("--------------------------------")
        bounding_points = self.obstacle_slice_polyplane.bounding_points
        '''finds the possible the two best indexes to be taken given the two intersection points.
        Two are required as the direction that the path is taken will determine
        which of the two need to be used.'''
        waypoint_segment_vector = waypoint_segment.segment[1] - waypoint_segment.segment[0]
        intersection_vector = intersection_points[1] - intersection_points[0]
        mag_intersection_vector = numpy.linalg.norm(intersection_vector)
        intersection_unit_vector = intersection_vector/mag_intersection_vector
        line_normal = numpy.cross(waypoint_segment_vector, self.obstacle_slice_polyplane.unit_normal)
        line_normal /= numpy.linalg.norm(line_normal)

        above_points = []
        below_points = []
        for i in range(0, len(bounding_points)):
            dot_with_normal = VectorMath.scalar_proj(bounding_points[i]-waypoint_segment.segment[0], line_normal)
            #print("dot with normal: ", dot_with_normal)
            if dot_with_normal > 0:#0.3:
                above_points.append(bounding_points[i])
            if dot_with_normal < 0:
                below_points.append(bounding_points[i])

        points_index = 0
        while points_index < len(above_points):
            dot_with_first_intersect = VectorMath.scalar_proj(above_points[points_index] - intersection_points[0], intersection_unit_vector)
            if dot_with_first_intersect < 0:
                del above_points[points_index]
            else:
                points_index += 1

        points_index = 0
        while points_index < len(below_points):
            dot_with_first_intersect = VectorMath.scalar_proj(below_points[points_index] - intersection_points[0], intersection_unit_vector)
            if dot_with_first_intersect < 0:
                del below_points[points_index]
            else:
                points_index += 1

        if len(above_points) < 1:
            return [waypoint_segment]


        #below_points.sort(key = lambda point: VectorMath.dot_angle_between(point - intersection_points[0], -line_normal))
        #above_points.sort(key = lambda point: VectorMath.dot_angle_between(point - intersection_points[0], line_normal))

        new_below_points = [intersection_points[0]]
        below_points_copy = list(below_points)

        while len(below_points_copy) > 0:
            compare_point = new_below_points[len(new_below_points)-1]
            smallest_distance = numpy.linalg.norm(below_points_copy[0] - compare_point)
            smallest_distance_index = 0
            for i in range(1, len(below_points_copy)):
                dist_to_new_point = numpy.linalg.norm(below_points_copy[i] - compare_point)
                if dist_to_new_point < smallest_distance:
                    smallest_distance = dist_to_new_point
                    smallest_distance_index = i

            new_below_points.append(below_points_copy[smallest_distance_index])
            del below_points_copy[smallest_distance_index]

        new_above_points = []

        start_segment_indexes = self.segment_indexes_point_lies_on(intersection_points[0])
        dot_first_point = VectorMath.scalar_proj(bounding_points[start_segment_indexes[0]]-waypoint_segment.segment[0], line_normal)
        dot_second_point = VectorMath.scalar_proj(bounding_points[start_segment_indexes[1]]-waypoint_segment.segment[0], line_normal)
        if dot_first_point > 0:
            new_above_points.append(bounding_points[start_segment_indexes[0]])
        else:
            new_above_points.append(bounding_points[start_segment_indexes[1]])

        above_points = self.create_safe_path_given_points(True, line_normal, above_points, waypoint_segment, intersection_points)
        below_points = self.create_safe_path_given_points(False, line_normal, below_points, waypoint_segment, intersection_points)

        '''
        above_points_copy = list(above_points)

        SAFETY_MARGIN = 0.05
        while len(above_points_copy) > 0:
            compare_point = new_above_points[len(new_above_points)-1]
            smallest_distance = None#numpy.linalg.norm(above_points_copy[0] - compare_point)
            smallest_distance_index = None
            for i in range(0, len(above_points_copy)):
                dist_to_new_point = numpy.linalg.norm(above_points_copy[i] - compare_point)
                segment = [compare_point, above_points_copy[i]]
                segment_midpoint = (segment[0]+segment[1])/2.0
                if not self.obstacle.point_in_obstacle(segment_midpoint, SAFETY_MARGIN):
                    if smallest_distance == None:
                        smallest_distance = dist_to_new_point
                        smallest_distance_index = i
                    elif dist_to_new_point < smallest_distance:
                        smallest_distance = dist_to_new_point
                        smallest_distance_index = i


            if smallest_distance_index != None:
                new_above_points.append(above_points_copy[smallest_distance_index])
                del above_points_copy[smallest_distance_index]
            else:
                break

        below_points = list(new_below_points)
        above_points = list(new_above_points)'''




        #below_points.sort(key = lambda point: VectorMath.dot_angle_between(point - intersection_points[0], -line_normal))

        '''needs to sort by angle to origin'''

        '''above_points.sort(key = lambda point : VectorMath.scalar_proj(point - waypoint_segment.segment[0], waypoint_segment_vector))
        below_points.sort(key = lambda point : VectorMath.scalar_proj(point - waypoint_segment.segment[0], waypoint_segment_vector))
        '''



        path_points = intersection_points[0:2]#[intersection_points[0], intersection_points[1]]
        if above_points != None and below_points != None:
            dist_to_above = numpy.linalg.norm(intersection_points[0] - above_points[0])
            dist_to_below = numpy.linalg.norm(intersection_points[0] - below_points[0])
            path_points = above_points if dist_to_above < dist_to_below else below_points
        elif above_points != None:
            path_points = above_points
        elif below_points != None:
            path_points = below_points
        else:
            print("path points never changed")
            return [waypoint_segment]
            
        out_segments = [WaypointSegment([path_points[i-1], path_points[i]], waypoint_segment.path_plane) for i in range(1, len(path_points))]


        '''out_segments.append(WaypointSegment([waypoint_segment.segment[0], intersection_points[0]], waypoint_segment.path_plane))
        out_segments.append(WaypointSegment([intersection_points[0], intersection_points[1]], waypoint_segment.path_plane))
        out_segments.append(WaypointSegment([intersection_points[1], waypoint_segment.segment[1]], waypoint_segment.path_plane))'''
        print("--------------------------------")
        return out_segments


    def create_safe_path_given_points(self, is_above, line_normal, points, waypoint_segment, intersection_points, INTERSECTION_SAFETY_MARGIN = 0.05):
        points_copy = list(points)
        out_points = []
        bounding_points = self.obstacle_slice_polyplane.bounding_points

        start_segment_indexes = self.segment_indexes_point_lies_on(intersection_points[0])
        dot_first_point = VectorMath.scalar_proj(bounding_points[start_segment_indexes[0]]-waypoint_segment.segment[0], line_normal)
        #dot_second_point = VectorMath.scalar_proj(bounding_points[start_segment_indexes[1]]-waypoint_segment.segment[0], line_normal)
        if is_above:
            if dot_first_point > 0:
                out_points.append(bounding_points[start_segment_indexes[0]])
            else:
                out_points.append(bounding_points[start_segment_indexes[1]])
        else:
            if dot_first_point < 0:
                out_points.append(bounding_points[start_segment_indexes[0]])
            else:
                out_points.append(bounding_points[start_segment_indexes[1]])


        SAFETY_MARGIN = 0.05
        while len(points_copy) > 0:
            compare_point = out_points[len(out_points)-1]
            smallest_distance = None
            smallest_distance_index = None
            for i in range(0, len(points_copy)):
                dist_to_new_point = numpy.linalg.norm(points_copy[i] - compare_point)
                segment = [compare_point, points_copy[i]]
                segment_midpoint = (segment[0]+segment[1])/2.0
                if not self.obstacle.point_in_obstacle(segment_midpoint, SAFETY_MARGIN):
                    if smallest_distance == None:
                        smallest_distance = dist_to_new_point
                        smallest_distance_index = i
                    elif dist_to_new_point < smallest_distance:
                        smallest_distance = dist_to_new_point
                        smallest_distance_index = i
            if smallest_distance_index != None:
                out_points.append(points_copy[smallest_distance_index])
                del points_copy[smallest_distance_index]
            else:
                '''path not safe'''
                return None

        return out_points


    def get_unblocked_step_from_start_to_end_indexes(self, start_index, end_index_plus_step, end_index_minus_step, safety_margin):
        '''cycles around the polygon from start to end index, both backwards and forward to see which route will not cause a
        collision. It checks for a collision by taking each segment from point to point's midpoint and seeing if that intersects the
        obstacle. This is fairly crude, but given the simplicity of the obstacle's shapes, it should work'''
        '''first checks the +1 step to see if there are any collisions with the obstacle.'''
        plus_step_possible = self.get_if_step_from_start_to_end_indexes_collides(start_index, end_index_plus_step, 1, safety_margin)
        minus_step_possible = self.get_if_step_from_start_to_end_indexes_collides(start_index, end_index_minus_step, -1, safety_margin)
        '''should pick the shorter of the two paths if both are possible'''
        if not plus_step_possible and not minus_step_possible:
            print("-------------------------")
            print("neither path is possible")
            #print("start index: ", start_index, ", end index: ", end_index)
            print('my bounding points: ', self.obstacle_slice_polyplane.bounding_points)
            print("-------------------------")
        if plus_step_possible:
            return 1
        elif minus_step_possible:
            return -1
        return None



    def get_if_step_from_start_to_end_indexes_collides(self, start_index, end_index, step, safety_margin):
        bounding_points = self.obstacle_slice_polyplane.bounding_points
        end_found = False
        path_open = False
        i = start_index
        while not end_found:
            iter_segment = (bounding_points[i], bounding_points[(i+step)%len(bounding_points)])
            iter_segment_midpoint = (iter_segment[0] + iter_segment[1])/2.0
            midpoint_in_obstacle = self.obstacle.point_in_obstacle(iter_segment_midpoint, safety_margin)
            if i == end_index:
                path_open = True
                end_found = True
            if midpoint_in_obstacle:
                end_found = True
            if i >= len(bounding_points) - 1 and step > 0:
                i = 0
            elif i <= 0 and step < 0:
                i = len(bounding_points) - 1
            else:
                i += step
        return path_open

    def path_from_index_to_index_with_step(self, start_index, end_index, step):
        bounding_points = self.obstacle_slice_polyplane.bounding_points
        end_found = False
        i = start_index
        out_point_path = []
        indexes = []
        while not end_found:
            out_point_path.append(bounding_points[i])
            indexes.append(i)
            if i == end_index:
                end_found = True
            if i >= len(bounding_points) - 1 and step > 0:
                i = 0
            elif i <= 0 and step < 0:
                i = len(bounding_points) - 1
            else:
                i += step
        return out_point_path, indexes


    def segment_indexes_point_lies_on(self, point, max_distance_for_intersection = None):
        bounding_points = self.obstacle_slice_polyplane.bounding_points
        smallest_dist = None
        smallest_dist_segment_indexes = None
        for i in range(0, len(bounding_points)):
            i_index = (i+1)%len(bounding_points)
            iter_bounding_segment = [bounding_points[i], bounding_points[i_index]]
            dist_to_iter_bounding_segment = LineMath.distance_from_segment_to_point(iter_bounding_segment, point, bounded = True)
            if dist_to_iter_bounding_segment != None and (smallest_dist == None or dist_to_iter_bounding_segment < smallest_dist):
                smallest_dist = dist_to_iter_bounding_segment
                smallest_dist_segment_indexes = (i, i_index)
        if max_distance_for_intersection == None:
            return smallest_dist_segment_indexes
        if smallest_dist < max_distance_for_intersection:
            return smallest_dist_segment_indexes
        return None

    def index_closest_to_point(self, point):
        indexes = [i for i in range(0, len(self.obstacle_slice_polyplane.bounding_points))]
        indexes_sorted_by_distance_to_point = sorted(indexes, key = lambda index: numpy.linalg.norm(point - self.obstacle_slice_polyplane.bounding_points[index]))
        return indexes_sorted_by_distance_to_point[0]

    def index_closest_to_point_outside_of_obstacle(self, point, safety_margin):
        indexes_outside_of_obstacle = []
        bounding_points = self.obstacle_slice_polyplane.bounding_points
        for i in range(0, len(bounding_points)):
            if not self.obstacle.point_in_obstacle(bounding_points[i], safety_margin):
                indexes_outside_of_obstacle.append(i)
        indexes_outside_of_obstacle.sort(key = lambda index: numpy.linalg.norm(bounding_points[index] - point))
        return indexes_outside_of_obstacle[0]
