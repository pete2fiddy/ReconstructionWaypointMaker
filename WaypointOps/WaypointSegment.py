import numpy
import Geometry.LineMath as LineMath

class WaypointSegment:
    '''takes a length 2 list of cartesian waypoints, and the WaypointPathPolyPlane that it lies on'''
    def __init__(self, segment, path_plane):
        self.segment = segment
        self.path_plane = path_plane

    '''takes this waypoint segment and fits it around all obstacles that it would collide with on its plane, then returns
    them as a list of waypoint segments'''
    def fit_to_obstacles(self):
        '''holds all the sliced polygons of the obstacles on the plane that this segment is on'''
        obstacle_slice_planes = self.path_plane.slicing_poly_planes
        path_intersects_obstacles = False
        segment_intersections = []
        for i in range(0, len(obstacle_slice_planes)):
            path_intersections_with_obstacle = obstacle_slice_planes[i].get_segment_intersections_with_edges(self.segment, .01)

            obstacle_segments = [[obstacle_slice_planes[i].bounding_points[j-1], obstacle_slice_planes[i].bounding_points[j]] for j in range(0, len(obstacle_slice_planes[i].bounding_points))]


            if len(path_intersections_with_obstacle) > 0:
                path_intersects_obstacles = True
                segment_intersections.extend(path_intersections_with_obstacle)

        if len(segment_intersections) > 0:
            segment_intersections.sort(key = lambda intersection : numpy.linalg.norm(intersection - self.segment[0]))
            out_segments = []
            for i in range(1, len(segment_intersections)):
                append_segment = [segment_intersections[i-1], segment_intersections[i]]
                append_waypoint_segment = WaypointSegment(append_segment, self.path_plane)
                out_segments.append(append_waypoint_segment)

            first_segment = WaypointSegment([self.segment[0], segment_intersections[0]], self.path_plane)
            last_segment = WaypointSegment([segment_intersections[len(segment_intersections)-1], self.segment[len(self.segment)-1]], self.path_plane)
            out_segments.insert(0, first_segment)
            out_segments.append(last_segment)
            return out_segments
            print("segment intersections: ", segment_intersections)
        return [self]




    def copy(self):
        '''not sure if just copying the list object removes the references of the numpy vectors inside to each other.
        The below is to circumvent that if that is an issue'''
        copied_segments = []
        for i in range(0, len(self.segments)):
            copied_segments.append(self.segment[i].copy())
        '''does not clone the path plane, as those SHOULD be linked'''
        return WaypointSegment(copied_segments, self.path_plane)
