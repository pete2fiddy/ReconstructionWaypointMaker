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
        obstacle_slice_planes = self.path_plane.get_obstacle_slices()
        path_intersects_obstacles = False
        '''2d array where the first index indicates the index of the slicing obstacle object index, and
        the second index gets you a given point that intersects that slicing obstacle'''
        segment_intersections = [[] for i in range(0, len(self.path_plane.slice_obstacle_shapes))]
        for i in range(0, len(obstacle_slice_planes)):
            path_intersections_with_obstacle = obstacle_slice_planes[i].get_segment_intersections_with_edges(self.segment, .01)

            if len(path_intersections_with_obstacle) > 0:
                path_intersects_obstacles = True
                segment_intersections[i].extend(path_intersections_with_obstacle)


        labeled_path_intersections = []
        for i in range(0, len(segment_intersections)):
            for j in range(0, len(segment_intersections[i])):
                labeled_path_intersections.append((segment_intersections[i][j], i))


        labeled_path_intersections.sort(key = lambda labeled_path_intersection: numpy.linalg.norm(labeled_path_intersection[0] - self.segment[0]))

        self.remove_neighboring_points_by_minimum_distance(labeled_path_intersections, 0.01)


        if len(labeled_path_intersections) <=1:
            return [self]

        '''for testing, just adds the intersections as points so you can see if it intersects correctly'''
        '''out_segments = [WaypointSegment([labeled_path_intersections[i-1][0], labeled_path_intersections[i][0]], self.path_plane) for i in range(1, len(labeled_path_intersections))]

        out_segments.insert(0, WaypointSegment([self.segment[0], out_segments[0].segment[0]], self.path_plane))
        out_segments.append(WaypointSegment([out_segments[len(out_segments)-1].segment[1],self.segment[1]], self.path_plane))
        return out_segments'''

        temp_labeled_path_intersections = []

        for i in range(1, len(labeled_path_intersections)):
            if labeled_path_intersections[i][1] == labeled_path_intersections[i-1][1]:
                iter_waypoint_segment = [labeled_path_intersections[i-1][0], labeled_path_intersections[i][0]]
                iter_obstacle_slice_shape = self.path_plane.slice_obstacle_shapes[labeled_path_intersections[i-1][1]]
                iter_path_around = iter_obstacle_slice_shape.path_around_obstacle(self, iter_waypoint_segment)
                for j in range(0, len(iter_path_around)):
                    temp_labeled_path_intersections.append((iter_path_around[j].segment[0], labeled_path_intersections[i-1][1]))
            else:
                temp_labeled_path_intersections.append(labeled_path_intersections[i])
            #temp_labeled_path_intersections.append(labeled_path_intersections[i])
            #temp_labeled_path_intersections.append((obstacle_slice_planes[labeled_path_intersections[i][1]].bounding_points[0], labeled_path_intersections[i][1]))

        labeled_path_intersections = temp_labeled_path_intersections

        '''for now, assumes that if a point enters a polygon, that its next intersection must be one that EXITS that same polygon
        This will not be the case until I add a polygon addition algorithm that will merge intersecting polygons'''


        out_points = [labeled_path_intersections[i][0] for i in range(0, len(labeled_path_intersections))]

        '''has to check if the first and last points are safe to append or insert. If they are inside of an obstacle, they cannot be
        inserted'''
        POINT_IN_OBSTACLE_SAFETY_MARGIN = 0.02
        segment1_in_bounds = True
        segment2_in_bounds = True
        for i in range(0, len(self.path_plane.slice_obstacle_shapes)):
            iter_slice_obstacle = self.path_plane.slice_obstacle_shapes[i].obstacle
            if iter_slice_obstacle.point_in_obstacle(self.segment[0], POINT_IN_OBSTACLE_SAFETY_MARGIN):
                segment1_in_bounds = False
            if iter_slice_obstacle.point_in_obstacle(self.segment[1], POINT_IN_OBSTACLE_SAFETY_MARGIN):
                segment2_in_bounds = False

        if segment1_in_bounds:
            out_points.insert(0, self.segment[0])
        if segment2_in_bounds:
            out_points.append(self.segment[1])


        out_segments = [WaypointSegment([out_points[i-1], out_points[i]], self.path_plane) for i in range(1, len(out_points))]



        return out_segments






    '''sometimes the same intersection is found more than once. This is used to remove those instances'''
    def remove_neighboring_points_by_minimum_distance(self, points, min_distance):
        i = 0
        while i < len(points)-1:

            p1 = points[i][0]
            p2 = points[i+1][0]
            if numpy.linalg.norm(p2-p1) < min_distance:
                points[i] = (((p1+p2)/2.0), points[i][1])
                del points[(i+1)]
            else:
                i+=1
        return None



    def copy(self):
        '''not sure if just copying the list object removes the references of the numpy vectors inside to each other.
        The below is to circumvent that if that is an issue'''
        copied_segments = []
        for i in range(0, len(self.segments)):
            copied_segments.append(self.segment[i].copy())
        '''does not clone the path plane, as those SHOULD be linked'''
        return WaypointSegment(copied_segments, self.path_plane)
