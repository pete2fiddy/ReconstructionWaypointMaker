from abc import ABC, abstractmethod
import GeoOps.GeoMath as GeoMath
import os

class WaypointBuilder2(ABC):
    '''is abstract so cannot be instantiated other than through subclasses'''
    def __init__(self, obstacles, GEO_ORIGIN):
        self.waypoint_segments = self.create_waypoint_segments()
        self.obstacles = obstacles
        '''GEO_ORIGIN is in caps to signify that it should NEVER be edited'''
        self.GEO_ORIGIN = GEO_ORIGIN
        self.obstacles.init_obstacle_cartesian_points(self.GEO_ORIGIN)
        self.init_obstacle_intersections_with_planes()
        '''placeholder variables to add in eventually:
        self.total_path: is a variable set by the subclasses.
        '''

    '''should have methods that support taking a path that is made with no regard to obstacle collisions and alter
    the path so that it does not collide witha ny obstacles.'''

    @abstractmethod
    def create_waypoint_segments(self):
        pass

    def geos_to_points(self, geopoints, GEO_ORIGIN):
        points_from_geos = [self.geo_to_point(geopoints[i], GEO_ORIGIN) for i in range(0, len(geopoints))]
        return points_from_geos

    '''converts the geopoints to cartesian relative to the origin GPS location, using meters'''
    def geo_to_point(self, geopoint, GEO_ORIGIN):
        return GeoMath.vector_between_geo_points(GEO_ORIGIN, geopoint) * 1000.0

    def init_obstacle_intersections_with_planes(self):
        segment_planes = self.waypoint_segments.segment_planes

        for i in range(0, len(segment_planes)):
            for j in range(0, len(self.obstacles)):
                intersections_with_plane = self.obstacles[j].unbounded_intersect_with_plane(segment_planes[i], 32)
                if intersections_with_plane != None and len(intersections_with_plane) > 2:
                    segment_planes[i].add_polygon_slice(intersections_with_plane)


    DRONE_PATH_EXTENSION = "/path"
    OBSTACLE_EXTENSION = "/obstacles"
    SLICE_EXTENSION = "/plane_slices"

    def save(self, dir, name):
        if not os.path.exists(dir + "/" + name):
            os.makedirs(dir + "/" + name)
        path_dir = dir + "/" + name + WaypointBuilder2.DRONE_PATH_EXTENSION + ".txt"
        obstacle_dir = dir + "/" + name + WaypointBuilder2.OBSTACLE_EXTENSION + ".txt"
        slice_dir = dir + "/" + name + WaypointBuilder2.SLICE_EXTENSION + ".txt"

        with open(path_dir, 'w') as path_output:
            path_output.write(self.waypoint_segments.save_str())
            path_output.close()

        '''obstacles and slices not currently implemented'''
        with open(obstacle_dir, 'w') as obstacle_output:
            write_str = ""
            for i in range(0, len(self.obstacles)):
                write_str += str(self.obstacles[i].obstacle_points())
                if i < len(self.obstacles) - 1:
                    write_str += "\n"
            obstacle_output.write(write_str)
            obstacle_output.close()

        with open(slice_dir, 'w') as slice_output:
            write_str = ""
            for i in range(0, len(self.waypoint_segments.segment_planes)):
                for j in range(0, len(self.waypoint_segments.segment_planes[i].slicing_poly_planes)):
                    write_str += str(self.waypoint_segments.segment_planes[i].slicing_poly_planes[j].bounding_points)
                    if j < len(self.waypoint_segments.segment_planes[i].slicing_poly_planes) - 1:
                        write_str += "\n"
                if i < len(self.waypoint_segments.segment_planes)-1:
                    write_str += "\n"
            slice_output.write(write_str)
            slice_output.close()
