from abc import ABC, abstractmethod
import GeoOps.GeoMath as GeoMath
import os
import numpy
import Geometry.LineMath as LineMath

class WaypointBuilder(ABC):
    '''is abstract so cannot be instantiated other than through subclasses'''
    DEFAULT_OBSTACLE_SLICE_RESOLUTION = 32

    def __init__(self, obstacles, GEO_ORIGIN):
        self.waypoint_segments = self.create_waypoint_segments()
        self.obstacles = obstacles
        '''GEO_ORIGIN is in caps to signify that it should NEVER be edited'''
        self.GEO_ORIGIN = GEO_ORIGIN
        self.obstacles.init_obstacle_cartesian_points(self.GEO_ORIGIN)
        self.init_obstacle_intersections_with_planes()
        self.waypoint_segments.avoid_obstacles()
        '''placeholder variables to add in eventually:
        self.total_path: is a variable set by the subclasses.
        '''

    '''treats the entire path as a vector valued function and checks individual points to see if they are inside of an obstacle
    by a certain margin. check_resolution determines how often a point is "created" and checked, in meters'''
    def path_is_safe(self, check_resolution, safety_radius):
        '''may not be working but can't figure out why'''
        num_path_checks = 0
        for i in range(0, len(self.waypoint_segments)):
            iter_segment = self.waypoint_segments[i].segment
            mag_iter_segment = numpy.linalg.norm(iter_segment[1] - iter_segment[0])
            num_separations = int(mag_iter_segment/check_resolution)

            if num_separations != 0:
                t_step = 1.0/float(num_separations)


                t = 0
                while t < 1:
                    point_at_t = LineMath.point_of_segment_at_t(iter_segment, t)
                    for obstacle_index in range(0, len(self.obstacles)):
                        if self.obstacles[obstacle_index].point_in_obstacle(point_at_t, safety_radius):
                            return False
                    t += t_step
                    num_path_checks += 1
            else:
                print("num separations = 0")
        print("len waypoint segments: ", len(self.waypoint_segments))
        print("num path checks: ", num_path_checks)
        return True



    '''should have methods that support taking a path that is made with no regard to obstacle collisions and alter
    the path so that it does not collide with any obstacles.'''

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
                intersections_with_plane = self.obstacles[j].intersect_with_plane(segment_planes[i], WaypointBuilder.DEFAULT_OBSTACLE_SLICE_RESOLUTION, bounded = True)

                if intersections_with_plane != None and len(intersections_with_plane) > 2:
                    segment_planes[i].add_polygon_slice(self.obstacles[j], intersections_with_plane)

    DRONE_PATH_EXTENSION = "/path"
    OBSTACLE_EXTENSION = "/obstacles"
    SLICE_EXTENSION = "/plane_slices"
    PATH_PLANE_EXTENSION = "/path_planes"

    def save(self, dir, name):
        if not os.path.exists(dir + "/" + name):
            os.makedirs(dir + "/" + name)
        path_dir = dir + "/" + name + WaypointBuilder.DRONE_PATH_EXTENSION + ".txt"
        obstacle_dir = dir + "/" + name + WaypointBuilder.OBSTACLE_EXTENSION + ".txt"
        slice_dir = dir + "/" + name + WaypointBuilder.SLICE_EXTENSION + ".txt"
        path_plane_dir = dir + "/" + name + WaypointBuilder.PATH_PLANE_EXTENSION + ".txt"

        with open(path_dir, 'w') as path_output:
            path_output.write(self.waypoint_segments.save_str())
            path_output.close()

        with open(obstacle_dir, 'w') as obstacle_output:
            write_str = ""
            for i in range(0, len(self.obstacles)):
                obstacle_points = self.obstacles[i].obstacle_points()
                for j in range(0, len(obstacle_points)):
                    write_str += str(obstacle_points[j].tolist())

                if i < len(self.obstacles) - 1:
                    write_str += "\n"
            obstacle_output.write(write_str)
            obstacle_output.close()

        with open(slice_dir, 'w') as slice_output:
            write_str = ""
            for i in range(0, len(self.waypoint_segments.segment_planes)):
                slices_on_plane = self.waypoint_segments.segment_planes[i].get_obstacle_slices()
                for j in range(0, len(slices_on_plane)):
                    bounding_points = slices_on_plane[j].bounding_points
                    for k in range(0, len(bounding_points)):
                        write_str += str(bounding_points[k].tolist())

                    if j < len(self.waypoint_segments.segment_planes[i].slice_obstacle_shapes) - 1:
                        write_str += "\n"

                if i < len(self.waypoint_segments.segment_planes)-1:
                    write_str += "\n"
            slice_output.write(write_str)
            slice_output.close()

        with open(path_plane_dir, 'w') as plane_output:
            write_str = ""
            for i in range(0, len(self.waypoint_segments.segment_planes)):
                bounding_points = self.waypoint_segments.segment_planes[i].bounding_points
                for j in range(0, len(bounding_points)):
                    write_str += str(bounding_points[j].tolist())
                if i < len(self.waypoint_segments.segment_planes) - 1:
                    write_str += "\n"
            plane_output.write(write_str)
            plane_output.close()
