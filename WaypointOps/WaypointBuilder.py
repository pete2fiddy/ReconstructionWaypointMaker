import GeoOps.GeoMath as GeoMath
import numpy
from Geometry.Prism import Prism
import VectorOps.VectorMath as VectorMath
from Geometry.FlatPolygon import FlatPolygon
from Geometry.PolyPlane import PolyPlane
from Geometry.WaypointPathPolyPlane import WaypointPathPolyPlane
import os

class WaypointBuilder:
    '''number of z meters per xy meter'''
    DEFAULT_ALTITUDE_ROC = 1.0/50.0

    '''number of meters between waypoints, assuming a corner isn't hit beforehand'''
    DEFAULT_WAYPOINT_PLACEMENT_DISTANCE = 5.0

    '''the minimum and maximum distance required to be between two waypoints'''
    DEFAULT_WAYPOINT_DISTANCE_THRESHOLDS = (1.0, 100.0)

    '''the number of meters y changes by for each scan line of floodfill'''
    DEFAULT_TOP_FILL_SPACING = 0.25

    '''the default resoution of the intersection between a plane and an obstacle -- determines how "jagged" the path taken
    to avoid the obstacle is'''
    DEFAULT_OBSTACLE_AVOIDANCE_RESOLUTION = 32

    '''Current problems:
    obstacles placed at corners cannot be traversed. I do not think this can be fixed on thefly as it flattens the obstacle
    down into a slice onto each geo plane, and loses all information about the depth. (i.e. it can only dodge up and down,
    not in the direction of the normal). To fix this, pre-process the bounding points and add points in 2d that will dodge
    the obstacles in all places that it needs to? Possible issue in that it could breach the geo fence too far inwards...

    floodfilling the top can go slightly out of bounds when it switches from the first to second set of segments.

    may intersect THROUGH an obstacle. Have only ever seen this once -- the ray casted through the obstacle was roughly
    through its center. It had a start point at one of the edges of the obstacle, but never found an end point (which
    I assume is the reason it failed)

    should throw an error after a path is generated that checks to see if the lines intersect any of the obstacles
    by a significant margin in 3d space, and not let the program run if they do.

    '''


    '''
    bounding geo points are all at a level altitude. The first waypoint is placed directly at the drone's position.
    Max altitude is in meters.
    '''
    def __init__(self, bounding_geo_points, obstacles, max_altitude, top_fill_method, altitude_roc = None, dist_between_waypoints = None, waypoint_distance_thresholds = None, top_fill_spacing = None):
        self.bounding_geo_points = bounding_geo_points
        self.obstacles = obstacles
        self.altitude_roc = WaypointBuilder.DEFAULT_ALTITUDE_ROC if altitude_roc == None else altitude_roc
        self.dist_between_waypoints = WaypointBuilder.DEFAULT_WAYPOINT_PLACEMENT_DISTANCE if dist_between_waypoints == None else dist_between_waypoints
        self.waypoint_distance_thresholds = WaypointBuilder.DEFAULT_WAYPOINT_DISTANCE_THRESHOLDS if waypoint_distance_thresholds == None else waypoint_distance_thresholds
        self.top_fill_spacing = WaypointBuilder.DEFAULT_TOP_FILL_SPACING if top_fill_spacing == None else top_fill_spacing
        self.max_altitude = max_altitude
        self.init_bounds()
        self.init_geo_planes()
        self.init_obstacle_shapes()
        self.init_plane_intersections()
        self.init_bounds_prism()
        self.total_path = []
        self.init_spiral_points()
        self.top_waypoints = top_fill_method(self, self.point_bounds)
        self.total_path.extend(self.top_waypoints)

    def init_bounds(self):
        self.geo_bounds_origin = GeoMath.get_avg_geo_point(self.bounding_geo_points)
        print("geo bounds origin: ", self.geo_bounds_origin)
        self.origin_z = self.geo_bounds_origin.alt * 1000.0
        '''
        point_bounds is in meters
        '''
        self.point_bounds = numpy.zeros((len(self.bounding_geo_points), 3))
        for i in range(0, self.point_bounds.shape[0]):
            '''
            *1000.0 to convert to meters
            '''
            self.point_bounds[i] = GeoMath.vector_between_geo_points(self.geo_bounds_origin, self.bounding_geo_points[i]) * 1000.0

        print("self.point_bounds is: ", self.point_bounds)

    def init_geo_planes(self):
        self.waypoint_path_poly_planes = []
        '''the poly plane at index i is the plane that is made up of the geo points i-1 and i'''
        for i in range(0, len(self.point_bounds)):
            p1 = self.point_bounds[i-1]
            p2 = self.point_bounds[i]
            p3 = p2.copy()
            p3[2] = self.max_altitude
            p4 = p1.copy()
            p4[2] = self.max_altitude
            iter_points = [p1, p2, p3, p4]
            iter_poly_plane = WaypointPathPolyPlane(iter_points)
            self.waypoint_path_poly_planes.append(iter_poly_plane)

    def init_obstacle_shapes(self):
        for i in range(0, len(self.obstacles)):
            self.obstacles[i].init_shape(self.geo_bounds_origin)

    def init_plane_intersections(self):
        self.plane_intersection_slices = []
        for i in range(0, len(self.waypoint_path_poly_planes)):
            iter_plane_intersection_points = []
            for j in range(0, len(self.obstacles)):
                iter_shape = self.obstacles[j].shape
                iter_shape_intersections = iter_shape.get_intersection_points_with_plane(self.waypoint_path_poly_planes[i], WaypointBuilder.DEFAULT_OBSTACLE_AVOIDANCE_RESOLUTION)

                if not iter_shape_intersections == None and len(iter_shape_intersections) > 2:
                    iter_plane_intersection_points.append(iter_shape_intersections)
                    self.plane_intersection_slices.append(iter_shape_intersections)
            if len(iter_plane_intersection_points) > 0:
                for j in range(0, len(iter_plane_intersection_points)):
                    self.waypoint_path_poly_planes[i].add_polygon_slice(iter_plane_intersection_points[j])


    def init_bounds_prism(self):
        bounding_rect = VectorMath.get_flat_bounds_of_vectors(self.point_bounds)
        self.bounding_prism = Prism.init_with_rect(bounding_rect, self.max_altitude)

    def init_spiral_points(self):
        self.spiral_points = []
        drone_xyz = self.point_bounds[0].copy();
        waypoint_index = 1
        self.spiral_points.append(self.point_bounds[0].copy())
        while drone_xyz[2] < self.max_altitude:

            dist_to_next_waypoint = numpy.linalg.norm(self.point_bounds[waypoint_index][:2] - drone_xyz[:2])

            if dist_to_next_waypoint < self.dist_between_waypoints:
                dz = dist_to_next_waypoint * self.altitude_roc
                drone_xyz = numpy.array([self.point_bounds[waypoint_index, 0], self.point_bounds[waypoint_index, 1], drone_xyz[2] + dz])

                if dist_to_next_waypoint < self.waypoint_distance_thresholds[0]:
                    '''
                    means that the distance from the previous point to the new one is too small, and that the two must be merged
                    '''
                    del self.spiral_points[len(self.spiral_points)-1]

                if waypoint_index >= self.point_bounds.shape[0]-1:
                    waypoint_index = 0
                else:
                    waypoint_index += 1
            else:
                vector_to_waypoint = self.point_bounds[waypoint_index][:2] - drone_xyz[:2]
                unit_vector_to_point = vector_to_waypoint/numpy.linalg.norm(vector_to_waypoint)
                move_vector = self.dist_between_waypoints * unit_vector_to_point
                pos_vector = drone_xyz[:2] + move_vector
                drone_xyz = numpy.array([pos_vector[0], pos_vector[1], drone_xyz[2] + self.dist_between_waypoints * self.altitude_roc])

            iter_segment = [self.spiral_points[len(self.spiral_points)-1], drone_xyz]
            print("iter segment is: ", iter_segment)
            iter_geoplane = self.waypoint_path_poly_planes[waypoint_index-1]
            iter_segment_with_obstacle_intersections = iter_geoplane.avoid_obstacles_with_line(iter_segment)
            self.spiral_points.extend(iter_segment_with_obstacle_intersections[1:len(iter_segment_with_obstacle_intersections)])
        self.total_path.extend(self.spiral_points)

    '''flood-fills the bounding box of the area at the max altitude. Does not work at concave corners, as it must skip outside
    of the geo bounds in order to reach the next point on the scan line'''
    def get_top_floodfill_points(self, top_point_bounds):

        '''needs to reverse lines so it doesn't waste time going a farther line rather than just moving down and
        going backwards'''

        bounding_flat_polygon = FlatPolygon(top_point_bounds)
        floodfill_lines = bounding_flat_polygon.get_floodfill_lines(self.top_fill_spacing)
        floodfill_waypoints = []
        longest_length = 0
        for i in range(0, len(floodfill_lines)):
            if len(floodfill_lines[i]) > longest_length:
                longest_length = len(floodfill_lines[i])

        for floodfill_at_y_index in range(0, longest_length):
            reverse = False
            for i in range(0, len(floodfill_lines)):
                iter_line = list(floodfill_lines[i])
                #if reverse:
                #    iter_line = list(reversed(iter_line))
                if floodfill_at_y_index < len(iter_line):
                    p1 = 0
                    p2 = 0
                    if not reverse:
                        p1 = iter_line[floodfill_at_y_index][0]
                        p2 = iter_line[floodfill_at_y_index][1]
                    else:
                        p1 = iter_line[floodfill_at_y_index][1]
                        p2 = iter_line[floodfill_at_y_index][0]
                    p1_with_z = numpy.array([p1[0], p1[1], self.max_altitude])
                    p2_with_z = numpy.array([p2[0], p2[1], self.max_altitude])
                    floodfill_lines[i][floodfill_at_y_index] = (p1_with_z, p2_with_z)
                    floodfill_waypoints.append(p1_with_z)
                    floodfill_waypoints.append(p2_with_z)
                reverse = not reverse
        return floodfill_waypoints

    '''creates a path by shrinking the perimeter of the gps bounds until it has covered the entire structure'''
    def get_top_perimeter_scale_points(self, top_point_bounds):
        bounding_flat_polygon = FlatPolygon(top_point_bounds)
        out_waypoints = []
        for i in range(1, 12):
            perimeter_points = bounding_flat_polygon.get_polygon_scaled_with_margin(self.top_fill_spacing * i).points.tolist()
            for j in range(0, len(perimeter_points)):
                perimeter_points[j] = numpy.array([perimeter_points[j][0], perimeter_points[j][1], self.max_altitude])
            out_waypoints.extend(perimeter_points)
        return out_waypoints


    DRONE_PATH_EXTENSION = "/path"
    OBSTACLE_EXTENSION = "/obstacles"
    SLICE_EXTENSION = "/plane_slices"
    '''saves the points to be displayed by the renderer'''
    def save(self, dir, name):
        if not os.path.exists(dir + "/" + name):
            os.makedirs(dir + "/" + name)
        path_dir = dir + "/" + name + WaypointBuilder.DRONE_PATH_EXTENSION + ".txt"
        obstacle_dir = dir + "/" + name + WaypointBuilder.OBSTACLE_EXTENSION + ".txt"
        slice_dir = dir + "/" + name + WaypointBuilder.SLICE_EXTENSION + ".txt"

        with open(path_dir, 'w') as path_output:
            path_output.write(str(self.total_path))
            path_output.close()

        with open(obstacle_dir, 'w') as obstacle_output:
            write_str = ""
            for i in range(0, len(self.obstacles)):
                write_str += str(self.obstacles[i].shape.get_points())
                if i < len(self.obstacles) - 1:
                    write_str += "\n"
            obstacle_output.write(write_str)
            obstacle_output.close()

        with open(slice_dir, 'w') as slice_output:
            write_str = ""
            for i in range(0, len(self.plane_intersection_slices)):
                write_str += str(self.plane_intersection_slices[i])
                if i < len(self.plane_intersection_slices)-1:
                    write_str += "\n"
            slice_output.write(write_str)
            slice_output.close()
