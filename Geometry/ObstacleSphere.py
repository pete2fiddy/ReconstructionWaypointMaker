import GeoOps.GeoMath as GeoMath
from Geometry.ObstacleShape import ObstacleShape
import numpy
from math import pi, cos, sin, sqrt

class ObstacleSphere(ObstacleShape):

    DEFAULT_HEIGHT_RESOLUTION = 12
    DEFAULT_THETA_RESOLUTION = 32

    def __init__(self, center_geo, radius_m):
        self.center_geo = center_geo
        self.radius_m = radius_m

    def init_cartesian_points(self, GEO_ORIGIN):
        self.center_point = GeoMath.vector_between_geo_points(GEO_ORIGIN, self.center_geo) * 1000.0

    def point_in_obstacle(self, point, safety_margin):
        
        return (numpy.linalg.norm(point - self.center_point) < self.radius_m - safety_margin)

    def intersect_with_plane(self, plane, resolution, bounded = True):
        dist_to_plane = plane.distance_to_point(self.center_point)
        if dist_to_plane > self.radius_m:
            return None
        unbounded_intersections = []
        slice_radius = self.slice_radius_at_displacement(dist_to_plane)
        plane_basises = plane.basises

        '''the distance to the plane after adding or subtracting the unit normal * the distance to the plane.
        Have to figure out whether the vector must be added or subtracted by using which point is closer'''
        center_point_plus_dist = plane.distance_to_point(self.center_point + plane.unit_normal * dist_to_plane)
        center_point_sub_dist = plane.distance_to_point(self.center_point - plane.unit_normal * dist_to_plane)

        slice_center_point = self.center_point.copy()
        slice_center_point += plane.unit_normal * dist_to_plane if center_point_plus_dist < center_point_sub_dist else -plane.unit_normal * dist_to_plane

        theta_multiplier = 2.0*pi/float(resolution)
        '''points are first made flat in the z axis, then are transformed to fit the plane correctly'''
        untransformed_circle_points = [0 for i in range(0, resolution)]

        for theta_index in range(0, resolution):
            theta = float(theta_multiplier * theta_index)
            point_at_rotation = slice_radius * numpy.array([cos(theta), sin(theta), 0])
            untransformed_circle_points[theta_index] = point_at_rotation

        transformed_circle_points = [untransformed_circle_points[i].dot(plane_basises) + slice_center_point for i in range(0, resolution)]
        if bounded:
            return self.clip_points_with_plane(transformed_circle_points, plane)
        return transformed_circle_points

    def slice_radius_at_displacement(self, displacement):
        return sqrt(self.radius_m**2 - displacement**2)

    def obstacle_points(self):
        draw_altitude = self.center_point[2] - self.radius_m
        height_increment = 2.0*self.radius_m / float(ObstacleSphere.DEFAULT_HEIGHT_RESOLUTION)
        theta_increment = 2.0*pi / float(ObstacleSphere.DEFAULT_THETA_RESOLUTION)
        obstacle_points = []
        while draw_altitude < self.center_point[2] + self.radius_m:
            radius_at_altitude = self.slice_radius_at_displacement(abs(draw_altitude - self.center_point[2]))
            theta = 0
            while theta < 2.0*pi:
                append_point = numpy.array([radius_at_altitude * cos(theta), radius_at_altitude * sin(theta), draw_altitude - self.center_point[2]])
                append_point += self.center_point
                obstacle_points.append(append_point)

                theta += theta_increment
            draw_altitude += height_increment
        return obstacle_points
