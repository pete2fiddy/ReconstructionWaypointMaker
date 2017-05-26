from math import pi, cos, sin
import numpy
from WaypointOps.WaypointSegments import WaypointSegments
from WaypointOps.WaypointSegment import WaypointSegment
from Geometry.WaypointPathPolyPlane import WaypointPathPolyPlane
from WaypointOps.WaypointBuilder import WaypointBuilder


class FixedAltLoiterWaypointBuilder(WaypointBuilder):

    DEFAULT_ALTITUDE_ROC = 0.01
    DEFAULT_THETA_RESOLUTION = 64
    def __init__(self, center_geopoint, radius_m, obstacles, altitude_roc = None, theta_resolution = None):
        self.center_geopoint = center_geopoint
        self.fly_alt = center_geopoint.alt
        self.radius_m = radius_m
        self.altitude_roc = altitude_roc if altitude_roc != None  else FixedAltLoiterWaypointBuilder.DEFAULT_ALTITUDE_ROC
        self.theta_resolution = theta_resolution if theta_resolution != None else FixedAltLoiterWaypointBuilder.DEFAULT_THETA_RESOLUTION
        self.init_point_bounds()
        self.init_point_path_polyplane()
        WaypointBuilder.__init__(self, obstacles, self.center_geopoint)

    def init_point_bounds(self):
        '''no operations necessary to convert geos to points because you are given
        a radius, and all points are already relative to the center geopoint'''
        self.point_bounds = []
        theta = 0
        theta_increment = 2.0 * pi/float(self.theta_resolution)
        while theta < 2.0*pi:
            point_at_angle = numpy.array([self.radius_m * cos(theta), self.radius_m * sin(theta), self.fly_alt])
            self.point_bounds.append(point_at_angle)
            theta += theta_increment

    def init_point_path_polyplane(self):
        self.point_path_polyplane = WaypointPathPolyPlane(self.point_bounds)

    def create_waypoint_segments(self):
        segments_list = [WaypointSegment([self.point_bounds[i-1], self.point_bounds[i]], self.point_path_polyplane) for i in range(0, len(self.point_bounds))]
        return WaypointSegments(segments_list)
