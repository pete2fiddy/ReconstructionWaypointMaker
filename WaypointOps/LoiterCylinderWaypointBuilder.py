from WaypointOps.WaypointBuilder import WaypointBuilder
from WaypointOps.WaypointSegments import WaypointSegments
from WaypointOps.WaypointSegment import WaypointSegment
from Geometry.WaypointPathPolyPlane import WaypointPathPolyPlane
from math import pi, cos, sin
import numpy

class LoiterCylinderWaypointBuilder(WaypointBuilder):

    DEFAULT_ALTITUDE_ROC = 0.03
    DEFAULT_THETA_RESOLUTION = 16

    def __init__(self, center_geopoint, radius_m, obstacles, max_alt, altitude_roc = None, theta_resolution = None):
        self.center_geopoint = center_geopoint
        self.min_alt = center_geopoint.alt
        self.max_alt = max_alt
        self.radius_m = radius_m
        self.altitude_roc = altitude_roc if altitude_roc != None else LoiterCylinderWaypointBuilder.DEFAULT_ALTITUDE_ROC
        self.theta_resolution = theta_resolution if theta_resolution != None else LoiterCylinderWaypointBuilder.DEFAULT_THETA_RESOLUTION
        self.init_point_bounds()
        self.init_point_path_polyplanes()
        WaypointBuilder.__init__(self, obstacles, self.center_geopoint)

    def init_point_bounds(self):
        '''no operations are needed to convert the point bounds from geo to
        cartesian since they are all relative to the center geo already,
        meaning that they are relative to the origin to begin with'''

        '''creating circles is fairly often duplicated, and could be moved to an
        exterior class. However, the constraints are often different, but only slightly
        and it may just be better to recode it every time (i.e. some require fixed height),
        fixed x, fixed y, or other constraints that will be difficult to generalize'''

        self.point_bounds = []
        theta = 0
        theta_increment = 2.0*pi/float(self.theta_resolution)
        while theta < 2.0*pi:
            point_at_angle = numpy.array([self.radius_m * cos(theta), self.radius_m * sin(theta), self.min_alt])
            self.point_bounds.append(point_at_angle)
            theta += theta_increment

    def init_point_path_polyplanes(self):
        '''is a duplicate of the same code from OffsetWaypointBuilder, possible to
        create a method that both can access that share the same code? Seems a little too
        corner-case'''
        self.point_path_polyplanes = []
        for i in range(0, len(self.point_bounds)):
            p1 = self.point_bounds[i-1]
            p2 = self.point_bounds[i]
            p3 = p2.copy()
            p3[2] = self.max_alt
            p4 = p1.copy()
            p4[2] = self.max_alt
            iter_points = [p1, p2, p3, p4]
            iter_poly_plane = WaypointPathPolyPlane(iter_points)
            self.point_path_polyplanes.append(iter_poly_plane)

    def create_waypoint_segments(self):
        waypoint_segments = WaypointSegments.init_empty()
        drone_xyz = self.point_bounds[0].copy()
        waypoint_index = 1
        while drone_xyz[2] < self.max_alt:
            vector_to_waypoint_2d = self.point_bounds[waypoint_index][:2] - drone_xyz[:2]
            dist_to_next_waypoint = numpy.linalg.norm(vector_to_waypoint_2d)
            unit_vector_to_point_2d = vector_to_waypoint_2d/numpy.linalg.norm(vector_to_waypoint_2d)
            move_vector_2d = dist_to_next_waypoint * unit_vector_to_point_2d
            next_drone_xyz = drone_xyz + numpy.append(move_vector_2d, dist_to_next_waypoint * self.altitude_roc)
            '''be careful when instantiating waypoint segments that you are matching the segment to the correct plane'''
            iter_segment = WaypointSegment([drone_xyz, next_drone_xyz], self.point_path_polyplanes[waypoint_index])
            '''copying in case it changes the pointer'''
            drone_xyz = next_drone_xyz.copy()

            waypoint_segments.append(iter_segment)
            waypoint_index = (waypoint_index + 1)%len(self.point_bounds)
        return waypoint_segments
