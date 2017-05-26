from Geometry.WaypointPathPolyPlane import WaypointPathPolyPlane
from Geometry.PolyPlane import PolyPlane
from WaypointOps.WaypointBuilder import WaypointBuilder
import GeoOps.GeoMath as GeoMath
from WaypointOps.WaypointSegment import WaypointSegment
from WaypointOps.WaypointSegments import WaypointSegments
import numpy
import os


class OffsetWaypointBuilder(WaypointBuilder):

    DEFAULT_ALTITUDE_ROC = 0.01

    def __init__(self, bounding_geopoints, obstacles, max_alt, altitude_roc = None):
        self.bounding_geopoints = bounding_geopoints
        self.max_alt = max_alt

        self.init_point_bounds()
        self.init_point_path_polyplanes()

        self.altitude_roc = altitude_roc if altitude_roc != None else OffsetWaypointBuilder.DEFAULT_ALTITUDE_ROC

        WaypointBuilder.__init__(self, obstacles, self.GEO_ORIGIN)

    def init_point_bounds(self):
        '''GEO_ORIGIN is in caps to signify that it should NEVER be edited (it gets passed to other classes and if there is a change in
        one place, it would be incredibly confusing to fix)'''
        self.GEO_ORIGIN = GeoMath.get_avg_geo_point(self.bounding_geopoints)
        self.point_bounds = self.geos_to_points(self.bounding_geopoints, self.GEO_ORIGIN)

    def init_point_path_polyplanes(self):
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
        '''is a duplicate of the same code from LoiterCylinderWaypointBuilder, possible to
        create a method that both can access that share the same code? Seems a little too
        corner-case'''
        waypoint_segments = WaypointSegments.init_empty()
        drone_xyz = self.point_bounds[0].copy()
        waypoint_index = 1
        while drone_xyz[2] < self.max_alt:
            dist_to_next_waypoint = numpy.linalg.norm(self.point_bounds[waypoint_index][:2] - drone_xyz[:2])
            vector_to_waypoint_2d = self.point_bounds[waypoint_index][:2] - drone_xyz[:2]
            unit_vector_to_point_2d = vector_to_waypoint_2d/numpy.linalg.norm(vector_to_waypoint_2d)
            move_vector_2d = dist_to_next_waypoint * unit_vector_to_point_2d

            next_drone_xyz = drone_xyz + numpy.append(move_vector_2d, dist_to_next_waypoint * self.altitude_roc)
            '''be careful when instantiating waypoint segments that you are matching the segment to the correct plane'''
            iter_segment = WaypointSegment([drone_xyz, next_drone_xyz], self.point_path_polyplanes[waypoint_index])
            '''copying in case it changes the pointer'''
            drone_xyz = next_drone_xyz.copy()

            waypoint_segments.append(iter_segment)
            waypoint_index = (waypoint_index + 1)%(len(self.point_bounds))

        return waypoint_segments
