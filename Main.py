import GeoOps.GeoMath as GeoMath
from GeoOps.GeoPoint import GeoPoint

import ImgOps.VectorDraw as VectorDraw
import numpy
import Geometry.LineMath as LineMath
from WaypointOps.OffsetWaypointBuilder import OffsetWaypointBuilder
import Geometry.AngleMath as AngleMath
from WaypointOps.Obstacles import Obstacles
from Geometry.ObstacleSphere import ObstacleSphere
from WaypointOps.LoiterCylinderWaypointBuilder import LoiterCylinderWaypointBuilder
from WaypointOps.FixedAltLoiterWaypointBuilder import FixedAltLoiterWaypointBuilder


#obstacle_geos = [GeoPoint(38.860461, -77.242483, 0.0035)]#
obstacle_geos = [GeoPoint(38.860438, -77.242484, 0.0025), GeoPoint(38.86042980000243,-77.24247560000045,0), GeoPoint(38.860469, -77.242500, .0045), GeoPoint(38.860461, -77.242483, 0.0035)]
obstacle_constraints = [.75, 0.85, 1.0, 1.1]#[1.1]#
obstacles = Obstacles.init_empty()


for i in range(0, len(obstacle_geos)):
    obstacles.append(ObstacleSphere(obstacle_geos[i], obstacle_constraints[i]))

geo_points = [GeoPoint(38.860408, -77.242508, 0), GeoPoint(38.860453, -77.242499, 0), GeoPoint(38.860443, -77.242451, 0), GeoPoint(38.860417, -77.242451, 0), GeoPoint(38.860428, -77.242469, 0)]
#geo_points = list(reversed(geo_points))

waypoint_builder = OffsetWaypointBuilder(geo_points, obstacles, 5.0)#LoiterCylinderWaypointBuilder(geo_points[0], 3, obstacles, 5.0)#FixedAltLoiterWaypointBuilder(geo_points[0], 3, obstacles)

waypoint_builder.save("/Users/phusisian/Desktop/DZYNE/Python/Generated Waypoints/", "waypoints10")
#path_is_safe = waypoint_builder.path_is_safe(0.02, 0.02)
#print("path safe? ", path_is_safe)
