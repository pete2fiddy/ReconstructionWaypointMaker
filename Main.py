import GeoOps.GeoMath as GeoMath
from GeoOps.GeoPoint import GeoPoint
from WaypointOps.WaypointBuilder import WaypointBuilder
import ImgOps.VectorDraw as VectorDraw
from Geometry.Sphere import Sphere
import numpy
from WaypointOps.Obstacle import Obstacle
import Geometry.LineMath as LineMath
from WaypointOps.OffsetWaypointBuilder import OffsetWaypointBuilder
import Geometry.AngleMath as AngleMath
from WaypointOps.Obstacles import Obstacles
from Geometry.ObstacleSphere import ObstacleSphere

line1 = [numpy.array([6,8,4]), numpy.array([12,15,4])]#[numpy.array([-2,-2,2]), numpy.array([4,2,4])]#[numpy.array([5,5,4]), numpy.array([10,10,6])]#
line2 = [numpy.array([6,8,2]), numpy.array([12,15,6])]#[numpy.array([1,-2,-1.5]), numpy.array([3,4,8.5])]#[numpy.array([0,-3,-2]), numpy.array([2,3,8])]#[numpy.array([5,5,5]), numpy.array([10,10,3])]#[numpy.array([6,8,2]), numpy.array([12,15,6])]
print("Intersection between lines: ", LineMath.get_points_of_minimum_distance_between_lines_3d(line1, line2, False))


obstacle_geos = [GeoPoint(38.860438, -77.242484, 0.0025), GeoPoint(38.86042980000243,-77.24247560000045,0), GeoPoint(38.860469, -77.242500, .0045), GeoPoint(38.860461, -77.242483, 0.0035)]
obstacle_constraints = [(.75,), (0.85,), (1.0,), (1.1,)]
obstacle_shape_type = Sphere
obstacles = Obstacles.init_empty()


for i in range(0, len(obstacle_geos)):
    obstacles.append(ObstacleSphere(obstacle_geos[i], obstacle_constraints[i][0]))

geo_points = [GeoPoint(38.860408, -77.242508, 0), GeoPoint(38.860453, -77.242499, 0), GeoPoint(38.860443, -77.242451, 0), GeoPoint(38.860417, -77.242451, 0), GeoPoint(38.860428, -77.242469, 0)]
geo_points = list(reversed(geo_points))

waypoint_builder = OffsetWaypointBuilder(geo_points, obstacles, 5.0)#(geo_points, obstacles, 5.0, WaypointBuilder.get_top_floodfill_points)
waypoint_builder.save("/Users/phusisian/Desktop/DZYNE/Python/Generated Waypoints/", "waypoints7")
