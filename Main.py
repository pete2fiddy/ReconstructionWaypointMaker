import GeoOps.GeoMath as GeoMath
from GeoOps.GeoPoint import GeoPoint
from WaypointOps.WaypointBuilder import WaypointBuilder
import ImgOps.VectorDraw as VectorDraw
from Geometry.Sphere import Sphere
import numpy
from WaypointOps.Obstacle import Obstacle

#geo_points = [GeoPoint(38.860841, -77.242213, 0), GeoPoint(38.860904, -77.242202, 0), GeoPoint(38.860893, -77.242140, 0),  GeoPoint(38.860851, -77.242133, 0), GeoPoint(38.860829, -77.242166, 0)]


obstacle_geos = [GeoPoint(38.860438, -77.242484, 0.0025), GeoPoint(38.86042980000243,-77.24247560000045,0), GeoPoint(38.860469, -77.242500, .001), GeoPoint(38.860461, -77.242483, 0.0035)]
obstacle_constraints = [(.75,), (0.85,), (1.0,), (1.1,)]
obstacle_shape_type = Sphere
obstacles = []

for i in range(0, len(obstacle_geos)):
    obstacles.append(Obstacle(obstacle_geos[i], obstacle_shape_type, obstacle_constraints[i]))

geo_points = [GeoPoint(38.860408, -77.242508, 0), GeoPoint(38.860453, -77.242499, 0), GeoPoint(38.860443, -77.242451, 0), GeoPoint(38.860417, -77.242451, 0), GeoPoint(38.860428, -77.242469, 0)]
geo_points = list(reversed(geo_points))
print("distance between points: ", GeoMath.get_total_haversine_distance(geo_points, is_loop = True))
waypoint_builder = WaypointBuilder(geo_points, obstacles, 5.0, WaypointBuilder.get_top_floodfill_points)
waypoint_builder.save("/Users/phusisian/Desktop/DZYNE/Python/Generated Waypoints/", "waypoints5")
