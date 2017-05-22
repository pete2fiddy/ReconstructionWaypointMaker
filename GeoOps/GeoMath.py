from math import pi, sin, cos, asin, sqrt, atan2
import numpy
from GeoOps.GeoPoint import GeoPoint

EARTH_RADIUS_KM = 6371.0
EQUITORIAL_EARTH_RADIUS_KM = 6378.1370
def haversine_distance(compare_geo_point1, compare_geo_point2, include_alt = False):
    dlon = compare_geo_point2.long_radians - compare_geo_point1.long_radians
    dlat = compare_geo_point2.lat_radians - compare_geo_point1.lat_radians

    a = (sin(dlat/2))**2 + cos(compare_geo_point1.lat_radians) * cos(compare_geo_point2.lat_radians) * (sin(dlon/2))**2
    c = 2.0 * atan2( sqrt(a), sqrt(1-a) )
    geo_dist = EARTH_RADIUS_KM* c
    if not include_alt:
        return geo_dist
    return sqrt(geo_dist**2 + (compare_geo_point2.alt - compare_geo_point1.alt)**2)

'''geo1 is the origin, geo2 is relative to it'''
def vector_between_geo_points(geo1, geo2):
    v1 = numpy.zeros((3))
    v2 = numpy.zeros((3))
    r1 = geo1.alt + EQUITORIAL_EARTH_RADIUS_KM
    r2 = geo2.alt + EQUITORIAL_EARTH_RADIUS_KM

    v1[0] = r1*cos(geo1.lat_radians)*cos(geo1.long_radians)
    v1[1] = r1*cos(geo1.lat_radians)*sin(geo1.long_radians)
    v1[2] = r1*sin(geo1.lat_radians) * (1.0 - (1.0/298.257223563))

    v2[0] = r2*cos(geo2.lat_radians)*cos(geo2.long_radians)
    v2[1] = r2*cos(geo2.lat_radians)*sin(geo2.long_radians)
    v2[2] = r2*sin(geo2.lat_radians) * (1.0 - (1.0/298.257223563))

    '''for some reason the above method was not working correctly for altitude, so adjusted it to below. Probably has something
    to do with keeping linear altitude scaled correctly, but I want altitude to be consistent, i.e. 50 ft here is the same as 50 ft
    there, even if the earth is curved between the points'''
    subtraction = v2-v1
    subtraction[2] = geo2.alt - geo1.alt
    return subtraction

def get_total_haversine_distance(geo_points, is_loop = False, include_alt = False):
    sum = 0
    for i in range(0, len(geo_points) - 1):
        sum += haversine_distance(geo_points[i], geo_points[i+1], include_alt = include_alt)#geo_points[i].haversine_distance(geo_points[i+1], include_alt = include_alt)
    sum += haversine_distance(geo_points[len(geo_points)-1], geo_points[0], include_alt = include_alt)#geo_points[len(geo_points)-1].haversine_distance(geo_points[0], include_alt = include_alt) if is_loop else 0
    return sum


def get_avg_geo_point(geo_points):
    x = 0
    y = 0
    z = 0
    height = 0
    for i in range(0, len(geo_points)):
        x += cos(geo_points[i].lat_radians) * cos(geo_points[i].long_radians)
        y += cos(geo_points[i].lat_radians) * sin(geo_points[i].long_radians)
        z += sin(geo_points[i].lat_radians)
        height += geo_points[i].alt


    x /= float(len(geo_points))
    y /= float(len(geo_points))
    z /= float(len(geo_points))

    central_longitude = atan2(y, x);
    central_square_root = sqrt(x**2 + y**2);
    central_latitude = atan2(z, central_square_root);
    return GeoPoint(central_latitude*180.0/pi, central_longitude*180.0/pi, height/float(len(geo_points)))
