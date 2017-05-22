
from math import pi, sin, cos, asin, sqrt
#from GeoOps.GeoMath import GeoMath



class GeoPoint:
    '''
    altitude of geo points are expressed in km
    '''
    def __init__(self, latitude, longitude, altitude = 0):
        self.lat = latitude
        self.long = longitude
        self.alt = altitude
        self.init_radians()
        self.import_geo_math()

    def import_geo_math(self):
        import GeoOps.GeoMath as GeoMath

    def init_radians(self):
        self.lat_radians = (self.lat/180.0)*pi
        self.long_radians = (self.long/180.0)*pi

    def __str__(self):
        return "lat: " + str(self.lat) + " long: " + str(self.long) + " alt: " + str(self.alt)
