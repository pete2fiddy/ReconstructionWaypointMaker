import GeoOps.GeoMath as GeoMath


class Obstacle:

    def __init__(self, geo_point, shape_class, constraints):
        self.geo_point = geo_point
        self.shape_class = shape_class
        self.constraints = constraints

    def init_shape(self, geo_origin):
        '''creates the point relative to the origin that this obstacle is situated on. Instantiates the shape on this point.
        Multiplies by 1000 so that the measurement is in meters'''
        center_point = GeoMath.vector_between_geo_points(geo_origin, self.geo_point) * 1000.0
        self.shape = self.shape_class(center_point, self.constraints)
