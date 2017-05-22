from math import sqrt,pi, sin, cos
import numpy

class Sphere:

    def __init__(self, center, constraints):
        self.center = center
        self.constraints = constraints
        self.radius = constraints[0]

    '''returns a list of all points and out_points, where out points is organized by layer, and all points is just a list of
    every point'''
    def get_points(self, resolution_height = 20, resolution_theta = 32):
        out_points = []
        all_points = []
        for i in range(0, resolution_height):
            slice_points = []
            displacement = (2.0*self.radius*float(i)/float(resolution_height))-self.radius
            radius_at_slice = self.get_slice_radius_at_displacement(displacement)
            displacement
            theta_multiplier = 2.0*pi/resolution_theta
            for j in range(0, resolution_theta+1):
                theta = theta_multiplier * float(j)
                append_point = numpy.array([radius_at_slice * cos(theta), radius_at_slice * sin(theta), displacement])
                append_point += self.center
                slice_points.append(append_point)
                all_points.append(append_point)
            out_points.append(slice_points)
        return all_points, out_points

    def get_slice_radius_at_displacement(self, displacement):
        return sqrt(self.radius**2 - displacement**2)

    def get_intersection_points_with_plane(self, poly_plane, resolution):
        dist_to_center = poly_plane.distance_to_point(self.center)
        if dist_to_center >= self.radius:
            return None
        slice_radius = self.get_slice_radius_at_displacement(dist_to_center)
        plane_basises = poly_plane.basises
        theta_multiplier = 2.0*pi/float(resolution)
        possible_plane_center_point1 = self.center - poly_plane.unit_normal*dist_to_center
        dist_to1 = poly_plane.distance_to_point(possible_plane_center_point1)
        possible_plane_center_point2 = self.center + poly_plane.unit_normal*dist_to_center
        dist_to2 = poly_plane.distance_to_point(possible_plane_center_point2)
        plane_center_point = possible_plane_center_point1 if dist_to1 < dist_to2 else possible_plane_center_point2

        untransformed_circle_points = []
        for theta_index in range(0, resolution + 1):
            theta = float(theta_multiplier * theta_index)
            iter_point = numpy.array([slice_radius * cos(theta), slice_radius * sin(theta), 0])
            untransformed_circle_points.append(iter_point)


        transformed_circle_points = []
        for i in range(0, len(untransformed_circle_points)):
            transformed_circle_points.append(untransformed_circle_points[i].dot(plane_basises) + plane_center_point)

        i = 0
        while i < len(transformed_circle_points):
            point_in_plane = poly_plane.point_lies_in_bounded_plane(transformed_circle_points[i])
            if not point_in_plane:
                del transformed_circle_points[i]
            else:
                i+=1
        '''if there are points outside of the bounds of the plane, after their removal, they must be replaced with points that make
        them the correct polygon inside the bounds of the plane (i.e. if it's cut off at an edge, at a point at the points on that edge
        that define the shape)'''
        return transformed_circle_points
