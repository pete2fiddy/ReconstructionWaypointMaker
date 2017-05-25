from math import sqrt,pi, sin, cos, atan2
import numpy
import Geometry.LineMath as LineMath
import VectorOps.VectorMath as VectorMath
import Geometry.AngleMath as AngleMath
from Geometry.PolyPlane import PolyPlane

class Sphere:

    MAX_DISTANCE_TO_LINE_FOR_INTERSECTION = 0.001

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

    def get_clipped_points(self, points_in, poly_plane):



        clipped_intersections = []
        points_plane = PolyPlane(points_in)

        points = sorted(list(points_in), key = lambda point: points_plane.get_angle_to_coplanar_point_from_first_basis(point))
        points_basis_vectors = points_plane.basises
        plane_contains_at_least_one_point = False
        point_in_plane_arr = []

        #print("transformed points are: ", points)
        '''finds all 3d points of intersection between the polygon and the poly plane'''
        for i in range(0, len(points)):
            #print("angle to point in plane: ", points_plane.get_angle_to_coplanar_point_from_first_basis(points[i]))
            point_segment = [points[i-1], points[i]]
            point_in_bounds = poly_plane.point_lies_in_bounded_plane(points[i])
            point_in_plane_arr.append(point_in_bounds)
            if point_in_bounds:
                plane_contains_at_least_one_point = True

            segment_intersections_with_plane = poly_plane.get_segment_intersections_with_edges(point_segment, Sphere.MAX_DISTANCE_TO_LINE_FOR_INTERSECTION)

            clipped_intersections.extend(segment_intersections_with_plane)


        '''if no points lie in the bounded plane, then it must be entirely clipped off'''
        if not plane_contains_at_least_one_point:
            return []

        '''if there are no intersections, nothing needs to be clipped'''
        if len(clipped_intersections) == 0:
            return points


        print("clip intersections: ", clipped_intersections)

        in_bounds_angle_ranges = []


        for i in range(0, len(point_in_plane_arr)):
            prev_point_in_plane = point_in_plane_arr[i-1]
            this_point_in_plane = point_in_plane_arr[i]



            if not prev_point_in_plane and this_point_in_plane:
                end_segment_index = i
                angle_to_point1 = points_plane.get_angle_to_coplanar_point_from_first_basis(points[i])
                for j in range(i, len(point_in_plane_arr)+i):
                    j_index = j%len(point_in_plane_arr)
                    if not point_in_plane_arr[j_index]:
                        end_segment_index = j_index
                        break

                print("found start index: ", i, " , and end index: ", end_segment_index)

                angle_to_point2 = points_plane.get_angle_to_coplanar_point_from_first_basis(points[end_segment_index])
                append_range = (angle_to_point1, angle_to_point2)
                in_bounds_angle_ranges.append(append_range)





        print("in bounds angle ranges: ", in_bounds_angle_ranges)

        point_index = 0

        while point_index < len(points):

            angle_to_point = points_plane.get_angle_to_coplanar_point_from_first_basis(points[point_index])
            angle_in_range = AngleMath.angle_in_angle_ranges(in_bounds_angle_ranges, angle_to_point)

            if angle_in_range:
                point_index+=1
            else:

                del[points[point_index]]

        points.extend(clipped_intersections)
        for i in range(0, len(poly_plane.bounding_points)):
            if points_plane.point_lies_in_bounded_plane(poly_plane.bounding_points[i]):
                points.append(poly_plane.bounding_points[i])

        new_points_plane = PolyPlane(points)
        #points.extend(plane_points_points_contain)
        points.sort(key = lambda point: new_points_plane.get_angle_to_coplanar_point_from_first_basis(point))

        return points










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
        #print("transformed circle points: ", transformed_circle_points)
        transformed_circle_points = self.get_clipped_points(transformed_circle_points, poly_plane)

        clip = False
        plane_segments = [[poly_plane.bounding_points[i-1], poly_plane.bounding_points[i]] for i in range(0, len(poly_plane.bounding_points))]
        if clip:
            '''all_intersections = []
            prev_point_intersected = False
            for i in range(0, len(transformed_circle_points)):
                iter_circle_segment = [transformed_circle_points[i-1], transformed_circle_points[i]]
                segment_intersections_with_plane = poly_plane.get)segment_intersections_with_plane(iter_circle_segment, Sphere.MAX_DISTANCE_TO_LINE_FOR_INTERSECTION)
                if len(segment_intersections_with_plane) >= 1:

                    all_intersections.extend([(segment_intersections_with_plane[j], i) for j in range(0, len(segment_intersections_with_plane))])
                    prev_point_intersected = True
            '''
            '''
            for i in range(0, len(plane_segments)):
                j = 0

                while j < len(transformed_circle_points):
                    point_in_plane = poly_plane.point_lies_in_bounded_plane(transformed_circle_points[j])
                    if not point_in_plane:
                        iter_circle_segment = [transformed_circle_points[j-1], transformed_circle_points[j]]
                        segment_intersections_with_plane = poly_plane.get_segment_intersections_with_edge_at_index(iter_circle_segment, i, Sphere.MAX_DISTANCE_TO_LINE_FOR_INTERSECTION)
                        if len(segment_intersections_with_plane) == 1:
                            transformed_circle_points[j] = segment_intersections_with_plane[0]
                            j+=1
                        else:
                            dist_to_first_index = numpy.linalg.norm(iter_circle_segment[1] - plane_segments[i][0])
                            dist_to_second_index = numpy.linalg.norm(iter_circle_segment[1] - plane_segments[i][1])
                            if dist_to_first_index < dist_to_second_index:
                                transformed_circle_points[j] = plane_segments[i][0]
                            else:
                                transformed_circle_points[j] = plane_segments[i][1]
                            j+=1
                            #del transformed_circle_points[j]
                    else:
                        j+=1'''

            '''
            out_poly = True
            for k in range(0, len(intersect_points)):
                transformed_circle_points'''

            '''temp_transformed_circle_points = []
            is_out = False
            print("clip intersect points: ", clip_intersect_indexes)
            for k in range(0, len(clip_intersect_indexes)):
                if not is_out:
                    temp_transformed_circle_points.extend(transformed_circle_points[clip_intersect_indexes[k]:clip_intersect_indexes[(k+1)%len(clip_intersect_indexes)]])
                is_out = not is_out
            transformed_circle_points = temp_transformed_circle_points'''




            i = 0
            previous_point_in_plane = None

            transformed_circle_points_poly_plane = PolyPlane(transformed_circle_points)
            circle_contain_poly_corners = []
            for i in range(0, len(poly_plane.bounding_points)):
                circle_contains_point = transformed_circle_points_poly_plane.point_lies_in_bounded_plane(poly_plane.bounding_points[i])
                if circle_contains_point:
                    circle_contain_poly_corners.append(poly_plane.bounding_points[i])

            all_intersection_indexes = []
            while i < len(transformed_circle_points):
                point_in_plane = poly_plane.point_lies_in_bounded_plane(transformed_circle_points[i])
                if not point_in_plane:
                    iter_3d_segment = [transformed_circle_points[i-1], transformed_circle_points[i]]
                    if previous_point_in_plane == None or previous_point_in_plane:

                        segment_intersections_with_plane = poly_plane.get_segment_intersections_with_edges(iter_3d_segment, Sphere.MAX_DISTANCE_TO_LINE_FOR_INTERSECTION)
                        if len(segment_intersections_with_plane) == 1:

                            all_intersection_indexes.append(i)
                            transformed_circle_points[i] = segment_intersections_with_plane[0]
                            i+=1

                    else:
                        '''plane_bounding_point_indexes = [i for i in range(0, len(poly_plane.bounding_points))]
                        bounding_plane_point_indexes_sorted_by_distance_to_segment = sorted(plane_bounding_point_indexes, key = lambda bounding_index: numpy.linalg.norm(poly_plane.bounding_points[bounding_index] - iter_3d_segment[0]))
                        closest_plane_bounding_point_index = bounding_plane_point_indexes_sorted_by_distance_to_segment[0]
                        left_segment = [poly_plane.bounding_points[closest_plane_bounding_point_index], poly_plane.bounding_points[closest_plane_bounding_point_index - 1]]
                        right_segment = [poly_plane.bounding_points[closest_plane_bounding_point_index], poly_plane.bounding_points[(closest_plane_bounding_point_index+1)%len(poly_plane.bounding_points)]]

                        sub_left_segment = left_segment[1] - left_segment[0]
                        sub_right_segment = right_segment[1] - right_segment[0]

                        mag_left_segment = numpy.linalg.norm(sub_left_segment)
                        mag_right_segment = numpy.linalg.norm(sub_right_segment)



                        proj_onto_left_segment_mag = VectorMath.scalar_proj((iter_3d_segment[1] - left_segment[0]), (sub_left_segment))
                        proj_onto_right_segment_mag = VectorMath.scalar_proj((iter_3d_segment[1] - right_segment[0]), (sub_right_segment))

                        point_placement_of_left = poly_plane.bounding_points[closest_plane_bounding_point_index] + sub_left_segment * (proj_onto_left_segment_mag/mag_left_segment)
                        point_placement_of_right = poly_plane.bounding_points[closest_plane_bounding_point_index] + sub_right_segment * (proj_onto_right_segment_mag/mag_right_segment)
                        left_point_in_bounds = not proj_onto_left_segment_mag < 0#proj_onto_left_segment_mag > 0 and proj_onto_left_segment_mag < mag_left_segment#poly_plane.point_lies_in_bounded_plane(point_placement_of_left)
                        right_point_in_bounds = not proj_onto_right_segment_mag < 0#proj_onto_right_segment_mag > 0 and proj_onto_right_segment_mag < mag_right_segment#poly_plane.point_lies_in_bounded_plane(point_placement_of_right)
                        if not left_point_in_bounds and not right_point_in_bounds:
                            transformed_circle_points[i] = poly_plane.bounding_points[closest_plane_bounding_point_index]

                        elif left_point_in_bounds and right_point_in_bounds:
                            print("both points were in bounds")
                        elif left_point_in_bounds:
                            transformed_circle_points[i] = point_placement_of_left
                        elif right_point_in_bounds:
                            transformed_circle_points[i] = point_placement_of_right

                        i+=1'''
                        del transformed_circle_points[i]


                    previous_point_in_plane = False
                else:
                    previous_point_in_plane = True
                    i+=1


        print("length of transformed circle points: ", len(transformed_circle_points))
        '''
        i = 0
        previous_point_in_plane = None
        while i < len(transformed_circle_points):
            point_in_plane = poly_plane.point_lies_in_bounded_plane(transformed_circle_points[i])
            if not point_in_plane:
                if previous_point_in_plane == None or previous_point_in_plane:
                    iter_3d_segment = [transformed_circle_points[i-1], transformed_circle_points[i]]
                    iter_flat_segment = [(iter_3d_segment[i]-poly_plane.origin).dot(poly_plane.basises) for i in range(0, len(iter_3d_segment))]
                    print("iter flat segment: ", iter_flat_segment)
                    flat_intersections = poly_plane.flat_basis_polygon.get_segment_intersections(iter_flat_segment)
                    print("flat intersections: ", flat_intersections)
                    flat_3d_intersections = [flat_intersections[j].dot(poly_plane.inverted_basises) + poly_plane.origin for j in range(0, len(flat_intersections))]

                else:
                    del transformed_circle_points[i]

                previous_point_in_plane = False
            else:
                previous_point_in_plane = True
                i+=1
                '''
        '''if there are points outside of the bounds of the plane, after their removal, they must be replaced with points that make
        them the correct polygon inside the bounds of the plane (i.e. if it's cut off at an edge, at a point at the points on that edge
        that define the shape)'''
        return transformed_circle_points
