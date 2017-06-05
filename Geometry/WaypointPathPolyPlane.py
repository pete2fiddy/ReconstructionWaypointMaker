from Geometry.PolyPlane import PolyPlane
from Geometry.FlatPolygon import FlatPolygon
import numpy
from Geometry.SliceObstacleShape import SliceObstacleShape

class WaypointPathPolyPlane(PolyPlane):

    '''assumes only four points are passed??? (Does it now? Made this comment a while ago)
    Points must be in polygon-point-draw order, and the first two points must be at the bottom of the polygon'''
    def __init__(self, bounding_points):
        PolyPlane.__init__(self, bounding_points)
        self.slice_obstacle_shapes = []
        self.slicing_flat_polygons = []
        '''flat_basis_bounds holds the bounding points of the polygon projected onto the plane using the self.basises vectors'''

    def add_polygon_slice(self, obstacle, poly_points):
        self.slice_obstacle_shapes.append(SliceObstacleShape(obstacle, PolyPlane(poly_points)))
        self.add_flat_basis_polygon_slice(poly_points)

    def add_flat_basis_polygon_slice(self, poly_points):
        transformed_poly_points = [(poly_points[i]-self.origin).dot(self.basises) for i in range(0, len(poly_points))]
        self.slicing_flat_polygons.append(FlatPolygon(transformed_poly_points))

    def get_obstacle_slices(self):
        out_slices = [self.slice_obstacle_shapes[i].obstacle_slice_polyplane for i in range(0, len(self.slice_obstacle_shapes))]
        return out_slices

    def avoid_obstacles_with_line(self, line_points):
        if not self.line_lies_in_unbounded_plane(line_points):
            return line_points
        flat_line_points = [(line_points[i]-self.origin).dot(self.basises) for i in range(0, len(line_points))]

        '''holds tuples in the format point: (numpy xyz), slicing poly plane intersecting that point'''
        intersection_points = []
        for i in range(0, len(self.slicing_flat_polygons)):
            iter_intersection_points = self.slicing_flat_polygons[i].get_segment_intersections(flat_line_points)
            if len(iter_intersection_points) > 0:
                num_zeros_to_append = line_points[0].shape[0] - iter_intersection_points[0].shape[0]
                extend_zero_list = [0 for j in range(0, num_zeros_to_append)]
                for j in range(0, len(iter_intersection_points)):

                    iter_intersection_points[j] = iter_intersection_points[j].tolist()
                    iter_intersection_points[j].extend(extend_zero_list)
                    iter_intersection_points[j] = numpy.array(iter_intersection_points[j])
                    if j%2 == 0:
                        intersection_points.append((iter_intersection_points[j].dot(self.inverted_basises) + self.origin, self.slice_obstacle_shapes[i].obstacle_slice_polyplane))
                    else:
                        intersection_points.append((iter_intersection_points[j].dot(self.inverted_basises) + self.origin, None))
        i=0

        while i < len(intersection_points):
            if not self.point_lies_in_bounded_plane(intersection_points[i][0]):


                del[intersection_points[i]]

            else:
                i+=1

        intersection_points.sort(key = lambda intersection_point: numpy.linalg.norm(intersection_point[0] - line_points[0]))
        intersection_points.insert(0, (line_points[0], None))
        intersection_points.append((line_points[1], None))


        out_segments = []
        i = 0
        while i < len(intersection_points):
            if not intersection_points[i][1] == None:
                if i < len(intersection_points)-1:

                    start_point = intersection_points[i-1][0]
                    end_point = intersection_points[i][0]
                    path_to_extend = intersection_points[i][1].get_path_between_start_and_end_around_perimeter(start_point, end_point)
                    out_segments.append(start_point)
                    out_segments.append(end_point)

            else:
                out_segments.append(intersection_points[i][0])
            i += 1
        return out_segments
