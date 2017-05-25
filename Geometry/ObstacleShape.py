from abc import ABC, abstractmethod

class ObstacleShape(ABC):

    @abstractmethod
    def obstacle_points(self):
        pass

    @abstractmethod
    def init_cartesian_points(self, GEO_ORIGIN):
        pass

    @abstractmethod
    def unbounded_intersect_with_plane(self, plane, resolution):
        pass

    def clip_points_with_plane(self, points, plane):
        '''filler to add'''
