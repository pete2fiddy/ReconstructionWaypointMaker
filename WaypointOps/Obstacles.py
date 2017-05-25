


class Obstacles:

    def __init__(self, obstacles):
        self.obstacles = obstacles

    @classmethod
    def init_empty(cls):
        return Obstacles([])

    def init_obstacle_cartesian_points(self, GEO_ORIGIN):
        for i in range(0, len(self)):
            self[i].init_cartesian_points(GEO_ORIGIN)

    def __getitem__(self, index):
        return self.obstacles[index]

    def __setitem__(self, value, index):
        self.obstacles[index] = value

    def __len__(self):
        return len(self.obstacles)

    def append(self, obstacle):
        self.obstacles.append(obstacle)
