

class WaypointSegments:

    def __init__(self, waypoint_segments):
        self.waypoint_segments = waypoint_segments
        print("waypoint segments: ", self.waypoint_segments)

    def __getitem__(self, index):
        return self.waypoint_segments[index]

    def __len__(self):
        return len(self.waypoint_segments)

    def __setitem__(self, index, waypoint_segment):
        self.waypoint_segments[index] = waypoint_segment

    def append(self, waypoint_segment):
        self.waypoint_segments.append(waypoint_segment)

    def save_str(self):
        point_lists = [self.waypoint_segments[i].segment[0] for i in range(0, len(self.waypoint_segments))]
        print("point lists: ", point_lists)
        return str(point_lists)

    @classmethod
    def init_empty(cls):
        waypoint_segments = WaypointSegments([])
        return waypoint_segments
