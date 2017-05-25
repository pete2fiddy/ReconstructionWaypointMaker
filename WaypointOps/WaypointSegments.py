

class WaypointSegments:

    def __init__(self, waypoint_segments):
        self.waypoint_segments = waypoint_segments
        self.init_segment_planes()
        print("waypoint segments: ", self.waypoint_segments)

    def init_segment_planes(self):
        self.segment_planes = []
        for i in range(0, len(self)):
            if self[i].path_plane not in self.segment_planes:
                self.segment_planes.append(self[i].path_plane)


    def __getitem__(self, index):
        return self.waypoint_segments[index]

    def __len__(self):
        return len(self.waypoint_segments)

    def __setitem__(self, index, waypoint_segment):
        self.waypoint_segments[index] = waypoint_segment

    def append(self, waypoint_segment):
        self.waypoint_segments.append(waypoint_segment)
        self.add_segment_plane_if_not_added(waypoint_segment)

    def add_segment_plane_if_not_added(self, waypoint_segment):
        if waypoint_segment.path_plane not in self.segment_planes:
            self.segment_planes.append(waypoint_segment.path_plane)

    def save_str(self):
        point_lists = [self.waypoint_segments[i].segment[0] for i in range(0, len(self.waypoint_segments))]
        return str(point_lists)

    @classmethod
    def init_empty(cls):
        waypoint_segments = WaypointSegments([])
        return waypoint_segments
