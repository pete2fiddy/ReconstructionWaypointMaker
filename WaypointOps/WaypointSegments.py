

class WaypointSegments:

    def __init__(self, waypoint_segments):
        self.waypoint_segments = waypoint_segments
        self.init_segment_planes()

    def init_segment_planes(self):
        self.segment_planes = []
        for i in range(0, len(self)):
            if self[i].path_plane not in self.segment_planes:
                self.segment_planes.append(self[i].path_plane)

    def avoid_obstacles(self):
        new_segments = []
        for i in range(0, len(self)):
            new_segments.extend(self[i].fit_to_obstacles())
        self.waypoint_segments = new_segments
        

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
        out_str = ""
        for i in range(0, len(self.waypoint_segments)):
            out_str += str(self.waypoint_segments[i].segment[0].tolist())

        return out_str

    def copy(self):
        copied_segments = [self.waypoint_segments[i].copy() for i in range(0, len(self.waypoitn_segments))]
        return WaypointSegments(copied_segments)

    @classmethod
    def init_empty(cls):
        waypoint_segments = WaypointSegments([])
        return waypoint_segments
