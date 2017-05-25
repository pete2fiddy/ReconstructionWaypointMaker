

class WaypointSegment:
    '''takes a length 2 list of cartesian waypoints, and the WaypointPathPolyPlane that it lies on'''
    def __init__(self, segment, path_plane):
        self.segment = segment
        self.path_plane = path_plane

    
