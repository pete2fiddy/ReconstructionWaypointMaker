

'''
line is a tuple of length two containing the first and second point, in that order.
Contain is whether or not to return a point if the line does not contain the point between its two points
(i.e. setting to false means it can return a point outside of the segment but along the same line)
'''
def get_x_of_line_at_y(line, y, contain):
    if contain and not line_contains_y(line, y):
        return None

    line_subtract = line[1]-line[0]
    y_ratio = float(y - line[0][1])/float(line_subtract[1])
    intersect_point = line[0] + (y_ratio * line_subtract)
    return intersect_point[0]

'''
does not return true if y rests on top of a corner
'''
def line_contains_y(line, y):
    return  ((line[0][1] > y and line[1][1] < y) or (line[0][1] < y and line[1][1] > y))
