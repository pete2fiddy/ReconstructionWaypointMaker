from math import pi, cos


def normalize_angle_to_0_and_2pi(angle):
    number_to_make_positive = (int(abs(angle)/(2.0*pi)) + 1) * 2.0*pi
    return (angle + number_to_make_positive)%(2.0*pi)

'''assumes angle1 and angle2 are normalized'''
def angle_is_between_angles_radians(angle1, angle2, test_angle):
    '''angle_to_one = cos(test_angle - angle1)
    angle_to_two = cos(test_angle - angle2)
    angle_between = cos(angle2 - angle1)
    return (angle_to_one < angle_between and angle_to_two < angle_between)'''
    normalized_test_angle = normalize_angle_to_0_and_2pi(test_angle)
    if abs(angle1 - angle2) < pi:
        return (normalized_test_angle > angle1 and normalized_test_angle < angle2) or (normalized_test_angle < angle1 and normalized_test_angle > angle2)
    else:
        if angle2 < angle1:
            return (normalized_test_angle > angle1 and normalized_test_angle < angle2 + 2.0*pi) or (normalized_test_angle < angle1 and normalized_test_angle > angle2 + 2.0*pi)
        else:
            return (normalized_test_angle > angle1 + 2.0*pi and normalized_test_angle < angle2) or (normalized_test_angle < angle1 + 2.0*pi and normalized_test_angle > angle2)
