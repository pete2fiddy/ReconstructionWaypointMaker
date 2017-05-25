from math import pi, cos, sin
import numpy
import VectorOps.VectorMath as VectorMath


def normalize_angle_to_0_and_2pi(angle):
    number_to_make_positive = (int(abs(angle)/(2.0*pi)) + 1) * 2.0*pi
    return (angle + number_to_make_positive)%(2.0*pi)

'''assumes angle1 and angle2 are normalized. Angle1 is where you are looking from, and angle2 is where you are looking to'''
def angle_is_between_angles_radians(angle1, angle2, test_angle):
    normalized_test_angle = normalize_angle_to_0_and_2pi(test_angle)

    a1_sub = (angle1 - normalized_test_angle)
    a2_sub = (  normalized_test_angle - angle2)
    dtheta = get_dtheta_from_angle_to_angle(angle1, angle2)
    if dtheta > 0:
        a1_sub = ( normalized_test_angle - angle1 )
        a2_sub = (   normalized_test_angle- angle2)
    return (a1_sub < 0 and a2_sub > 0) or (a1_sub > 0 and a2_sub < 0)



def get_dtheta_from_angle_to_angle(angle1, angle2):
    smaller_angle = angle2
    bigger_angle = angle1
    min_angle_between = VectorMath.dot_angle_between(numpy.array([cos(angle1), sin(angle1)]), numpy.array([cos(angle2), sin(angle2)]))
    max_angle_between = 2*pi-min_angle_between

    if1_dist = abs((angle1 + min_angle_between)%(2*pi) - angle2)
    if2_dist = abs(normalize_angle_to_0_and_2pi(angle1 - min_angle_between) - angle2)
    if3_dist = abs(normalize_angle_to_0_and_2pi(angle1 + max_angle_between) - angle2)

    if if1_dist < if2_dist and if1_dist < if3_dist:
        return min_angle_between
    elif if2_dist < if1_dist and if2_dist < if3_dist:
        return - min_angle_between
    elif if3_dist < if1_dist and if3_dist < if2_dist:
        return max_angle_between
    else:
        return - max_angle_between




    '''
    smaller_angle = angle1 if angle1 < angle2 else angle2
    bigger_angle = angle2 if angle2 > angle1 else angle1
    normalized_test_angle = normalize_angle_to_0_and_2pi(test_angle)
    if abs(smaller_angle - bigger_angle) < pi:
        return (normalized_test_angle > smaller_angle and normalized_test_angle < bigger_angle) or (normalized_test_angle < smaller_angle and normalized_test_angle > bigger_angle)
    else:
        if angle2 < angle1:
            return (normalized_test_angle > angle1 and normalized_test_angle < angle2 + 2.0*pi) or (normalized_test_angle < angle1 and normalized_test_angle > angle2 + 2.0*pi)
        else:
            return (normalized_test_angle > angle1 + 2.0*pi and normalized_test_angle < angle2) or (normalized_test_angle < angle1 + 2.0*pi and normalized_test_angle > angle2)




    angle1_vector = numpy.array([cos(angle1), sin(angle1)])
    angle2_vector = numpy.array([cos(angle2), sin(angle2)])
    min_angle_between = numpy.dot(angle1_vector, angle2_vector)/(numpy.linalg.norm(angle1_vector)*numpy.linalg.norm(angle2_vector))
    max_angle_between = 2*pi - min_angle_between'''

    '''if angle2-angle1 < pi:
        return test_angle-angle1 < angle2-angle1
    else:
        return angle2-angle1 < test_angle - angle1'''





'''assumes angles are sorted from smallest to biggest inside each range, but not necessarily in order of ranges'''
def angle_in_angle_ranges(angle_ranges, angle):
    for i in range(0, len(angle_ranges)):
        if angle_is_between_angles_radians(angle_ranges[i][0], angle_ranges[i][1], angle):
            return True

    return False
