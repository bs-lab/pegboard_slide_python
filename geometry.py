from math import atan2, cos, sin, pi, sqrt
import numpy as np

np.seterr(all='raise')

# constants
DEBUG = False
RTOL = 0.0000001  # m


# --------------------------------------------------------------------------------------------------
class Peg:
    def __init__(self, x, y, r, xid):
        self.center = (x, y)
        self.radius = r
        self.id = xid


class FlatSurface:
    def __init__(self, pt1, pt2, xid):
        self.pt1 = pt1
        self.pt2 = pt2
        self.id = xid
        self.angle = atan2(pt2[1] - pt1[1], pt2[0] - pt1[0])
        self.vec = [pt2[0] - pt1[0], pt2[1] - pt1[1]]
        self.mag = np.linalg.norm(self.vec)
        self.unit_vec = self.vec / self.mag


class PuckData:
    def __init__(self, radius, max_time, xid):
        self.id = xid
        self.radius = radius  # radius (m)
        self.t = []           # time (s)
        self.x = []           # x-coordinate (m)
        self.y = []           # y-coordinate (m)
        self.v = []           # velocity (m/s)
        self.angle = []       # velocity angle (rad)
        self.inplay = True    # false if puck reaches the bottom
        self.final_time = max_time
        self.sliding_on = -1
        self.sliding_list = []

    def append(self, t, x, y, v, a, flat_id):
        self.t.append(t)
        self.x.append(x)
        self.y.append(y)
        self.v.append(v)
        self.angle.append(a)
        self.sliding_list.append(flat_id)


# --------------------------------------------------------------------------------------------------
def calc_dist_to_object(xy, flat):
    """"""
    dist_to_object = abs((flat.vec[1]) * xy[0] - (flat.vec[0]) * xy[1] +
                         flat.pt2[0] * flat.pt1[1] - flat.pt2[1] * flat.pt1[0]) / flat.mag

    return dist_to_object


# --------------------------------------------------------------------------------------------------
def get_line_circle_intersection(pt1, pt2, circle_center, circle_radius):
    """
    Returns the intersection point of a line-segment and circle. Since a line intersects a circle
    at two locations (unless tangent), the intersect point closer to the start of the line segment
    is chosen.

    INPUTS:
    -------
    pt1           -- first point defining the line segment
    pt2           -- second point defining the line segment
    circle_center -- center of the circle
    circle_radius -- radius of the circle

    RETURNS:
    --------
    the x- and y-coordinates of the nearer intersection point
    """
    # ----------------------------------------------------------------------------------------------
    def sign(x):
        """Helper function, equivalent to numpy.sign"""
        if x < 0:
            return -1
        else:
            return 1

    pt1[0] = pt1[0] - circle_center[0]
    pt1[1] = pt1[1] - circle_center[1]
    pt2[0] = pt2[0] - circle_center[0]
    pt2[1] = pt2[1] - circle_center[1]
    dx = pt2[0] - pt1[0]
    dy = pt2[1] - pt1[1]
    dr2 = dx**2 + dy**2
    D = pt1[0]*pt2[1] - pt2[0]*pt1[1]

    if DEBUG:
        print(f'GLCI: point #1 at: {pt1[0] + circle_center[0]} {pt1[1] + circle_center[1]}')
        print(f'GLCI: point #2 at: {pt2[0] + circle_center[0]} {pt2[1] + circle_center[1]}')
        print(f'GLCI: dr2 is {dr2} and D is {D}')
        print(f'GLCI: circle_radius is {circle_radius}')
        print(f'GLCI: circle center is {circle_center}')

    # if dr2 > 0.00001:
    try:
        AA = sqrt(circle_radius**2 * dr2 - D**2)

        # one of two intersection points
        x_int_a = (D * dy + sign(dy)*dx * AA) / (dr2)
        y_int_a = (-D * dx + abs(dy) * AA) / (dr2)

        # two of two intersection points
        x_int_b = (D * dy - sign(dy) * dx * AA) / (dr2)
        y_int_b = (-D * dx - abs(dy) * AA) / (dr2)

        # whichever point is closer to pt1 is used
        dist_a2 = (pt1[0] - x_int_a)**2 + (pt1[1] - y_int_a)**2
        dist_b2 = (pt1[0] - x_int_b)**2 + (pt1[1] - y_int_b)**2

        if dist_a2 < dist_b2:
            x_intercept = x_int_a + circle_center[0]
            y_intercept = y_int_a + circle_center[1]
            if DEBUG:
                print('GLCI: returning 1st result')
        else:
            x_intercept = x_int_b + circle_center[0]
            y_intercept = y_int_b + circle_center[1]
            if DEBUG:
                print('GLCI: returning 2nd result')

        # print('pt1:', pt1)
        # print('pt2:', pt2)
        # print('ctr:', circle_center)
        # print('intersects at ', x_int_a, y_int_a)
        # print('           or ', x_int_b, y_int_b)
        # print('dist_a2:', dist_a2)
        # print('dist_b2:', dist_b2)

    # else:
    except (FloatingPointError, Warning, ValueError, ZeroDivisionError):
        # the two provided points are too close together to get a line segment, so instead
        #   determine the point on the circle closest to the first point
        theta = atan2(pt1[1], pt1[0])
        x_intercept = circle_radius * cos(theta) + circle_center[0]
        y_intercept = circle_radius * sin(theta) + circle_center[1]
        if DEBUG:
            print(f'GLCI: points too close. Instead, returning {x_intercept} {y_intercept}')

    return x_intercept, y_intercept


# --------------------------------------------------------------------------------------------------
def calc_puck_flat_intersection(pt1, pt2, puck_radius, flat):
    """
    Returns the intersection point of a puck and line-segment.

    INPUTS:
    -------
    pt1         -- puck location at previous time step
    pt2         -- puck location at current time step
    puck_radius -- puck radius
    flat        -- instance of FlatSurface class which the puck is hitting

    RETURNS:
    --------
    x_intercept -- x-coordinate of the intersection point
    y_intercept -- y-coordinate of theintersection point
    """
    # distance from previous time step's puck to surface
    dist_1 = calc_dist_to_object(pt1, flat)

    # distance from current time step's puck to surface
    dist_2 = calc_dist_to_object(pt2, flat)

    # determine x-value at desired distance
    if dist_1 - dist_2 > 0.0000001:
        x_intercept = pt1[0] - (pt1[0] - pt2[0]) * (dist_1 - puck_radius) / (dist_1 - dist_2)
        y_intercept = pt1[1] - (pt1[1] - pt2[1]) * (dist_1 - puck_radius) / (dist_1 - dist_2)

    else:
        pt_vec = [pt2[0] - flat.pt1[0], pt2[1] - flat.pt1[1]]
        sdist = np.dot(flat.unit_vec, pt_vec) / flat.mag

        # get point on flat surface that is closest to pt2
        pt_on_flat = [flat.pt1[0] + flat.unit_vec[0] * sdist * flat.mag,
                      flat.pt1[1] + flat.unit_vec[1] * sdist * flat.mag]

        # cross product to determine which side of the flat surface the puck is located
        in_out = flat.unit_vec[0]*pt_vec[1] - flat.unit_vec[1]*pt_vec[0]
        if in_out > 0:
            x_intercept = puck_radius * cos(flat.angle + pi / 2) + pt_on_flat[0]
            y_intercept = puck_radius * sin(flat.angle + pi / 2) + pt_on_flat[1]
        else:
            x_intercept = puck_radius * cos(flat.angle - pi / 2) + pt_on_flat[0]
            y_intercept = puck_radius * sin(flat.angle - pi / 2) + pt_on_flat[1]
        # x_intercept = puck_radius * cos(flat.angle + pi / 2) + pt_on_flat[0]
        # y_intercept = puck_radius * sin(flat.angle + pi / 2) + pt_on_flat[1]

        # print(f'    pt1 {pt1}')
        # print(f'    pt2 {pt2}')
        # print(f'    dist_1 {dist_1}')
        # print(f'    dist_2 {dist_2}')
        # print(f'    flat pt1 {flat.pt1}')
        # print(f'    flat pt2 {flat.pt2}')
        # print(f'    unit_vec {flat.unit_vec}')
        # print(f'    pt_vec {pt_vec}')
        # print(f'    sdist {sdist}')
        # print(f'    pt_on_flat {pt_on_flat}')
        # print(f'    puck_radius {puck_radius}  flat.angle= {degrees(flat.angle)}')
        # print(f'    in_out {in_out}')
        # print(f'    zero div results: {x_intercept} {y_intercept}')

    return x_intercept, y_intercept


# --------------------------------------------------------------------------------------------------
def puck_near_flat(pt, radius, flat):
    """"""
    pt_vec = [pt[0] - flat.pt1[0], pt[1] - flat.pt1[1]]
    sdist = np.dot(flat.unit_vec, pt_vec) / flat.mag

    dist_to_flat = calc_dist_to_object(pt, flat)

    if 0 <= sdist <= 1 and dist_to_flat <= radius + RTOL:
        return True
    else:
        return False
