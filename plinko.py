import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from math import atan2, cos, sin, ceil, degrees, pi, sqrt
import numpy as np
from scipy import interpolate

np.seterr(all='raise')

# constants
RTOL = 0.0000001  # m
GC = 9.81         # m/s^2
TERM_VEL = 15.    # m/s
MIN_VEL = 0.10    # m/s
TRANSITION_VEL = 0.8  # m/s
MAX_TIME = 10.0   # s
DT = 0.0001       # s
REDUCE_VEL_FACT = 0.6
REFRESH_FACT = 1.0
LOWEST_Y = -10
DEBUG = False
DEBUG_2 = False
PEG_RADIUS = 0.25
PUCK_RADIUS = 0.10
FPS = 30
DILATION = 2
OUTFILE_MP4 = "outfile.mp4"

# user inputs
VO = 10.0  # m/s
NUM_PUCKS = 4
ANGLE = sys.argv[1]

launch_angle = float(ANGLE) * pi / 180  # radians

print("INPUTS:")
print('  launch angle', degrees(launch_angle))
print('  initial vel ', VO)
print('  terminal vel', TERM_VEL)
print('  max sim time', MAX_TIME)
print('')


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
    def __init__(self, radius, xid):
        self.id = xid
        self.radius = radius  # radius (m)
        self.t = []           # time (s)
        self.x = []           # x-coordinate (m)
        self.y = []           # y-coordinate (m)
        self.v = []           # velocity (m/s)
        self.angle = []       # velocity angle (rad)
        self.inplay = True    # false if puck reaches the bottom
        self.final_time = MAX_TIME
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
def create_pegs():
    """Creates list of Peg instances"""
    peg_rows = [-2, -4, -6, -8]
    peg_cols = [-6, -4, -2, 0, 2, 4, 6]

    pegs = []
    pid = -1
    for r in peg_rows:
        for c in peg_cols:
            pid += 1
            pegs.append(Peg(c, r, PEG_RADIUS, pid))

    # include offset rows
    for r in peg_rows:
        for c in peg_cols:
            pid += 1
            pegs.append(Peg(c+1, r+1, PEG_RADIUS, pid))

    return pegs


# --------------------------------------------------------------------------------------------------
def create_puck_spray(num_pucks, first_angle, final_angle):
    """Creates list of PuckData instances, with evenly spaced initial angles"""
    pucks = []

    if num_pucks > 1:
        del_angle = (2 * pi - final_angle + first_angle) / (num_pucks - 1)
    else:
        del_angle = 0

    puck_angle = first_angle + del_angle
    x_coord = 0
    y_coord = 0
    t = 0

    for p in range(num_pucks):
        pucks.append(PuckData(PUCK_RADIUS, p))
        puck_angle -= del_angle
        print(f'  puck # {p}, angle {degrees(puck_angle):16.12f}')
        pucks[-1].append(t, x_coord, y_coord, VO, puck_angle, -1)

    print('')

    return pucks


# --------------------------------------------------------------------------------------------------
def create_puck_shower(num_pucks, left_x, right_x):
    """Creates list of PuckData instances, with evenly spaced launch locations"""
    pucks = []
    if num_pucks > 1:
        x_spacing = (right_x - left_x) / (num_pucks - 1)
    else:
        x_spacing = 0

    x_coord = left_x - x_spacing
    y_coord = 0
    t = 0

    for p in range(num_pucks):
        pucks.append(PuckData(PUCK_RADIUS, p))
        x_coord += x_spacing
        pucks[-1].append(t, x_coord, y_coord, VO, launch_angle)
        print('p stuff', p, x_coord, y_coord, degrees(launch_angle))

    return pucks


# --------------------------------------------------------------------------------------------------
def create_flat_surfaces():
    """"""
    lh_wall_x = -7
    rh_wall_x = 8
    max_wall_y = 2
    min_wall_y = -9

    flat_surfaces = []

    # create sides
    flat_surfaces.append(FlatSurface([lh_wall_x, max_wall_y], [lh_wall_x, min_wall_y], 0))
    flat_surfaces.append(FlatSurface([rh_wall_x, max_wall_y], [rh_wall_x, min_wall_y], 1))

    # create bottom
    flat_surfaces.append(FlatSurface([lh_wall_x, min_wall_y], [-1, min_wall_y - 0.5], 2))
    # flat_surfaces.append(FlatSurface([rh_wall_x, min_wall_y], [2, min_wall_y - 0.5], 3))
    flat_surfaces.append(FlatSurface([rh_wall_x, min_wall_y], [0, min_wall_y - 0.5], 3))

    flat = FlatSurface([-3, -3], [0, -3.75], 4)
    # flat = FlatSurface([-3, -3], [0, -4], 4)
    flat_surfaces.append(flat)

    for flat in flat_surfaces:
        print(f'  flat angle: {flat.id} {degrees(flat.angle)}')
    print('')

    return flat_surfaces


# --------------------------------------------------------------------------------------------------
def make_circle_points(center, radius):
    """only used for plotting, not analysis"""
    theta = 0
    x = []
    y = []
    while True:
        if theta > 2 * pi:
            break

        theta += 2 * pi * 0.01
        x.append(radius * cos(theta) + center[0])
        y.append(radius * sin(theta) + center[1])

    return(x, y)


# --------------------------------------------------------------------------------------------------
def align_to_framerate(puck, max_time, framerate, dilation=1.0):
    """
    Interpolates the position puck data to align with the mp4 framerate.

    INPUTS:
    -------
    puck      -- PuckData instance
    max_time  -- max simulation time (when either all pucks are out of play or MAX_TIME reached)
    framerate -- frame rate (frames per second)

    OPTIONAL INPUT:
    ---------------
    dilation -- a factor used to slow down or speed up video.  A value above 1 slows down ,
                  below 1 speeds up

    RETURNS:
    --------
    frame_data -- list of interpolated positional puck data at specific time frames
    """
    interp_func_x = interpolate.interp1d(puck.t, puck.x)
    interp_func_y = interpolate.interp1d(puck.t, puck.y)
    num_frames = ceil(max_time * framerate * dilation)
    frame_data = []
    for f in range(num_frames):
        t = f / (framerate * dilation)
        x = interp_func_x(t)
        y = interp_func_y(t)
        frame_data.append((x.tolist(), y.tolist()))

    if DEBUG:
        print("number of frames:", len(frame_data))
        print("  max time", max_time)
        print("  # frames", num_frames)

    return frame_data


# --------------------------------------------------------------------------------------------------
def make_plot(pucks, avi_filename=""):
    """Creates either a matplotlib graph or an mp4 file showing the results of the analysis"""
    # ----------------------------------------------------------------------------------------------
    def init_plot():
        plt.title(f'angle={degrees(launch_angle):12.8f}')
        plt.xlim(-9, 9)
        plt.axis('equal')
        for peg in pegs:
            xx, yy = make_circle_points(peg.center, peg.radius)
            plt.plot(xx, yy, 'b')

        for flats in flat_surfaces:
            plt.plot([flats.pt1[0], flats.pt2[0]], [flats.pt1[1], flats.pt2[1]], 'r')

        return puck_dot,

    # ----------------------------------------------------------------------------------------------
    def update_plot(xy):
        xs = []
        ys = []
        for x in xy:
            xs.append(x[0])
            ys.append(x[1])
            # if DEBUG:
            #     print('x in xy:', x)

        puck_dot.set_data(xs, ys)
        return puck_dot,

    # ----------------------------------------------------------------------------------------------
    fig1 = plt.figure()
    puck_dot, = plt.plot([], [], 'ro', ms=50 * PUCK_RADIUS)

    frame_data = []
    max_time = max([x.final_time for x in pucks])

    for puck in pucks:
        frame_data.append(align_to_framerate(puck, max_time, FPS, dilation=DILATION))

    if avi_filename.strip():
        ani = animation.FuncAnimation(fig1, update_plot, zip(*frame_data),
                                      init_func=init_plot, blit=True, save_count=len(frame_data[0]))

        sys.stdout.write(f'saving to "{avi_filename}"...\n')
        Writer = animation.writers['ffmpeg']
        writer = Writer(fps=FPS, metadata=dict(artist='bs-lab'), bitrate=1800)
        ani.save(avi_filename, writer=writer)
        sys.stdout.write(f'saved to "{avi_filename}"\n')

    else:
        init_plot()

        for f in range(len(frame_data[0])):
            # plt.pause(1/FPS)
            plt.pause(0.04)
            for p, puck in enumerate(pucks):
                x = frame_data[p][f][0]
                y = frame_data[p][f][1]
                plt.plot(x, y, 'r.')
                xx, yy = make_circle_points([x, y], puck.radius)
                plt.plot(xx, yy, 'r')
            # plt.draw()

        # prevent figure from disappearing
        plt.show()


# --------------------------------------------------------------------------------------------------
def get_position(vel, angle, t, accel):
    """
    Get puck position using ballistic trajectory equations.

    INPUTS:
    -------
    vel   -- initial puck velocity magnitude
    angle -- initial puck velocity angle
    t     -- time elapsed
    accel -- initial puck acceleration

    RETURNS:
    --------
    x     -- updated puck x-coordinate after elapsed time
    y     -- updated puck y-coordinate after elapsed time
    """
    x = vel * t * cos(angle)
    y = vel * t * sin(angle) - (0.5 * accel * t**2)
    return x, y


# --------------------------------------------------------------------------------------------------
def get_velocity(v_init, angle, t, accel):
    """
    Get puck velocity using ballistic trajectory equations.

    INPUTS:
    -------
    v_init -- initial puck velocity magnitude
    angle  -- initial puck velocity angle
    t      -- time elapsed
    accel  -- initial puck acceleration

    RETURNS:
    --------
    updated puck velocity magnitude after elapsed time
    """
    vx = v_init * cos(angle)
    vy = v_init * sin(angle) - (accel * DT)
    return sqrt(vx**2 + vy**2)


# --------------------------------------------------------------------------------------------------
def calc_rebound(old_angle, old_vel, x, y, circle_center, circle_radius):
    """
    Calculates the puck velocity after rebounding off of a circular object (like a peg or
      other puck).

    INPUTS:
    -------
    old_angle     -- puck's velocity angle just prior to impact with the peg
    old_vel       -- puck's velocity magnitude prior to impact with the peg
    x             -- x-coordinate of the puck (whose surface should be on the surface of the peg)
    y             -- y-coordinate of the puck (whose surface should be on the surface of the peg)
    circle_center -- center location of circular object
    circle_radius -- radius of circular object

    RETURNS:
    --------
    new_angle -- puck's velocity angle just after impact with the peg
    new_vel   -- puck's velocity magnitude just after impact with the peg
    """
    # get the ricochet angle
    phi = atan2(y - circle_center[1], x - circle_center[0])
    alpha = phi - old_angle - pi
    new_angle = phi + alpha
    angle_change = new_angle - old_angle

    # reduce the puck velocity arbitrarily by REDUCE_VEL_FACT and the angle of impact
    new_vel = old_vel * (1 - (1 - REDUCE_VEL_FACT) * abs(sin(angle_change / 2)))
    if new_vel < MIN_VEL:
        new_vel = MIN_VEL

    if DEBUG_2:
        print(f"  rebound: {degrees(new_angle):8.4f}")

    if DEBUG:
        sys.stdout.write(f'old & new angles {degrees(old_angle)} {degrees(new_angle)}\n')
        sys.stdout.write(f'{old_vel} multiplied by of ')
        sys.stdout.write(f'{(1 - (1 - REDUCE_VEL_FACT) * abs(sin(angle_change / 2)))}')
        sys.stdout.write(f' to get {new_vel}\n')

    return new_angle, new_vel


# --------------------------------------------------------------------------------------------------
def calc_rebound_flat(old_angle, old_vel, x, y, flat):
    """
    Calculates the puck velocity after rebounding off a flat surface.

    INPUTS:
    -------
    old_angle -- puck's velocity angle just prior to impact with the flat surface
    old_vel   -- puck's velocity magnitude prior to impact with the flat surface
    x         -- x-coordinate of the puck (whose surface should be on the flat surface)
    y         -- y-coordinate of the puck (whose surface should be on the flat surface)
    flat      -- instance of FlatSurface class which the puck is hitting

    RETURNS:
    --------
    new_angle -- puck's velocity angle just after impact with the flat surface
    new_vel   -- puck's velocity magnitude just after impact with the flat surface
    """
    new_angle = 2 * flat.angle - old_angle
    angle_change = new_angle - old_angle

    # reduce the puck velocity arbitrarily by REDUCE_VEL_FACT and the angle of impact
    new_vel = old_vel * (1 - (1 - REDUCE_VEL_FACT) * abs(sin(angle_change / 2)))
    if new_vel < MIN_VEL:
        new_vel = MIN_VEL

    return new_angle, new_vel


# --------------------------------------------------------------------------------------------------
def slide_down_flat(prev_vel, prev_x, prev_y, puck_radius, flat):
    """
    Calculates the position of the puck using the equation, accel = dv / dt

    INPUTS:
    -------
    prev_vel    -- puck velocity magnitude at previous time step
    prev_x      -- x-coordinate of the puck at the previous time step
    prev_y      -- y-coordinate of the puck at the previous time step
    puck_radius -- radius of the puck
    flat        -- instance of FlatSurface class which the puck is sliding along

    RETURNS:
    --------
    phi      -- puck velocity angle at current time step
    curr_vel -- puck velocity magnitude at current time step
    curr_x   -- x-coordinate of the puck at the current time step
    curr_y   -- y-coordinate of the puck at the current time step
    sdist    -- percentage along the flat surface where the puck is located
                  (0 would be at beginning of surface, 1 would be at end)
    """
    phi = flat.angle
    if 0 < phi < pi:
        phi -= pi
    a_parallel = abs(GC * sin(phi))

    # accel = (vf - vi) / dt --> vf = accel * dt + vi
    curr_vel = prev_vel + a_parallel * DT

    # get normalized ratio of where previous coordinate is projected onto the flat surface
    pt_vec = [prev_x - flat.pt1[0], prev_y - flat.pt1[1]]
    sdist = np.dot(flat.unit_vec, pt_vec) / flat.mag

    if DEBUG_2:
        print(f'  sliding down flat #{flat.id}')
        print(f'  a_parallel {a_parallel}')
        print(f'  prev_vel {prev_vel}')
        print(f'  curr_vel {curr_vel}')
        print(f'  previous x&y {prev_x} {prev_y}')
        print(f'  unit vector {flat.unit_vec}')
        print(f'  flat.angle {degrees(flat.angle)}')
        print(f'  phi {degrees(phi)}')
        print(f'  angle used {degrees(pi + phi)}')
        print(f'  sdist {sdist}')

    # WARNING -- needs cross product to determine if it is plus or minus pi/2
    # ensure that previous location was on the surface...
    # pt_on_flat = [flat.pt1[0] + flat.unit_vec[0] * sdist * flat.mag,
    #               flat.pt1[1] + flat.unit_vec[1] * sdist * flat.mag]
    # prev_x = puck_radius * cos(phi + pi / 2) + pt_on_flat[0]
    # prev_y = puck_radius * sin(phi + pi / 2) + pt_on_flat[1]
    # if DEBUG_2:
    #     print(f'  pt_on_flat {pt_on_flat}')
    #     print(f'      now x&y {prev_x} {prev_y}')

    curr_x = prev_x + prev_vel * DT * cos(phi)
    curr_y = prev_y + prev_vel * DT * sin(phi)

    if DEBUG_2:
        print(f'  current x&y {curr_x} {curr_y}')

    return phi, curr_vel, curr_x, curr_y, sdist


# --------------------------------------------------------------------------------------------------
def puck_collision(puck_1_angle, puck_1_vel, puck_1_x, puck_1_y, puck_2):
    """
    Calculates the resulting velocities of two pucks after they collide.

    INPUTS:
    -------
    puck_1_angle -- velocity angle of 1st puck
    puck_1_vel   -- velocity magnitude of 1st puck
    puck_1_x     -- x-coordinate of 1st puck
    puck_1_y     -- y-coordinate of 1st puck
    puck_2       -- PuckData instance of 2nd puck

    RETURNS:
    --------
    angle_1 -- velocity angle of 1st puck after impact
    vel_1   -- velocity magnitude of 1st puck after impact
    angle_2 -- velocity angle of 2nd puck after impact
    vel_2   -- velocity magnitude of 2nd puck after impact
    """
    # get angle of contact
    contact_angle = atan2(puck_1_y - puck_2.y[-1], puck_1_x - puck_2.x[-1])

    # useful terms
    del1 = puck_1_angle - contact_angle
    del2 = puck_2.angle[-1] - contact_angle
    capp = contact_angle + 0.5 * pi

    # get 1st puck's velocity after elastic collision
    vx = puck_2.v[-1] * cos(del2) * cos(contact_angle) + puck_1_vel * sin(del1) * cos(capp)
    vy = puck_2.v[-1] * cos(del2) * sin(contact_angle) + puck_1_vel * sin(del1) * sin(capp)
    vel_1 = sqrt(vx**2 + vy**2)
    angle_1 = atan2(vy, vx)

    # get 2nd puck's velocity after elastic collision
    vx = puck_1_vel * cos(del1) * cos(contact_angle) + puck_2.v[-1] * sin(del2) * cos(capp)
    vy = puck_1_vel * cos(del1) * sin(contact_angle) + puck_2.v[-1] * sin(del2) * sin(capp)
    vel_2 = sqrt(vx**2 + vy**2)
    angle_2 = atan2(vy, vx)

    return angle_1, vel_1, angle_2, vel_2


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

    if DEBUG_2:
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
            if DEBUG_2:
                print('GLCI: returning 1st result')
        else:
            x_intercept = x_int_b + circle_center[0]
            y_intercept = y_int_b + circle_center[1]
            if DEBUG_2:
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
        if DEBUG_2:
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


# --------------------------------------------------------------------------------------------------
def update_puck(puck, pucks, t):
    """Updates the PuckData instance based on its projectile motion (and possible ricochets after
       a time interval of DT."""
    # do not exceed terminal velocity
    if puck.v[-1] >= TERM_VEL:
        accel = 0
    else:
        accel = GC

    if puck.sliding_on >= 0:
        new_angle, new_vel, x, y, sdist = slide_down_flat(puck.v[-1], puck.x[-1], puck.y[-1],
                                                          puck.radius,
                                                          flat_surfaces[puck.sliding_on])
        if sdist > 1:
            puck.sliding_on = -1

    else:
        # update the relative position
        x_rel, y_rel = get_position(puck.v[-1], puck.angle[-1], DT, accel)

        # update the velocity
        new_vel = get_velocity(puck.v[-1], puck.angle[-1], DT, accel)

        if new_vel >= TERM_VEL:
            accel = 0
            new_vel = TERM_VEL

            # recalculate relative position based on terminal velocity and no acceleration
            x_rel, y_rel = get_position(TERM_VEL, puck.angle[-1], DT, accel)

        # update velocity's angle
        x_inc, y_inc = get_position(puck.v[-1], puck.angle[-1], DT - DT / 100, accel)
        new_angle = atan2(y_rel - y_inc, x_rel - x_inc)

        # update the absolute position
        x = puck.x[-1] + x_rel
        y = puck.y[-1] + y_rel

    # check if position is inside or contacting flat surface
    for flat in flat_surfaces:
        if not puck_near_flat([x, y], puck.radius, flat):
            continue

        dist_to_flat = calc_dist_to_object([x, y], flat)
        if dist_to_flat <= puck.radius + RTOL:
            if new_vel < TRANSITION_VEL:
                puck.sliding_on = flat.id

            if dist_to_flat < puck.radius + RTOL:
                if DEBUG_2:
                    print(f'distance to flat surface = {dist_to_flat}')
                    print(f'{puck.id} inside surface!!!!!!!!! {flat.id}')
                    print('  old puck coords', x, y)
                    print(' new_vel = ', new_vel)

                # re-calculate current position so that it is on the surface, not inside
                x, y = calc_puck_flat_intersection([puck.x[-1], puck.y[-1]], [x, y],
                                                   puck.radius, flat)
                if DEBUG_2:
                    print('  new puck coords', x, y)

            # calculate rebound angle & reduced velocity
            if puck.sliding_on < 0:
                new_angle, new_vel = calc_rebound_flat(new_angle, new_vel, x, y, flat)

            if new_vel < TRANSITION_VEL or puck.sliding_on >= 0:
                puck.sliding_on = flat.id

        else:
            puck.sliding_on = -1

    # check if position is inside or contacting peg surface
    for peg in pegs:
        dist_to_center = sqrt((x - peg.center[0])**2 + (y - peg.center[1])**2)
        if dist_to_center <= (peg.radius + puck.radius + RTOL):
            puck.sliding_on = -1
            if dist_to_center < (peg.radius + puck.radius + RTOL):
                if DEBUG_2:
                    print(f"{puck.id} inside peg!!!!!!!!! {peg.id}")
                    print('  old puck coords', x, y)
                # re-calculate current position so that it is on the surface, not inside
                x, y = get_line_circle_intersection([puck.x[-1], puck.y[-1]],
                                                    [x, y], peg.center, peg.radius + puck.radius)
                if DEBUG_2:
                    print('  new puck coords', x, y)

            # calculate rebound angle & reduced velocity
            new_angle, new_vel = calc_rebound(new_angle, new_vel, x, y, peg.center, peg.radius)

    # check if position is inside or contacting other pucks
    for other_puck in pucks:
        if other_puck.id >= puck.id:
            # if other puck has higher ID, then it hasn't been processed yet in this frame
            continue

        if not other_puck.inplay:
            # other puck is no longer bouncing about, so treat it as non-existent
            continue

        dist_to_center = sqrt((x - other_puck.x[-1])**2 + (y - other_puck.y[-1])**2)
        if dist_to_center <= (other_puck.radius + puck.radius + RTOL) and t > 0.2:
            if dist_to_center < (other_puck.radius + puck.radius + RTOL):
                if DEBUG_2:
                    print(f"{puck.id} inside other puck!!!!!!!!! {other_puck.id}")
                    print('  old puck coords', x, y)
                # re-calculate current position so that it is on the surface, not inside
                x, y = get_line_circle_intersection([puck.x[-1], puck.y[-1]],
                                                    [x, y], [other_puck.x[-1], other_puck.y[-1]],
                                                    other_puck.radius + puck.radius)
                if DEBUG_2:
                    print('  new puck coords', x, y)

            # should puck_collision use 'new_angle' and 'new_vel', 'x', and 'y' ???
            new_angle, new_vel, other_angle, other_vel = puck_collision(new_angle, new_vel,
                                                                        x, y, other_puck)
            other_puck.v[-1] = other_vel
            other_puck.angle[-1] = other_angle

            # or, could treat the other puck as an immovable object, like a peg...
            # new_angle, new_vel = calc_rebound(new_angle, new_vel, x, y,
            #                                   [other_puck.x[-1], other_puck.y[-1]],
            #                                   other_puck.radius)

    puck.append(t, x, y, new_vel, new_angle, puck.sliding_on)

    if DEBUG:
        sys.stdout.write(f"---- {puck.id} {t:9.4f} {x:9.4f} {y:9.4f} {degrees(new_angle):9.4f} ")
        sys.stdout.write(f"{new_vel:9.4f} {accel:9.4f} {puck.sliding_on}\n")

    return puck


# --------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    """"""
    vel = VO
    t = 0
    x = 0
    y = 0

    pegs = create_pegs()
    flat_surfaces = create_flat_surfaces()

    last_puck_angle = pi - launch_angle - 0.2
    pucks = create_puck_spray(NUM_PUCKS, launch_angle, last_puck_angle)
    # pucks = create_puck_shower(num_pucks, -6, 6)

    if DEBUG:
        sys.stdout.write("          time         x         y     angle       vel     accel")
        sys.stdout.write("  sliding_on\n")

    while True:
        # end simulation if it is taking too long
        if t > MAX_TIME:
            sys.stdout.write(f'\nbreaking at time = {t:12.8f}\n\n')
            break

        # end simulation if all pegs have reached bottom
        keep_going = False
        for puck in pucks:
            if puck.inplay:
                keep_going = True
        if not keep_going:
            sys.stdout.write(f'\nbreaking at time = {t:12.8f}\n\n')
            break

        for p, puck in enumerate(pucks):
            if not puck.inplay:
                puck.append(t, puck.x[-1], puck.y[-1], puck.v[-1], puck.angle[-1], -1)
                continue
            if puck.y[-1] <= LOWEST_Y:
                sys.stdout.write(f'\npuck # {p} reached bottom at time={t:8.4f}\n\n')
                puck.inplay = False
                puck.final_time = t
                # break

            puck = update_puck(puck, pucks, t)

        t += DT

    if DEBUG:
        # print out results to the screen
        for puck in pucks:
            sys.stdout.write("      time       x-pos       y-pos         vel\n")
            for t, x, y, v, s in zip(puck.t, puck.x, puck.y, puck.v, puck.sliding_list):
                sys.stdout.write(f"{t:10.4f} {x:11.8f} {y:11.8f} {v:11.8f} {puck.id} {s}\n")
            print('maxtime', max(puck.t))
            print('')

    # create matplotlib figure or mp4 file
    make_plot(pucks, avi_filename=OUTFILE_MP4)
    # make_plot(pucks)
