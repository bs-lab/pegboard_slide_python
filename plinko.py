import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from math import atan2, cos, sin, ceil, degrees, pi, sqrt
from scipy import interpolate

# constants
RTOL = 0.0000001  # m
GC = 9.81         # m/s^2
TERM_VEL = 15.    # m/s
MIN_VEL = 0.05    # m/s
MAX_TIME = 10.0   # s
DT = 0.0001       # s
REDUCE_VEL_FACT = 0.6
REFRESH_FACT = 1.0
LOWEST_Y = -9
DEBUG = False
PEG_RADIUS = 0.25
PUCK_RADIUS = 0.10

# user inputs
angle = sys.argv[1]
vo = 10.0  # m/s
launch_angle = float(angle) * pi / 180  # radians

print("INPUTS:")
print('  launch angle', degrees(launch_angle))
print('  initial vel ', vo)
print('  terminal vel', TERM_VEL)
print('  max sim time', MAX_TIME)
print('')


# --------------------------------------------------------------------------------------------------
class Peg:
    def __init__(self, x, y, r, xid):
        self.center = (x, y)
        self.radius = r
        self.id = xid


# class FlatSurface:
#     def __init__(self):
#         self.start_pt = []
#         self.stop_pt = []


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

    def append(self, t, x, y, v, a):
        self.t.append(t)
        self.x.append(x)
        self.y.append(y)
        self.v.append(v)
        self.angle.append(a)


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
        pucks[-1].append(t, x_coord, y_coord, vo, puck_angle)

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
        pucks[-1].append(t, x_coord, y_coord, vo, launch_angle)
        print('p stuff', p, x_coord, y_coord, degrees(launch_angle))

    return pucks


# --------------------------------------------------------------------------------------------------
def calc_ricochet_angle(x, y, vel_angle, circle_center, circle_radius):
    """
    Calculates the angle at which the puck (located on the peg at {x, y} with a velocity angle of
    vel_angle), bounces of the circlulate object. Assumes elastic collision.

    INPUTS:
    -------
    x             -- x-coordinate of the puck (should be on the surface of the peg)
    y             -- y-coordinate of the puck (should be on the surface of the peg)
    vel_angle     -- velocity angle of the puck
    circle_center -- center location of circular object
    circle_radius -- radius of circular object

    RETURNS:
    --------
    the rebound (i.e. ricochet) angle
    """
    phi = atan2(y - circle_center[1], x - circle_center[0])
    alpha = phi - vel_angle - pi
    return phi + alpha


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
def align_to_framerate(puck, max_time, framerate, dilation=2.0):
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
        print("number of of frames:", len(frame_data))
        print("  max time", max_time)
        print("  # frames", num_frames)

    return frame_data


# --------------------------------------------------------------------------------------------------
def make_plot(pucks, avi_filename=""):
    """Creates either a matplotlib graph or an mp4 file showing the results of the analysis"""
    # ----------------------------------------------------------------------------------------------
    def init_plot():
        plt.title(f'angle={degrees(launch_angle)}')
        plt.xlim(-8, 8)
        plt.axis('equal')
        for peg in pegs:
            xx, yy = make_circle_points(peg.center, peg.radius)
            plt.plot(xx, yy, 'b')

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

    fps = 15

    frame_data = []
    max_time = max([x.final_time for x in pucks])

    for puck in pucks:
        frame_data.append(align_to_framerate(puck, max_time, fps))

    if avi_filename.strip():
        ani = animation.FuncAnimation(fig1, update_plot, zip(*frame_data),
                                      init_func=init_plot, blit=True, save_count=len(frame_data[0]))

        sys.stdout.write(f'saving to "{avi_filename}"...\n')
        Writer = animation.writers['ffmpeg']
        writer = Writer(fps=fps, metadata=dict(artist='bs-lab'), bitrate=1800)
        ani.save(avi_filename, writer=writer)
        sys.stdout.write(f'saved to "{avi_filename}"\n')

    else:
        init_plot()

        for f in range(len(frame_data[0])):
            # plt.pause(1/fps)
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
    x             -- x-coordinate of the puck (should be on the surface of the peg)
    y             -- y-coordinate of the puck (should be on the surface of the peg)
    circle_center -- center location of circular object
    circle_radius -- radius of circular object

    RETURNS:
    --------
    new_angle -- puck's velocity angle just after impact with the peg
    new_vel   -- puck's velocity magnitude just after impact with the peg
    """
    # get the ricochet angle
    new_angle = calc_ricochet_angle(x, y, old_angle, circle_center, circle_radius)
    angle_change = new_angle - old_angle
    # reduce the puck velocity arbitrarily by REDUCE_VEL_FACT and the angle of impact
    new_vel = old_vel * (1 - (1 - REDUCE_VEL_FACT) * abs(sin(angle_change / 2)))
    if new_vel < MIN_VEL:
        new_vel = MIN_VEL

    print(f"  rebound: {degrees(new_angle):8.4f}")

    if DEBUG:
        sys.stdout.write(f'old & new angles {degrees(old_angle)} {degrees(new_angle)}\n')
        sys.stdout.write(f'{old_vel} multiplied by of ')
        sys.stdout.write(f'{(1 - (1 - REDUCE_VEL_FACT) * abs(sin(angle_change / 2)))}')
        sys.stdout.write(f' to get {new_vel}\n')

    return new_angle, new_vel


# --------------------------------------------------------------------------------------------------
def get_line_circle_intersection(pt1, pt2, circle_center, circle_radius):
    """
    Returns the intersection point of a line-segment and circle. Since a line intersects a circle
    at two locations (unless tangent), the intersect point closer to the start of the line segment
    is chosen.

    INPUTS:
    -------
    pt1           -- first point defning the line segment
    pt2           -- second point defning the line segment
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

    # print('pt1:', pt1)
    # print('pt2:', pt2)
    # print('ctr:', circle_center)
    # print('intersects at ', x_int_a, y_int_a)
    # print('           or ', x_int_b, y_int_b)
    # print('dist_a2:', dist_a2)
    # print('dist_b2:', dist_b2)

    if dist_a2 < dist_b2:
        return x_int_a + circle_center[0], y_int_a + circle_center[1]
    else:
        return x_int_b + circle_center[0], y_int_b + circle_center[1]


# --------------------------------------------------------------------------------------------------
def update_puck(puck, pucks):
    """Updates the PuckData instance based on its projectile motion (and possible ricochets after
       a time interval of DT."""
    # do not exceed terminal velocity
    if puck.v[-1] >= TERM_VEL:
        accel = 0
    else:
        accel = GC

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

    # check if position is inside or contacting surface
    for peg in pegs:
        dist_to_center = sqrt((x - peg.center[0])**2 + (y - peg.center[1])**2)
        if dist_to_center <= (peg.radius + puck.radius + RTOL):
            if dist_to_center < (peg.radius + puck.radius + RTOL):
                print(f"inside peg!!!!!!!!! {peg.id}")
                # re-calculate current position so that it is on the surface, not inside
                print('  old puck coords', x, y)
                x, y = get_line_circle_intersection([puck.x[-1], puck.y[-1]],
                                                    [x, y], peg.center, peg.radius + puck.radius)
                print('  new puck coords', x, y)

            # calculate rebound angle & reduced velocity
            new_angle, new_vel = calc_rebound(new_angle, new_vel, x, y, peg.center, peg.radius)

    # check if position is inside or contacting other pucks
    for other_puck in pucks:
        if other_puck.id >= puck.id:
            continue
        dist_to_center = sqrt((x - other_puck.x[-1])**2 + (y - other_puck.y[-1])**2)
        if dist_to_center <= (other_puck.radius + puck.radius + RTOL) and y < -0.5:
            if dist_to_center < (other_puck.radius + puck.radius + RTOL):
                print(f"inside other puck!!!!!!!!! {other_puck.id}")
                # re-calculate current position so that it is on the surface, not inside
                print('  old puck coords', x, y)
                x, y = get_line_circle_intersection([puck.x[-1], puck.y[-1]],
                                                    [x, y], [other_puck.x[-1], other_puck.y[-1]],
                                                    other_puck.radius + puck.radius)
                print('  new puck coords', x, y)

            # calculate puck's rebound angle & reduced velocity
            new_angle, new_vel = calc_rebound(new_angle, new_vel, x, y,
                                              [other_puck.x[-1], other_puck.y[-1]],
                                              other_puck.radius)

            # calculate rebound angle and reduced velocity of other puck
            other_angle, other_vel = calc_rebound(other_puck.angle[-1], other_puck.v[-1],
                                                  other_puck.x[-1], other_puck.y[-1],
                                                  [x, y], puck.radius)
            other_puck.v[-1] = other_vel
            other_puck.angle[-1] = other_angle

    puck.append(t, x, y, new_vel, new_angle)

    if DEBUG:
        sys.stdout.write(f"{t:9.4f} {x:9.4f} {y:9.4f} {degrees(new_angle):9.4f} ")
        sys.stdout.write(f"{new_vel:9.4f} {accel:9.4f}\n")

    return puck


# --------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    """"""
    vel = vo
    t = 0
    x = 0
    y = 0

    pegs = create_pegs()

    num_pucks = 4
    last_puck_angle = pi - launch_angle - 0.2
    pucks = create_puck_spray(num_pucks, launch_angle, last_puck_angle)
    # pucks = create_puck_shower(num_pucks, -6, 6)

    if DEBUG:
        sys.stdout.write("     time         x         y     angle       vel     accel\n")

    while True:
        if t > MAX_TIME:
            sys.stdout.write(f'\nbreaking at time = {t:12.8f}\n\n')
            break

        for p, puck in enumerate(pucks):
            if not puck.inplay:
                puck.append(t, puck.x[-1], puck.y[-1], puck.v[-1], puck.angle[-1])
                continue
            if puck.y[-1] <= LOWEST_Y:
                sys.stdout.write(f'\npuck # {p} reached bottom at time={t:8.4f}\n\n')
                puck.inplay = False
                puck.final_time = t
                # break

            puck = update_puck(puck, pucks)

        t += DT

    if DEBUG:
        # print out results to the screen
        for puck in pucks:
            sys.stdout.write("      time       x-pos       y-pos         vel\n")
            for t, x, y, v in zip(puck.t, puck.x, puck.y, puck.v):
                sys.stdout.write(f"{t:10.4f} {x:11.8f} {y:11.8f} {v:11.8f}\n")
            print('maxtime', max(puck.t))
            print('')

    # create matplotlib figure or mp4 file
    make_plot(pucks, avi_filename="outfile.mp4")
    # make_plot(pucks)
