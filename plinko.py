import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from math import atan2, cos, sin, ceil, degrees, pi, sqrt
from scipy import interpolate

# constants
RTOL = 0.0000001 # m
GC = 9.81        # m/s^2
TERM_VEL = 15.   # m/s
MAX_TIME = 8.0   # s
DT = 0.0001       # s
REDUCE_VEL_FACT = 0.6
REFRESH_FACT = 1.0
LOWEST_Y = -9

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
    def __init__(self, x, y, r):
        self.center = (x, y)
        self.radius = r


class PuckData:
    def __init__(self):
        self.t = []   # time (s)
        self.x = []   # x-position (m)
        self.y = []   # y-position (m)
        self.v = []   # velocity (m/s)
        self.angle = []  # velocity angle (rad)

    def append(self, t, x, y, v, a):
        self.t.append(t)
        self.x.append(x)
        self.y.append(y)
        self.v.append(v)
        self.angle.append(a)


# --------------------------------------------------------------------------------------------------
def create_pegs():
    peg_rows = [-2, -4, -6, -8]
    peg_cols = [-6, -4, -2, 0, 2, 4, 6]

    pegs = []
    for r in peg_rows:
        for c in peg_cols:
            pegs.append(Peg(c, r, 0.25))

    # include offset rows
    for r in peg_rows:
        for c in peg_cols:
            pegs.append(Peg(c+1, r+1, 0.25))

    return pegs


# --------------------------------------------------------------------------------------------------
def calc_ricochet_angle(x, y, vel_angle, peg):
    """"""
    phi = atan2(y - peg.center[1], x - peg.center[0])
    alpha = phi - vel_angle - pi
    return phi + alpha


# --------------------------------------------------------------------------------------------------
def calc_time_step_to_impact(t_curr, ddt, x_prev, y_prev, peg):
    """find ddt that would have resulted in ball just hitting the peg"""
    d_prev = sqrt((x_prev - peg.center[0])**2 + (y_prev - peg.center[1])**2)
    d_curr = dist_to_center

    t_prev = t_curr - ddt
    t_need = t_prev - (t_prev - t_curr) * (d_prev - peg.radius) / (d_prev - d_curr)
    t_curr -= ddt
    ddt = t_need - t_prev

    return t_curr, ddt


# --------------------------------------------------------------------------------------------------
def make_circle_points(peg):
    """only used for plotting, not analysis"""
    theta = 0
    x = []
    y = []
    while True:
        if theta > 2 * pi:
            break

        theta += 2 * pi * 0.01
        x.append(peg.radius * cos(theta) + peg.center[0])
        y.append(peg.radius * sin(theta) + peg.center[1])

    return(x, y)


# --------------------------------------------------------------------------------------------------
def align_to_framerate(puck_data, framerate=30, dilation=3):
    """"""
    interp_func_x = interpolate.interp1d(puck_data.t, puck_data.x)
    interp_func_y = interpolate.interp1d(puck_data.t, puck_data.y)
    max_time = max(puck_data.t)
    num_frames = ceil(max_time * framerate * dilation)
    frame_data = []
    for f in range(num_frames):
        t = f / (framerate * dilation)
        frame_data.append((interp_func_x(t), interp_func_y(t)))

    print("number of of frames:", len(frame_data))
    return frame_data


# --------------------------------------------------------------------------------------------------
def make_plot(puck_data, avi_filename=""):
    """"""
    # ----------------------------------------------------------------------------------------------
    def init_plot():
        plt.title(f'angle={degrees(angle)}')
        plt.xlim(-8, 8)
        plt.axis('equal')
        for peg in pegs:
            xx, yy = make_circle_points(peg)
            plt.plot(xx, yy, 'b')

        return line_trace, puck_dot,

    # ----------------------------------------------------------------------------------------------
    def update_plot(xy):
        xdata.append(xy[0])
        ydata.append(xy[1])
        line_trace.set_data(xdata, ydata)
        puck_dot.set_data(xy[0], xy[1])
        return line_trace, puck_dot,

    # ----------------------------------------------------------------------------------------------
    fig1 = plt.figure()
    line_trace, = plt.plot([], [], '-', color='0.5')
    puck_dot, = plt.plot([], [], 'ro', ms=3)

    if avi_filename.strip():
        fps = 15
        frame_data = align_to_framerate(puck_data, framerate=fps)
        Writer = animation.writers['ffmpeg']
        writer = Writer(fps=fps, metadata=dict(artist='bs-lab'), bitrate=1800)

        xdata = []
        ydata = []
        ani = animation.FuncAnimation(fig1, update_plot, frame_data, init_func=init_plot, blit=True)
        print(f'saving to "{avi_filename}"...')
        ani.save(avi_filename, writer=writer)
        print(f'saved to "{avi_filename}"')

    else:
        init_plot()

        for i in range(1, len(puck_data.t)):
            delta_t = puck_data.t[i] - puck_data.t[i-1]
            plt.pause(delta_t * REFRESH_FACT)
            plt.plot(puck_data.x[i], puck_data.y[i], 'r.')
            plt.draw()

        # prevent figure from disappearing
        plt.show()


# --------------------------------------------------------------------------------------------------
def get_position(vel, angle, t, accel):
    """"""
    x = vel * t * cos(angle)
    y = vel * t * sin(angle) - (0.5 * accel * t**2)
    return x, y


# --------------------------------------------------------------------------------------------------
def get_velocity(v_init, angle, accel):
    """"""
    vx = v_init * cos(angle)
    vy = v_init * sin(angle) - (accel * DT)
    new_vel = sqrt(vx**2 + vy**2)
    return vx, vy, new_vel


# --------------------------------------------------------------------------------------------------
def calc_rebound(old_angle, old_vel, x, y, peg):
    new_angle = calc_ricochet_angle(x, y, old_angle, peg)
    # reduce the puck velocity arbitrarily by REDUCE_VEL_FACT and the angle of impact
    angle_change = new_angle - old_angle
    new_vel = old_vel * (1 - (1 - REDUCE_VEL_FACT) * abs(sin(angle_change / 2)))
    print(f"  rebound: {degrees(new_angle):8.4f}\n")
    # print('old & new angles', degrees(old_angle), degrees(new_angle))
    # print("reduction of ", (1 - REDUCE_VEL_FACT) * abs(sin(angle_change / 2)))
    return new_angle, new_vel


# --------------------------------------------------------------------------------------------------
def get_line_circle_intersection(pt1, pt2, circle_center, circle_radius):
    """"""
    # ----------------------------------------------------------------------------------------------
    def sign(x):
        """"""
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
    x_int_a = (D * dy + sign(dy)*dx * AA) / (dr2) + circle_center[0]
    y_int_a = (-D * dx + abs(dy) * AA) / (dr2) + circle_center[1]

    # two of two intersection points
    x_int_b = (D * dy - sign(dy) * dx * AA) / (dr2) + circle_center[0]
    y_int_b = (-D * dx - abs(dy) * AA) / (dr2) + circle_center[1]

    # print('intersects at ', x_int_a, y_int_a)
    # print('           or ', x_int_b, y_int_b)

    # whichever point is closer to pt1 is used
    dist_a2 = (pt1[0] - x_int_a)**2 + (pt1[1] - y_int_a)**2
    dist_b2 = (pt1[0] - x_int_b)**2 + (pt1[1] - y_int_b)**2

    if dist_a2 < dist_b2:
        return x_int_a, y_int_a
    else:
        return x_int_b, y_int_b


# --------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    """"""
    pegs = create_pegs()
    puck_data = PuckData()

    vel = vo
    angle = launch_angle
    ddt = DT
    t = 0
    x = 0
    y = 0

    puck_data.append(t, x, y, vel, angle)

    # sys.stdout.write("     time         x         y     angle       vel     accel\n")

    while True:
        if puck_data.y[-1] <= LOWEST_Y:
            sys.stdout.write('reached bottom\n')
            break
        if t > MAX_TIME:
            sys.stdout.write(f'breaking at time = {t:12.8f}\n')
            break

        t += ddt

        # do not exceed terminal velocity
        if puck_data.v[-1] >= TERM_VEL:
            accel = 0
        else:
            accel = GC

        # update the relative position
        x_rel, y_rel = get_position(puck_data.v[-1], puck_data.angle[-1], ddt, accel)

        # update the velocity
        vx, vy, new_vel = get_velocity(puck_data.v[-1], puck_data.angle[-1], accel)

        if new_vel >= TERM_VEL:
            accel = 0
            new_vel = TERM_VEL

            # recalculate relative position based on terminal velocity and no acceleration
            x_rel, y_rel = get_position(TERM_VEL, puck_data.angle[-1], ddt, accel)

        # update velocity's angle
        x_inc, y_inc = get_position(puck_data.v[-1], puck_data.angle[-1], ddt - ddt/100, accel)
        new_angle = atan2(y_rel - y_inc, x_rel - x_inc)

        # update the absolute position
        x = puck_data.x[-1] + x_rel
        y = puck_data.y[-1] + y_rel

        # check if position is inside or contacting surface
        for i, peg in enumerate(pegs):
            dist_to_center = sqrt((x - peg.center[0])**2 + (y - peg.center[1])**2)
            if dist_to_center <= (peg.radius + RTOL):
                if dist_to_center < (peg.radius + RTOL):
                    print(f"inside!!!!!!!!! {i}")
                    # re-calculate current position so that it is on the surface, not inside
                    print('old coords', x, y)
                    x, y = get_line_circle_intersection([puck_data.x[-1], puck_data.y[-1]],
                                                        [x, y], peg.center, peg.radius)
                    print('new coords', x, y)

                # calculate rebound angle & reduced velocity
                new_angle, new_vel = calc_rebound(new_angle, new_vel, x, y, peg)

        puck_data.append(t, x, y, new_vel, new_angle)

        # sys.stdout.write(f"{t:9.4f} {x:9.4f} {y:9.4f} {degrees(new_angle):9.4f} ")
        # sys.stdout.write(f"{new_vel:9.4f} {accel:9.4f}\n")

        # reset time delta value for next iteration
        ddt = DT

    # print out results to the screen
    # sys.stdout.write("      time       x-pos       y-pos         vel\n")
    # for t, x, y, v in zip(puck_data.t, puck_data.x, puck_data.y, puck_data.v):
    #     sys.stdout.write(f"{t:10.4f} {x:11.8f} {y:11.8f} {v:11.8f}\n")

    # create matplotlib figure or mp4 file
    make_plot(puck_data, avi_filename="outfile.mp4")
