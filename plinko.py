import math
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy import interpolate

# constants
RTOL = 0.000001  # m
GC = 9.81        # m/s^2
TERM_VEL = 15.   # m/s
MAX_TIME = 8.   # s
DT = 0.005       # s
REDUCE_VEL_FACT = 0.6
REFRESH_FACT = 1.0
LOWEST_Y = -9

# user inputs
angle = sys.argv[1]
vo = 10.0  # m/s
launch_angle = float(angle) * math.pi / 180  # radians

print("INPUTS:")
print('  launch angle', math.degrees(launch_angle))
print('  initial vel ', vo)
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

    def append(self, t, x, y, v):
        self.t.append(t)
        self.x.append(x)
        self.y.append(y)
        self.v.append(v)


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
    phi = math.atan2(y - peg.center[1], x - peg.center[0])
    alpha = phi - vel_angle - math.pi
    return phi + alpha


# --------------------------------------------------------------------------------------------------
def calc_time_step_to_impact(t_curr, ddt, peg):
    """find ddt that would have resulted in ball just hitting the peg"""
    d_prev = math.sqrt((x_prev - peg.center[0])**2 + (y_prev - peg.center[1])**2)
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
        if theta > 2 * math.pi:
            break

        theta += 2 * math.pi * 0.01
        x.append(peg.radius * math.cos(theta) + peg.center[0])
        y.append(peg.radius * math.sin(theta) + peg.center[1])

    return(x, y)


# --------------------------------------------------------------------------------------------------
def align_to_framerate(puck_data, framerate=30, dilation=3):
    """"""
    interp_func_x = interpolate.interp1d(puck_data.t, puck_data.x)
    interp_func_y = interpolate.interp1d(puck_data.t, puck_data.y)
    max_time = max(puck_data.t)
    num_frames = math.ceil(max_time * framerate * dilation)
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
    x = vel * t * math.cos(angle)
    y = vel * t * math.sin(angle) - (0.5 * accel * t**2)
    return x, y


# --------------------------------------------------------------------------------------------------
def get_velocity(v_init, angle, accel):
    """"""
    vx = v_init * math.cos(angle)
    vy = v_init * math.sin(angle) - (accel * DT)
    new_vel = math.sqrt(vx**2 + vy**2)
    return vx, vy, new_vel


# --------------------------------------------------------------------------------------------------
def calc_rebound(old_angle, old_vel, x, y, peg):
    new_angle = calc_ricochet_angle(x, y, old_angle, peg)
    # reduce the puck velocity arbitrarily by REDUCE_VEL_FACT and the angle of impact
    angle_change = new_angle - old_angle
    new_vel = old_vel * (1 - (1 - REDUCE_VEL_FACT) * abs(math.sin(angle_change / 2)))
    print(f"  rebound: {math.degrees(new_angle):8.4f}\n")
    # print('old & new angles', math.degrees(old_angle), math.degrees(new_angle))
    # print("reduction of ", (1 - REDUCE_VEL_FACT) * abs(math.sin(angle_change / 2)))
    return new_angle, new_vel


# --------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    """"""
    pegs = create_pegs()
    puck_data = PuckData()

    vel = vo
    angle = launch_angle
    ddt = DT
    t = 0
    x_prev = 0
    y_prev = 0
    x = 0
    y = 0

    puck_data.append(t, x, y, vel)

    sys.stdout.write("     time         x         y     angle       vel     accel\n")
    while True:
        if puck_data.y[-1] <= LOWEST_Y:
            sys.stdout.write('reached bottom\n')
            break
        if t > MAX_TIME:
            sys.stdout.write(f'breaking at time = {t:12.8f}\n')
            break

        t += ddt

        # do not exceed terminal velocity
        if vel >= TERM_VEL:
            accel = 0
        else:
            accel = GC

        # update the relative position
        x_rel, y_rel = get_position(vel, angle, ddt, accel)

        # update the velocity
        vx, vy, new_vel = get_velocity(vel, angle, accel)

        if new_vel >= TERM_VEL:
            accel = 0
            new_vel = TERM_VEL

            # recalculate relative position based on terminal velocity and no acceleration
            x_rel, y_rel = get_position(TERM_VEL, angle, ddt, accel)

        # update velocity's angle
        x_inc, y_inc = get_position(vel, angle, ddt - ddt/100, accel)
        new_angle = math.atan2(y_rel - y_inc, x_rel - x_inc)

        # update the absolute position
        x += x_rel
        y += y_rel

        # check if inside circle
        inside_peg = False
        for i, peg in enumerate(pegs):
            dist_to_center = math.sqrt((x - peg.center[0])**2 + (y - peg.center[1])**2)
            if dist_to_center < peg.radius - RTOL:
                print(f"contact! {i} {dist_to_center:10.6f} {x:10.6f} {y:10.6f}")
                # find ddt that would have resulted in ball *just* hitting the peg
                t, ddt = calc_time_step_to_impact(t, ddt, peg)
                x -= x_rel
                y -= y_rel
                inside_peg = True
                continue

        if inside_peg:
            continue

        # check if contacting surface
        for i, peg in enumerate(pegs):
            dist_to_center = math.sqrt((x - peg.center[0])**2 + (y - peg.center[1])**2)
            if (peg.radius - RTOL) <= dist_to_center <= (peg.radius + RTOL):
                print(f"exact!!!!!!!!! {i}")
                # calculate rebound angle & reduced velocity
                new_angle, new_vel = calc_rebound(new_angle, new_vel, x, y, peg)

        puck_data.append(t, x, y, new_vel)

        sys.stdout.write(f"{t:9.4f} {x:9.4f} {y:9.4f} {math.degrees(new_angle):9.4f} ")
        sys.stdout.write(f"{new_vel:9.4f} {accel:9.4f}\n")

        # update values for next iteration
        angle = new_angle
        vel = new_vel
        ddt = DT
        x_prev = x
        y_prev = y

    # print out results to the screen
    sys.stdout.write("      time       x-pos       y-pos         vel\n")
    for t, x, y, v in zip(puck_data.t, puck_data.x, puck_data.y, puck_data.v):
        sys.stdout.write(f"{t:10.4f} {x:11.8f} {y:11.8f} {v:11.8f}\n")

    # create matplotlib figure or mp4 file
    make_plot(puck_data, avi_filename="outfile.mp4")
