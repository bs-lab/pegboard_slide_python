import math
import matplotlib.pyplot as plt
import sys

# constants
RTOL = 0.000001  # m
GC = 9.81        # m/s^2
TERM_VEL = 15.   # m/s
MAX_TIME = 1.0   # s
DT = 0.005       # s

# user inputs
vo = 10.0  # m/s
launch_angle = -27 * math.pi / 180  # radians

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


pegs = []
# pegs.append(Peg(4.300, -3.250, 0.25))
pegs.append(Peg(4.569, -3.250, 0.25))
pegs.append(Peg(5.650, -6.000, 0.25))

# will not intersect the 2nd peg if DT = 0.01
# pegs.append(Peg(4.569, -3.250, 0.25))
# pegs.append(Peg(5.850, -6.000, 0.25))


# --------------------------------------------------------------------------------------------------
def calc_ricochet_angle(x, y, vel_angle, peg):
    """"""
    phi = math.atan2(y - peg.center[1], x - peg.center[0])
    alpha = phi - vel_angle - math.pi
    return phi + alpha


# --------------------------------------------------------------------------------------------------
def calc_time_step_to_impact(time, ddt, peg):
    """find ddt that would have resulted in ball just hitting the peg"""
    t_prev = time - ddt
    t_curr = time
    d_prev = math.sqrt((x_prev - peg.center[0])**2 + (y_prev - peg.center[1])**2)
    d_curr = dist_to_center
    t_need = t_prev - (t_prev - t_curr) * (d_prev - peg.radius) / (d_prev - d_curr)
    time -= ddt
    ddt = t_need - t_prev
    # print("    stuff", t_prev, t_curr, d_prev, d_curr, t_need)
    # print('    for next time, ', time, ddt, time + ddt)
    return time, ddt


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
def make_plot(puck_data):
    """"""
    for peg in pegs:
        xx, yy = make_circle_points(peg)
        plt.plot(peg.center[0], peg.center[1], 'o')
        plt.plot(xx, yy, 'k')

    # plt.plot(all_x, all_y)
    # plt.plot(puck_data.x, puck_data.y, 'x-')
    plt.plot(puck_data.x, puck_data.y, '-')

    plt.axis('equal')
    plt.show()


# --------------------------------------------------------------------------------------------------
def get_position(vel, angle, time, accel):
    """"""
    x = vel * time * math.cos(angle)
    y = vel * time * math.sin(angle) - (0.5 * accel * time**2)
    return x, y


# --------------------------------------------------------------------------------------------------
def get_velocity(v_init, angle, accel):
    """"""
    vx = v_init * math.cos(angle)
    vy = v_init * math.sin(angle) - (accel * DT)
    new_vel = math.sqrt(vx**2 + vy**2)
    return vx, vy, new_vel


# --------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    """"""
    all_x = []
    all_y = []
    all_t = []
    # print("                 time         x         y     angle       vel")
    max_i = math.floor(MAX_TIME / DT) + 1
    for i in range(max_i):
        time = DT * i
        x, y = get_position(vo, launch_angle, time, GC)
        vx = vo * math.cos(launch_angle)
        vy = vo * math.sin(launch_angle) - GC * time
        vel = math.sqrt(vx**2 + vy**2)
        angle = math.atan2(y, x)
        # print(f"continuous {time:9.4f} {x:9.4f} {y:9.4f} {math.degrees(angle):16.8f} {vel:16.8f}")
        all_t.append(time)
        all_x.append(x)
        all_y.append(y)

    x = 0
    y = 0
    puck_data = PuckData()

    vel = vo
    angle = launch_angle
    ddt = DT
    time = 0
    x_prev = 0
    y_prev = 0

    puck_data.append(time, x, y, vel)

    sys.stdout.write("     time         x         y     angle       vel     accel\n")
    while True:
        if time > MAX_TIME:
            sys.stdout.write(f'breaking at time = {time:12.8f}\n')
            break

        time += ddt

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
                time, ddt = calc_time_step_to_impact(time, ddt, peg)
                x -= x_rel
                y -= y_rel
                inside_peg = True
                continue

        if inside_peg:
            continue

        for i, peg in enumerate(pegs):
            dist_to_center = math.sqrt((x - peg.center[0])**2 + (y - peg.center[1])**2)
            if (peg.radius - RTOL) <= dist_to_center <= (peg.radius + RTOL):
                print(f"exact!!!!!!!!! {i}")
                # calculate rebound angle
                new_angle = calc_ricochet_angle(x, y, new_angle, peg)
                print(f"rebound: {math.degrees(new_angle):8.4f}\n")

        puck_data.append(time, x, y, new_vel)

        sys.stdout.write(f"{time:9.4f} {x:9.4f} {y:9.4f} {math.degrees(new_angle):9.4f} ")
        sys.stdout.write(f"{new_vel:9.4f} {accel:9.4f}\n")

        angle = new_angle
        vel = new_vel
        ddt = DT

        x_prev = x
        y_prev = y

    # print out results to the screen
    sys.stdout.write("      time       x-pos       y-pos         vel\n")
    for t, x, y, v in zip(puck_data.t, puck_data.x, puck_data.y, puck_data.v):
        sys.stdout.write(f"{t:10.4f} {x:11.8f} {y:11.8f} {v:11.8f}\n")

    # create matplotlib figure
    make_plot(puck_data)
