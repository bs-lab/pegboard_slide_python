import math
import matplotlib.pyplot as plt
import sys

# constants
RTOL = 0.000001  # m
GC = 9.81        # m/s^2
TERM_VEL = 15.   # m/s
MAX_TIME = 0.8   # s
DT = 0.01        # s

# user inputs
vo = 10.0  # m/s
launch_angle = -27 * math.pi / 180  # radians

# peg_center = [4.523, -3.25]
peg_center = [4.569, -3.25]
peg_radius = 0.25

print("INPUTS:")
print('  launch angle', math.degrees(launch_angle))
print('  initial vel ', vo)
print('  max sim time', MAX_TIME)
print('')


# --------------------------------------------------------------------------------------------------
def make_plot():
    # box_x = [4.0, 4.5, 4.5, 4.0, 4.0]
    # box_y = [-3.5, -3.5, -3.0, -3.0, -3.5]
    # plt.plot(box_x, box_y)
    circle_x, circle_x2, circle_y = make_circle_points(peg_center, peg_radius)
    plt.plot(all_x, all_y)

    plt.plot(disc_x, disc_y, 'x')
    plt.plot(circle_x, circle_y)
    plt.plot(circle_x2, circle_y)
    plt.axis('equal')
    plt.show()


# --------------------------------------------------------------------------------------------------
def calc_ricochet_angle(x, y):
    """"""
    phi = math.atan2(y - peg_center[1], x - peg_center[0])
    alpha = phi + new_angle - math.pi
    return new_angle + 2 * alpha


# --------------------------------------------------------------------------------------------------
def calc_time_step_to_impact(time, ddt):
    """find ddt that would have resulted in ball just hitting the peg"""
    t_prev = time - ddt
    t_curr = time
    d_prev = math.sqrt((x_prev - peg_center[0])**2 + (y_prev - peg_center[1])**2)
    d_curr = dist_to_center
    t_need = t_prev - (t_prev - t_curr) * (d_prev - peg_radius) / (d_prev - d_curr)
    time -= ddt
    ddt = t_need - t_prev
    # print("    stuff", t_prev, t_curr, d_prev, d_curr, t_need)
    # print('    for next time, ', time, ddt, time + ddt)
    return time, ddt


# --------------------------------------------------------------------------------------------------
def get_circle_x(yyy):
    x1 = math.sqrt(peg_radius**2 - (yyy - peg_center[1])**2) + peg_center[0]
    x2 = -1*math.sqrt(peg_radius**2 - (yyy - peg_center[1])**2) + peg_center[0]
    return x1, x2


# --------------------------------------------------------------------------------------------------
def make_circle_points(circle_center, circle_radius):
    """only used for plotting, not analysis"""
    yyy_min = circle_center[1] - circle_radius
    yyy_max = circle_center[1] + circle_radius

    dy = (yyy_max - yyy_min) / 200
    yyy = yyy_min - dy

    circle_x = []
    circle_x2 = []
    circle_y = []

    while True:
        yyy += dy
        if yyy > yyy_max:
            break
        xxx, xxx2 = get_circle_x(yyy)
        circle_x.append(xxx)
        circle_x2.append(xxx2)
        circle_y.append(yyy)

    return(circle_x, circle_x2, circle_y)


# --------------------------------------------------------------------------------------------------
def get_position(vel, angle, time, accel):
    x = vel * time * math.cos(angle)
    y = vel * time * math.sin(angle) - (0.5 * accel * time**2)
    return x, y


# --------------------------------------------------------------------------------------------------
def get_velocity(v_init, angle, accel):
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
    disc_t = []
    disc_x = []
    disc_y = []
    disc_v = []

    vel = vo
    angle = launch_angle
    ddt = DT
    time = 0
    x_prev = 0
    y_prev = 0

    disc_t.append(time)
    disc_x.append(x)
    disc_y.append(y)
    disc_v.append(vel)

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
        dist_to_center = math.sqrt((x - peg_center[0])**2 + (y - peg_center[1])**2)
        if dist_to_center < peg_radius - RTOL:
            # print(f"contact! {dist_to_center:10.6f} {x:10.6f} {y:10.6f}")
            # find ddt that would have resulted in ball *just* hitting the peg
            time, ddt = calc_time_step_to_impact(time, ddt)
            x -= x_rel
            y -= y_rel
            continue

        if (peg_radius - RTOL) <= dist_to_center <= (peg_radius + RTOL):
            # print("exact!!!!!!!!!")
            # calculate rebound angle
            new_angle = calc_ricochet_angle(x, y)

        disc_t.append(time)
        disc_x.append(x)
        disc_y.append(y)
        disc_v.append(new_vel)

        sys.stdout.write(f"{time:9.4f} {x:9.4f} {y:9.4f} {math.degrees(new_angle):9.4f} ")
        sys.stdout.write(f"{new_vel:9.4f} {accel:9.4f}\n")

        angle = new_angle
        vel = new_vel
        ddt = DT

        x_prev = x
        y_prev = y

    sys.stdout.write("      time       x-pos       y-pos         vel\n")
    for t, x, y, v in zip(disc_t, disc_x, disc_y, disc_v):
        sys.stdout.write(f"{t:10.4f} {x:11.8f} {y:11.8f} {v:11.8f}\n")

    make_plot()
