import math
import matplotlib.pyplot as plt
import time as xtime

RTOL = 0.00001
g = 9.81   # m/s^2
vo = 10.  # m/s
term_vel = 15
dt = 0.02
max_time = 0.8   # s
max_i = math.floor(max_time / dt) + 1

launch_angle = -27 * math.pi / 180  # radians

box_x = [4.0, 4.5, 4.5, 4.0, 4.0]
box_y = [-3.5, -3.5, -3.0, -3.0, -3.5]

peg_center = [4.20, -3.25]
#peg_center = [3.90, -3.25]
peg_radius = 0.25


def get_circle_x(yyy):
    blah1 = math.sqrt(peg_radius**2 - (yyy - peg_center[1])**2) + peg_center[0]
    blah2 = -1*math.sqrt(peg_radius**2 - (yyy - peg_center[1])**2) + peg_center[0]
    return blah1, blah2


yyy_min = peg_center[1] - peg_radius
yyy_max = peg_center[1] + peg_radius

dy = (yyy_max - yyy_min) / 20
print("x: min, max ", peg_center[0] - peg_radius, peg_center[0] + peg_radius)
print("y: min, max ", yyy_min, yyy_max)
print('dy', dy)

yyy = yyy_min - dy
circle_x = []
circle_x2 = []
circle_y = []
while True:
    yyy += dy
    if yyy > yyy_max:
        break
    xxx, xxx2 = get_circle_x(yyy)
    print("xxx, yyy", xxx, yyy)
    circle_x.append(xxx)
    circle_x2.append(xxx2)
    circle_y.append(yyy)


# --------------------------------------------------------------------------------------------------
def get_position_new(vel, angle, time, accel):
    x = vel * time * math.cos(angle)
    y = vel * time * math.sin(angle) - (0.5 * accel * time**2)
    return x, y


# --------------------------------------------------------------------------------------------------
def get_position(t):
    x = vo * t * math.cos(launch_angle)
    y = vo * t * math.sin(launch_angle) - (0.5 * g * t**2)
    return x, y


# --------------------------------------------------------------------------------------------------
def get_velocity(v_init, angle, accel):
    vx = v_init * math.cos(angle)
    vy = v_init * math.sin(angle) - (accel * dt)
    new_vel = math.sqrt(vx**2 + vy**2)
    return vx, vy, new_vel


# --------------------------------------------------------------------------------------------------
all_x = []
all_y = []
all_t = []
print("                 time         x         y     angle       vel")
for i in range(max_i):
    time = dt * i
    x, y = get_position(time)
    vx = vo * math.cos(launch_angle)
    vy = vo * math.sin(launch_angle) - g * time
    vel = math.sqrt(vx**2 + vy**2)
    angle = math.atan2(y, x)
    # print(f"continuous  {time:9.4f} {x:9.4f} {y:9.4f} {math.degrees(angle):16.8f} {vel:16.8f}")
    all_t.append(time)
    all_x.append(x)
    all_y.append(y)

print('angle', launch_angle * 180 / math.pi)

x = 0
y = 0
disc_x = []
disc_y = []
disc_t = []
disc_x.append(x)
disc_y.append(y)
disc_t.append(0)

vel = vo
angle = launch_angle
print("                 time         x         y     angle       vel")
print('max_i', max_i)
ddt = dt
time = 0
x_prev = 0
y_prev = 0

while True:
    if vel >= term_vel:
        accel = 0
    else:
        accel = g

    time += ddt

    x_new, y_new = get_position_new(vel, angle, ddt, accel)
    x_inc, y_inc = get_position_new(vel, angle, ddt - ddt/100, accel)
    vx, vy, new_vel = get_velocity(vel, angle, accel)

    if vel > term_vel:
        # recalculate position based on terminal velocity and no acceleration
        x_new, y_new = get_position_new(term_vel, angle, ddt, 0)
        x_inc, y_inc = get_position_new(term_vel, angle, ddt - ddt/100, 0)
        vx, vy, new_vel = get_velocity(term_vel, angle, 0)

    x += x_new
    y += y_new

    new_angle = math.atan2(y_new - y_inc, x_new - x_inc)
    #print('new_angle', new_angle, x_new, x_inc, y_new, y_inc)
    # print(f"{time:9.4f} {x:9.4f} {y:9.4f}")

    # check if inside circle
    dist_to_center = math.sqrt((x - peg_center[0])**2 + (y - peg_center[1])**2)
    if dist_to_center < peg_radius - RTOL:
        print("contact!", dist_to_center, x, y)
        # find ddt that would have resulted in ball just hitting the peg
        t_prev = time - ddt
        t_curr = time
        d_prev = math.sqrt((x_prev - peg_center[0])**2 + (y_prev - peg_center[1])**2)
        d_curr = dist_to_center
        t_need = t_prev - (t_prev - t_curr) * (d_prev - peg_radius) / (d_prev - d_curr)
        print("    stuff", t_prev, t_curr, d_prev, d_curr, t_need)
        time -= ddt
        ddt = t_need - t_prev
        x -= x_new
        y -= y_new
        print('    for next time, ', time, ddt, time + ddt)
        # xtime.sleep(1)
        continue

    if (peg_radius - RTOL) <= dist_to_center <= (peg_radius + RTOL):
        print("exact!!!!!!!!!")
        # calculate rebound angle
        print('current trajectory', math.degrees(new_angle))
        phi = math.atan2(y - peg_center[1], x - peg_center[0])
        print(' wrt circle', math.degrees(phi), x, y)
        alpha = phi + new_angle - math.pi
        print(' alpha', math.degrees(alpha))
        final_angle = new_angle + 2 * alpha
        print(' final', math.degrees(final_angle))
        new_angle = final_angle

    disc_x.append(x)
    disc_y.append(y)
    disc_t.append(time)

    print(f"discrete    {time:9.4f} {x:9.4f} {y:9.4f} {math.degrees(new_angle):16.8f} {new_vel:16.8f}")
 
    angle = new_angle
    vel = new_vel
    ddt = dt

    x_prev = x
    y_prev = y

    if time > max_time:
        print('breaking at time=', time)
        break

for t, x, y in zip(disc_t, disc_x, disc_y):
    print(f"{t:12.8f} {x:12.8f} {y:12.8f}")

# plt.plot(all_x, all_y)
plt.plot(disc_x, disc_y, 'x')
# plt.plot(box_x, box_y)
plt.plot(circle_x, circle_y)
plt.plot(circle_x2, circle_y)
plt.axis('equal')
plt.show()
# --------------------------------------------------------------------------------------------------
