import sys
from math import atan2, degrees, pi, sqrt
import geometry as geom
import eom
import plotting

# constants
RTOL = 0.0000001  # m
GC = 9.81         # m/s^2
TERM_VEL = 15.    # m/s
TRANSITION_VEL = 0.8  # m/s
MAX_TIME = 10.0   # s
DT = 0.0001       # s
LOWEST_Y = -10
DEBUG = False
DEBUG_2 = False
PEG_RADIUS = 0.25
PUCK_RADIUS = 0.10
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
def create_pegs():
    """Creates list of Peg instances"""
    peg_rows = [-2, -4, -6, -8]
    peg_cols = [-6, -4, -2, 0, 2, 4, 6]

    pegs = []
    pid = -1
    for r in peg_rows:
        for c in peg_cols:
            pid += 1
            pegs.append(geom.Peg(c, r, PEG_RADIUS, pid))

    # include offset rows
    for r in peg_rows:
        for c in peg_cols:
            pid += 1
            pegs.append(geom.Peg(c+1, r+1, PEG_RADIUS, pid))

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
        pucks.append(geom.PuckData(PUCK_RADIUS, MAX_TIME, p))
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
        pucks.append(geom.PuckData(PUCK_RADIUS, MAX_TIME, p))
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
    flat_surfaces.append(geom.FlatSurface([lh_wall_x, max_wall_y], [lh_wall_x, min_wall_y], 0))
    flat_surfaces.append(geom.FlatSurface([rh_wall_x, max_wall_y], [rh_wall_x, min_wall_y], 1))

    # create bottom
    flat_surfaces.append(geom.FlatSurface([lh_wall_x, min_wall_y], [-1, min_wall_y - 0.5], 2))
    # flat_surfaces.append(geom.FlatSurface([rh_wall_x, min_wall_y], [2, min_wall_y - 0.5], 3))
    flat_surfaces.append(geom.FlatSurface([rh_wall_x, min_wall_y], [0, min_wall_y - 0.5], 3))

    flat = geom.FlatSurface([-3, -3], [0, -3.75], 4)
    # flat = geom.FlatSurface([-3, -3], [0, -4], 4)
    flat_surfaces.append(flat)

    for flat in flat_surfaces:
        print(f'  flat angle: {flat.id} {degrees(flat.angle)}')
    print('')

    return flat_surfaces


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
        new_angle, new_vel, x, y, sdist = eom.slide_down_flat(puck.v[-1], puck.x[-1], puck.y[-1],
                                                              puck.radius, DT,
                                                              flat_surfaces[puck.sliding_on])
        if sdist > 1:
            puck.sliding_on = -1

    else:
        # update the relative position
        x_rel, y_rel = eom.get_position(puck.v[-1], puck.angle[-1], DT, accel)

        # update the velocity
        new_vel = eom.get_velocity(puck.v[-1], puck.angle[-1], DT, accel)

        if new_vel >= TERM_VEL:
            accel = 0
            new_vel = TERM_VEL

            # recalculate relative position based on terminal velocity and no acceleration
            x_rel, y_rel = eom.get_position(TERM_VEL, puck.angle[-1], DT, accel)

        # update velocity's angle
        x_inc, y_inc = eom.get_position(puck.v[-1], puck.angle[-1], DT - DT / 100, accel)
        new_angle = atan2(y_rel - y_inc, x_rel - x_inc)

        # update the absolute position
        x = puck.x[-1] + x_rel
        y = puck.y[-1] + y_rel

    # check if position is inside or contacting flat surface
    for flat in flat_surfaces:
        if not geom.puck_near_flat([x, y], puck.radius, flat):
            continue

        dist_to_flat = geom.calc_dist_to_object([x, y], flat)
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
                x, y = geom.calc_puck_flat_intersection([puck.x[-1], puck.y[-1]], [x, y],
                                                        puck.radius, flat)
                if DEBUG_2:
                    print('  new puck coords', x, y)

            # calculate rebound angle & reduced velocity
            if puck.sliding_on < 0:
                new_angle, new_vel = eom.calc_rebound_flat(new_angle, new_vel, x, y, flat)

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
                x, y = geom.get_line_circle_intersection([puck.x[-1], puck.y[-1]], [x, y],
                                                         peg.center, peg.radius + puck.radius)
                if DEBUG_2:
                    print('  new puck coords', x, y)

            # calculate rebound angle & reduced velocity
            new_angle, new_vel = eom.calc_rebound(new_angle, new_vel, x, y, peg.center, peg.radius)

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
                x, y = geom.get_line_circle_intersection([puck.x[-1], puck.y[-1]], [x, y],
                                                         [other_puck.x[-1], other_puck.y[-1]],
                                                         other_puck.radius + puck.radius)
                if DEBUG_2:
                    print('  new puck coords', x, y)

            # should puck_collision use 'new_angle' and 'new_vel', 'x', and 'y' ???
            new_angle, new_vel, other_angle, other_vel = eom.puck_collision(new_angle, new_vel,
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
    plot_title = f'angle={degrees(launch_angle):12.8f}'
    plotting.make_plot(pucks, pegs, flat_surfaces, plot_title=plot_title, avi_filename=OUTFILE_MP4)
    # make_plot(pucks)
