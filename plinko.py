import sys
from math import atan2, asin, ceil, floor, degrees, radians, pi, sqrt
import geometry as geom
import eom
import plotting
import inputs

# constants
TERM_VEL = 15.    # m/s
TRANSITION_VEL = 0.8  # m/s
DT = 0.0001       # s
LOWEST_Y = -10
DEBUG = False
DEBUG_2 = False
PEG_RADIUS = 0.25   # m
PUCK_RADIUS = 0.10  # m
FPS = 30
DILATION = 2

print('INPUTS:')
print('  launch angle', inputs.ANGLE)
print('  initial vel ', inputs.VO)
print('  terminal vel', TERM_VEL)
print('  max sim time', inputs.MAX_TIME)
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
def create_puck_spray(num_pucks, first_angle):
    """"""
    pucks = []

    # 135 degree sweeps
    final_angle = first_angle - 0.75 * pi   # 135 degree sweeps

    # 360 degree sweeps (has problem in that first & last pucks of a sweep use same angle)
    # final_angle = first_angle - 2 * pi

    x_coord = 0
    y_coord = 1
    t = 0
    start_time = 0
    safe_time = 0.1
    num_safe_frames = safe_time / DT

    spacing_angle = asin(5 * PUCK_RADIUS / (inputs.VO * DT * num_safe_frames))
    max_pucks_per_sweep = floor(abs(final_angle - first_angle) / spacing_angle) + 1
    # print(f'first_angle {first_angle} {degrees(first_angle)}')
    # print(f'final_angle {final_angle} {degrees(final_angle)}')
    # print(f'max_pucks_per {max_pucks_per_sweep}')
    # print(f'was spacing angle {spacing_angle}')
    if num_pucks < max_pucks_per_sweep:
        max_pucks_per_sweep = num_pucks

    spacing_angle = (final_angle - first_angle) / (max_pucks_per_sweep - 1)
    # print(f'now spacing angle {spacing_angle}')
    # print(f'final_angle {degrees(final_angle)}')

    # puck_angle = first_angle - spacing_angle
    time_between = DT * floor(num_safe_frames / max_pucks_per_sweep)
    time_per_sweep = DT * (num_safe_frames + 1)
    start_time = -1 * time_per_sweep

    s = -1  # sweep counter
    for p in range(num_pucks):
        if p % max_pucks_per_sweep == 0:
            print("  ------------------------- new sweep")
            puck_angle = first_angle - spacing_angle
            s += 1
            start_time = s * time_per_sweep - time_between

        start_time += time_between
        pucks.append(geom.PuckData(PUCK_RADIUS, start_time, inputs.MAX_TIME, p))
        puck_angle += spacing_angle
        print(f'  puck #{p}, angle {degrees(puck_angle):14.8f} @ start_time = {start_time:8.4f}')
        pucks[-1].append(t, x_coord, y_coord, inputs.VO, puck_angle, -1)

    print('')

    return pucks


# --------------------------------------------------------------------------------------------------
def create_puck_shower(num_pucks, left_x, right_x, launch_angle):
    """Creates list of PuckData instances, with evenly spaced launch locations"""
    pucks = []
    if num_pucks > 1:
        x_spacing = (right_x - left_x) / (num_pucks - 1)
    else:
        x_spacing = 0

    x_coord = left_x - x_spacing
    y_coord = 0
    t = 0
    start_time = 0

    for p in range(num_pucks):
        pucks.append(geom.PuckData(PUCK_RADIUS, start_time, inputs.MAX_TIME, p))
        x_coord += x_spacing
        pucks[-1].append(t, x_coord, y_coord, inputs.VO, launch_angle, -1)

    return pucks


# --------------------------------------------------------------------------------------------------
def create_block_of_pucks(num_pucks):
    """"""
    block_rh_wall_x = 3
    block_lh_wall_x = -1 * block_rh_wall_x

    min_block_wall_y = 0

    # create block of pucks (leave 10% of diameter as spacing between pucks)
    x_spacing = 0.2 * PUCK_RADIUS
    y_spacing = 0.2 * PUCK_RADIUS

    block_width = block_rh_wall_x - block_lh_wall_x
    pucks_per_row = floor(block_width / (2 * PUCK_RADIUS + x_spacing))
    num_rows = ceil(num_pucks / pucks_per_row)

    print(f'pucks/row = {pucks_per_row}')
    print(f'num rows  = {num_rows}')
    print('')

    x_init = block_lh_wall_x + PUCK_RADIUS + x_spacing / 2
    y_coord = min_block_wall_y + PUCK_RADIUS + y_spacing
    vel_init = 0
    puck_angle = 0
    start_time = 0

    pucks = []
    for p in range(num_pucks):
        if p % pucks_per_row == 0:
            # reset the x- & y-coord for the next row
            x_coord = x_init
            y_coord += 2 * PUCK_RADIUS + y_spacing

        pucks.append(geom.PuckData(PUCK_RADIUS, start_time, inputs.MAX_TIME, p))
        pucks[-1].append(t, x_coord, y_coord, vel_init, puck_angle, -1)

        x_coord += PUCK_RADIUS*2.2

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
def update_puck(puck, pucks, flat_surfaces, t):
    """Updates the PuckData instance based on its projectile motion (and possible ricochets after
       a time interval of DT."""
    # do not exceed terminal velocity
    if puck.v[-1] >= TERM_VEL:
        accel = 0
    else:
        accel = eom.GC

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
        if dist_to_flat <= puck.radius + geom.RTOL:
            if new_vel < TRANSITION_VEL:
                puck.sliding_on = flat.id

            if dist_to_flat < puck.radius + geom.RTOL:
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
        if dist_to_center <= (peg.radius + puck.radius + geom.RTOL):
            puck.sliding_on = -1
            if dist_to_center < (peg.radius + puck.radius + geom.RTOL):
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
        if other_puck.id == puck.id:
            continue

        if not other_puck.inplay:
            # other puck is no longer bouncing about, so treat it as non-existent
            continue

        # NOTE: may be faster to pre-calculate all distances at once
        dist_to_center = sqrt((x - other_puck.x[-1])**2 + (y - other_puck.y[-1])**2)

        if dist_to_center <= (other_puck.radius + puck.radius + geom.RTOL):
            if dist_to_center < (other_puck.radius + puck.radius + geom.RTOL):
                if DEBUG_2:
                    print(f"{puck.id} inside other puck!!!!!!!!! {other_puck.id}")
                    print('  old puck coords', x, y)
                # re-calculate current position so that it is on the surface, not inside
                x, y = geom.get_line_circle_intersection([puck.x[-1], puck.y[-1]], [x, y],
                                                         [other_puck.x[-1], other_puck.y[-1]],
                                                         other_puck.radius + puck.radius)
                if DEBUG_2:
                    print('  new puck coords', x, y)

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
    launch_angle = radians(float(inputs.ANGLE))
    vel = inputs.VO
    t = 0
    x = 0
    y = 0

    pegs = create_pegs()
    flat_surfaces = create_flat_surfaces()

    if inputs.SHOT_TYPE == 'spray':
        pucks = create_puck_spray(inputs.NUM_PUCKS, launch_angle)
    elif inputs.SHOT_TYPE == 'shower':
        pucks = create_puck_shower(inputs.NUM_PUCKS, -6, 6, launch_angle)
    elif inputs.SHOT_TYPE == 'block':
        pucks = create_block_of_pucks(inputs.NUM_PUCKS)
    else:
        sys.stderr.write(f'*** unknown SHOT_TYPE: {inputs.SHOT_TYPE} ***\n')
        raise TypeError

    if DEBUG:
        sys.stdout.write("          time         x         y     angle       vel     accel")
        sys.stdout.write("  sliding_on\n")

    while True:
        if abs((t * 100) % 1) - 0 < 0.000001:
            sys.stderr.write(f"time ={t:6.2f}\n")

        for p, puck in enumerate(pucks):
            if abs(t - puck.start_time) < 0.000001:
                puck.inplay = True
                sys.stdout.write(f'launching puck #{puck.id} at time = {t:12.8f}\n\n')

            # if puck has not yet launched or is done, just repeat its previous state
            if not puck.inplay:
                puck.append(t, puck.x[-1], puck.y[-1], puck.v[-1], puck.angle[-1], -1)
                continue

            if puck.y[-1] <= LOWEST_Y:
                sys.stdout.write(f'puck #{p} reached bottom at time={t:8.4f}\n\n')
                puck.inplay = False
                puck.final_time = t
                # break

            puck = update_puck(puck, pucks, flat_surfaces, t)

        # end simulation if all pegs have reached bottom
        keep_going = False
        for puck in pucks:
            if puck.inplay:
                keep_going = True
        if not keep_going:
            sys.stdout.write(f'\nbreaking early at time = {t:12.8f}\n\n')
            break

        # end simulation if it is taking too long
        if t >= inputs.MAX_TIME:
            sys.stdout.write(f'\nbreaking at time = {t:12.8f}\n\n')
            break

        t = round(t + DT, 4)   # using 't += DT' causes floating error build-ups

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
    plotting.make_plot(pucks, pegs, flat_surfaces, plot_title=plot_title,
                       avi_filename=inputs.OUTFILE_MP4, fps=FPS, dilation=DILATION)
    # plotting.make_plot(pucks, pegs, flat_surfaces, plot_title=plot_title)
