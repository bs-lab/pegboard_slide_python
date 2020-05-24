import sys
from math import pi, cos, sin, atan2, degrees, sqrt
import numpy as np

np.seterr(all='raise')

# constants
GC = 9.81         # m/s^2
# MIN_VEL = 0.10    # m/s
MIN_VEL = 0.00    # m/s
REDUCE_VEL_FACT = 0.6
DEBUG = False
DEBUG_2 = False


# --------------------------------------------------------------------------------------------------
def get_position(vel: float, angle: float, dt: float, accel: float) -> tuple:
    """
    Get puck position using ballistic trajectory equations.

    INPUTS:
    -------
    vel   -- initial puck velocity magnitude
    angle -- initial puck velocity angle
    dt     -- time elapsed
    accel -- initial puck acceleration

    RETURNS:
    --------
    x     -- updated puck x-coordinate after elapsed time
    y     -- updated puck y-coordinate after elapsed time
    """
    x = vel * dt * cos(angle)
    y = vel * dt * sin(angle) - (0.5 * accel * dt**2)
    return x, y


# --------------------------------------------------------------------------------------------------
def get_velocity(v_init: float, angle: float, dt: float, accel: float) -> float:
    """
    Get puck velocity using ballistic trajectory equations.

    INPUTS:
    -------
    v_init -- initial puck velocity magnitude
    angle  -- initial puck velocity angle
    dt     -- time elapsed
    accel  -- initial puck acceleration

    RETURNS:
    --------
    updated puck velocity magnitude after elapsed time
    """
    vx = v_init * cos(angle)
    vy = v_init * sin(angle) - (accel * dt)
    return sqrt(vx**2 + vy**2)


# --------------------------------------------------------------------------------------------------
def calc_rebound(old_angle: float, old_vel: float, x: float, y: float, circle_center: list,
                 circle_radius: float) -> tuple:
    """
    Calculates the puck velocity after rebounding off of a circular object (like a peg or
      other puck).

    INPUTS:
    -------
    old_angle     -- puck's velocity angle just prior to impact with the peg
    old_vel       -- puck's velocity magnitude prior to impact with the peg
    x             -- x-coordinate of the puck (whose surface should be on the surface of the peg)
    y             -- y-coordinate of the puck (whose surface should be on the surface of the peg)
    circle_center -- a list; center location of circular object
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
def calc_rebound_flat(old_angle: float, old_vel: float, x: float, y: float, flat) -> tuple:
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
def slide_down_flat(prev_vel: float, prev_x: float, prev_y: float, puck_radius: float, dt: float,
                    flat) -> tuple:
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
    curr_vel = prev_vel + a_parallel * dt

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

    curr_x = prev_x + prev_vel * dt * cos(phi)
    curr_y = prev_y + prev_vel * dt * sin(phi)

    if DEBUG_2:
        print(f'  current x&y {curr_x} {curr_y}')

    return phi, curr_vel, curr_x, curr_y, sdist


# --------------------------------------------------------------------------------------------------
def puck_collision(puck_1_angle: float, puck_1_vel: float, puck_1_x: float, puck_1_y: float,
                   puck_2) -> tuple:
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
