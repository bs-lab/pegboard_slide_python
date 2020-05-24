import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from math import cos, sin, ceil, pi
from scipy import interpolate

# constants
DEBUG = False


# --------------------------------------------------------------------------------------------------
def make_circle_points(center: list, radius: float) -> tuple:
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

    return x, y


# --------------------------------------------------------------------------------------------------
def align_to_framerate(puck, max_time: float, framerate: int, dilation: float = 1.0) -> list:
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
def make_plot(pucks, pegs, flat_surfaces, plot_title: str = "", avi_filename="", fps=30,
              dilation=1):
    """Creates either a matplotlib graph or an mp4 file showing the results of the analysis"""
    # ----------------------------------------------------------------------------------------------
    def init_plot():
        plt.title(plot_title)
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
    puck_dot, = plt.plot([], [], 'ro', ms=45*pucks[0].radius)

    frame_data = []
    max_time = max([x.final_time for x in pucks])

    for puck in pucks:
        frame_data.append(align_to_framerate(puck, max_time, fps, dilation=dilation))

    if avi_filename.strip():
        ani = animation.FuncAnimation(fig1, update_plot, zip(*frame_data),
                                      init_func=init_plot, blit=True, save_count=len(frame_data[0]))

        sys.stderr.write(f'saving to "{avi_filename}"...\n')
        Writer = animation.writers['ffmpeg']
        writer = Writer(fps=fps, metadata=dict(artist='bs-lab'), bitrate=1800)
        ani.save(avi_filename, writer=writer)
        sys.stderr.write(f'saved to "{avi_filename}"\n')

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
