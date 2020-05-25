import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from math import cos, sin, pi

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
def make_plot(frame_list, pucks, pegs, flat_surfaces, plot_title: str = "", avi_filename: str = "",
              fps: int = 30):
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
            #     print(f'x {x}, xy {xy}')

        puck_dot.set_data(xs, ys)
        return puck_dot,

    # ----------------------------------------------------------------------------------------------
    fig1 = plt.figure()
    puck_dot, = plt.plot([], [], 'ro', ms=45*pucks[0].radius)

    if avi_filename.strip():
        ani = animation.FuncAnimation(fig1, update_plot, frame_list,
                                      init_func=init_plot, blit=True, save_count=len(frame_list[0]))

        sys.stderr.write(f'creating "{avi_filename}"\n')
        Writer = animation.writers['ffmpeg']
        writer = Writer(fps=fps, metadata=dict(artist='bs-lab'), bitrate=1800)
        ani.save(avi_filename, writer=writer)

    else:
        init_plot()

        for f in range(len(frame_list[0])):
            # plt.pause(1/fps)
            plt.pause(0.04)
            for p, puck in enumerate(pucks):
                x = frame_list[p][f][0]
                y = frame_list[p][f][1]
                plt.plot(x, y, 'r.')
                xx, yy = make_circle_points([x, y], puck.radius)
                plt.plot(xx, yy, 'r')
            # plt.draw()

        # prevent figure from disappearing
        plt.show()
