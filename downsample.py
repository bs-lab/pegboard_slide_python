import sys
from math import ceil, degrees
from scipy import interpolate

# constants
DEBUG = False


# --------------------------------------------------------------------------------------------------
def align_to_framerate(puck, max_time: float, framerate: int, dilation: float = 1.0) -> dict:
    """
    Interpolates the position puck data to align with the mp4 framerate.

    INPUTS:
    -------
    puck      -- PuckData instance
    max_time  -- max simulation time (when either all pucks are out of play or MAX_TIME reached)
    framerate -- frame rate (frames per second)

    OPTIONAL INPUT:
    ---------------
    dilation -- a factor used to slow down or speed up video.  A value above 1 slows down,
                  below 1 speeds up

    RETURNS:
    --------
    frame_dict -- dict of interpolated positional puck data at specific time frames whose keys
                    are time stamps
    """
    interp_func_x = interpolate.interp1d(puck.t, puck.x)
    interp_func_y = interpolate.interp1d(puck.t, puck.y)
    interp_func_v = interpolate.interp1d(puck.t, puck.v)
    interp_func_a = interpolate.interp1d(puck.t, puck.angle)
    interp_func_s = interpolate.interp1d(puck.t, puck.sliding_list)

    num_frames = ceil(max_time * framerate * dilation)
    frame_dict = {}
    for f in range(num_frames):
        t = round(f / (framerate * dilation), 8)
        x = interp_func_x(t)
        y = interp_func_y(t)
        v = interp_func_v(t)
        a = interp_func_a(t)
        s = interp_func_s(t)

        #angle = degrees(a.tolist())

        # frame_dict.append((t, x.tolist(), y.tolist(), v.tolist(), a.tolist(), s.tolist()))
        frame_dict[t] = [x.tolist(), y.tolist(), v.tolist(), degrees(a.tolist()), s.tolist()]

    # if DEBUG:
    #     print("number of frames:", len(frame_dict))
    #     print("  max time", max_time)
    #     print("  # frames", num_frames)

    return frame_dict


# --------------------------------------------------------------------------------------------------
def create_frame_data(pucks: list, fps: int = 30, dilation: int = 1) -> dict:
    """
    INPUTS:
    -------
    pucks -- list of PuckData instances

    OPTIONAL INPUT:
    ---------------
    fps      -- sampling rate (frames per second)
    dilation -- a factor used to slow down or speed up video.  A value above 1 slows down,
                  below 1 speeds up

    RETURNS:
    --------
    frame_data -- dictionary, whose keys are PuckData id's and values are dictionaries (whose
                    keys are time stamps)
    """
    max_time = max([x.final_time for x in pucks])
    frame_data = {}

    for puck in pucks:
        frame_data[puck.id] = align_to_framerate(puck, max_time, fps, dilation=dilation)

    return frame_data


# --------------------------------------------------------------------------------------------------
def convert_to_list_of_lists(dict_of_dict: dict) -> list:
    """
    Converts the time history structure from a dictionary of dictionaries to a list of lists.

    INPUTS:
    -------
    dict_of_dict -- dictionary whose keys are PuckData id's and values are dictionaries (whose
                      keys are time stamps and values are state info)

    RETURNS:
    --------
    frame_list -- a list, each entry corresponds to the next time stamp, and each entry is a list
                    of lists corresponding to the state of each puck
    """
    frame_list = []
    puck_ids = list(dict_of_dict)

    # get time stamps from any dict entry. They should all be the same.
    times = sorted(list(dict_of_dict[puck_ids[0]]))

    for t in times:
        frame = []
        for pid in puck_ids:
            frame.append(dict_of_dict[pid][t])

        frame_list.append(frame)

    return frame_list


# --------------------------------------------------------------------------------------------------
def write_dict_to_file(dict_of_dict: dict, thist_file: str):
    """
    Writes time history data to the file.

    INPUTS:
    -------
    dict_of_dict -- dictionary whose keys are PuckData id's and values are dictionaries (whose
                      keys are time stamps and values are state info)
    thist_file   -- name of time history file to be created or overwritten

    RETURNS:
    --------
    None
    """
    puck_ids = sorted(list(dict_of_dict))

    # get time stamps from any dict entry. They should all be the same.
    times = sorted(list(dict_of_dict[puck_ids[0]]))

    sys.stderr.write(f'creating "{thist_file}"\n')
    with open(thist_file, 'w') as fo:
        for t in times:
            for pid in puck_ids:
                entry = dict_of_dict[pid][t]
                fo.write(f'{pid} {t:8.4f} {entry[0]:12.8f} {entry[1]:12.8f} {entry[2]:12.8f}')
                fo.write(f'{entry[3]:12.6f} {entry[4]:1.0f}\n')

