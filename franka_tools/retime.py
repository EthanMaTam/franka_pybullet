import math
import numpy as np

from pybullet_planning import INF, adjust_path, waypoints_from_path, \
                              get_pairs, get_difference, safe_zip, clip
from .utils import get_max_velocities

DEFAULT_SPEED_FRACTION = 0.3
def decompose_into_paths(joints, path):
    current_path = []
    joint_sequence = []
    path_sequence = []
    for q1, q2 in get_pairs(path):
        # Zero velocities aren't enforced otherwise
        indices, = np.nonzero(get_difference(q1, q2))
        current_joints = tuple(joints[j] for j in indices)
        if not joint_sequence or (current_joints != joint_sequence[-1]):
            if current_path:
                path_sequence.append(current_path)
            joint_sequence.append(current_joints)
            current_path = [tuple(q1[j] for j in indices)]
        current_path.append(tuple(q2[j] for j in indices))
    if current_path:
        path_sequence.append(current_path)
    return safe_zip(joint_sequence, path_sequence)

def interpolate_path(robot, joints, path, velocity_fraction=DEFAULT_SPEED_FRACTION,
                     k=1, bspline=False, dump=False, **kwargs):
    from scipy.interpolate import CubicSpline, interp1d
    #from scipy.interpolate import CubicHermiteSpline, KroghInterpolator
    # https://scikit-learn.org/stable/auto_examples/linear_model/plot_polynomial_interpolation.html
    # TODO: local search to retime by adding or removing waypoints
    # TODO: iteratively increase spacing until velocity / accelerations meets limits
    # https://docs.scipy.org/doc/scipy/reference/tutorial/interpolate.html
    # Waypoints are followed perfectly, twice continuously differentiable
    # TODO: https://pythonrobotics.readthedocs.io/en/latest/modules/path_tracking.html#mpc-modeling
    path, time_from_starts = retime_trajectory(robot, joints, path, velocity_fraction=velocity_fraction, sample_step=None)
    if k == 3:
        if bspline:
            positions = approximate_spline(time_from_starts, path, k=k, **kwargs)
        else:
            # bc_type= clamped | natural | ((1, 0), (1, 0))
            positions = CubicSpline(time_from_starts, path, bc_type='clamped', extrapolate=False)
    else:
        kinds = {1: 'linear', 2: 'quadratic', 3: 'cubic'} # slinear
        positions = interp1d(time_from_starts, path, kind=kinds[k], axis=0, assume_sorted=True)

    if not dump:
        return positions
    # TODO: only if CubicSpline
    velocities = positions.derivative()
    accelerations = positions.derivative()
    for i, t in enumerate(positions.x):
        print(i, round(t, 3), positions(t), velocities(t), accelerations(t))
    # TODO: compose piecewise functions
    # TODO: ramp up and ramp down path
    # TODO: quadratic interpolation between endpoints
    return positions

def approximate_spline(time_from_starts, path, k=3, approx=INF):
    from scipy.interpolate import make_interp_spline, make_lsq_spline
    x = time_from_starts
    if approx == INF:
        positions = make_interp_spline(time_from_starts, path, k=k, t=None, bc_type='clamped')
        positions.x = positions.t[positions.k:-positions.k]
    else:
        # TODO: approximation near the endpoints
        # approx = min(approx, len(x) - 2*k)
        assert approx <= len(x) - 2 * k
        t = np.r_[(x[0],) * (k + 1),
                  # np.linspace(x[0]+1e-3, x[-1]-1e-3, num=approx, endpoint=True),
                  np.linspace(x[0], x[-1], num=2 + approx, endpoint=True)[1:-1],
                  (x[-1],) * (k + 1)]
        # t = positions.t # Need to slice
        # w = np.zeros(...)
        w = None
        positions = make_lsq_spline(x, path, t, k=k, w=w)
    positions.x = positions.t[positions.k:-positions.k]
    return positions

def retime_trajectory(robot, joints, path, only_waypoints=False,
                      velocity_fraction=DEFAULT_SPEED_FRACTION, **kwargs):
    """
    :param robot:
    :param joints:
    :param path:
    :param velocity_fraction: fraction of max_velocity
    :return:
    """
    path = adjust_path(robot, joints, path)
    if only_waypoints:
        path = waypoints_from_path(path)
    max_velocities = velocity_fraction * np.array(get_max_velocities(robot, joints))
    return ramp_retime_path(path, max_velocities, **kwargs)

def ramp_retime_path(path, max_velocities, acceleration_fraction=INF, sample_step=None):
    """
    :param path:
    :param max_velocities:
    :param acceleration_fraction: fraction of velocity_fraction*max_velocity per second
    :param sample_step:
    :return:
    """
    assert np.all(max_velocities)
    accelerations = max_velocities * acceleration_fraction
    dim = len(max_velocities)
    #difference_fn = get_difference_fn(robot, joints)
    # TODO: more fine grain when moving longer distances

    # Assuming instant changes in accelerations
    waypoints = [path[0]]
    time_from_starts = [0.]
    for q1, q2 in get_pairs(path):
        differences = get_difference(q1, q2) # assumes not circular anymore
        #differences = difference_fn(q1, q2)
        distances = np.abs(differences)
        duration = max([compute_min_duration(distances[idx], max_velocities[idx], accelerations[idx])
                        for idx in range(dim)] + [0.])
        time_from_start = time_from_starts[-1]
        if sample_step is not None:
            waypoints, time_from_starts = add_ramp_waypoints(differences, accelerations, q1, duration, sample_step,
                                                             waypoints, time_from_starts)
        waypoints.append(q2)
        time_from_starts.append(time_from_start + duration)
    return waypoints, time_from_starts

def compute_min_duration(distance, max_velocity, acceleration):
    if distance == 0:
        return 0
    max_ramp_duration = max_velocity / acceleration
    if acceleration == INF:
        #return distance / max_velocity
        ramp_distance = 0.
    else:
        ramp_distance = 0.5 * acceleration * math.pow(max_ramp_duration, 2)
    remaining_distance = distance - 2 * ramp_distance
    if 0 <= remaining_distance:  # zero acceleration
        remaining_time = remaining_distance / max_velocity
        total_time = 2 * max_ramp_duration + remaining_time
    else:
        half_time = np.sqrt(distance / acceleration)
        total_time = 2 * half_time
    return total_time

def add_ramp_waypoints(differences, accelerations, q1, duration, sample_step, waypoints, time_from_starts):
    dim = len(q1)
    distances = np.abs(differences)
    time_from_start = time_from_starts[-1]

    ramp_durations = [compute_ramp_duration(distances[idx], accelerations[idx], duration)
                      for idx in range(dim)]
    directions = np.sign(differences)
    for t in np.arange(sample_step, duration, sample_step):
        positions = []
        for idx in range(dim):
            distance = compute_position(ramp_durations[idx], duration, accelerations[idx], t)
            positions.append(q1[idx] + directions[idx] * distance)
        waypoints.append(positions)
        time_from_starts.append(time_from_start + t)
    return waypoints, time_from_starts

def compute_ramp_duration(distance, acceleration, duration):
    discriminant = max(0, math.pow(duration * acceleration, 2) - 4 * distance * acceleration)
    velocity = 0.5 * (duration * acceleration - math.sqrt(discriminant))  # +/-
    #assert velocity <= max_velocity
    ramp_time = velocity / acceleration
    predicted_distance = velocity * (duration - 2 * ramp_time) + acceleration * math.pow(ramp_time, 2)
    assert abs(distance - predicted_distance) < 1e-6
    return ramp_time

def compute_position(ramp_time, max_duration, acceleration, t):
    velocity = acceleration * ramp_time
    max_time = max_duration - 2 * ramp_time
    t1 = clip(t, 0, ramp_time)
    t2 = clip(t - ramp_time, 0, max_time)
    t3 = clip(t - ramp_time - max_time, 0, ramp_time)
    #assert t1 + t2 + t3 == t
    return 0.5 * acceleration * math.pow(t1, 2) + velocity * t2 + \
           velocity * t3 - 0.5 * acceleration * math.pow(t3, 2)