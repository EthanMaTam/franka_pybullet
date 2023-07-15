import numpy as np

from math import *

from pybullet_planning import plan_joint_motion, plan_direct_joint_motion, get_joint_positions, \
    get_sample_fn, get_moving_links, get_distance_fn, pairwise_collision, set_joint_positions, \
    BodySaver, get_sample_fn, set_joint_positions, multiply, invert, get_moving_links, \
    pairwise_collision, uniform_pose_generator, get_movable_joints, wait_for_user, INF, get_extend_fn, \
    child_link_from_joint, create_attachment

from franka_tools.franka_ik import get_proper_ik_solution
from franka_tools.path_planning import path_free_joint_space
from franka_tools.utils import FConf, get_descendant_obstacles, get_link_obstacles
from franka_tools.utils import create_obj_attachment
from franka_tools.command import Command, Sequence, Trajectory, State, Detach, ApproachTrajectory, Attach

GRIPPER_RESOLUTION = 0.01
ARM_RESOLUTION = 0.05
MAX_CONF_DISTANCE = 0.75
NEARBY_APPROACH = 0.75
PRINT_FAILURES = True
SELF_COLLISIONS = True

def parse_fluents(world, fluents):
    obstacles = set()
    for fluent in fluents:
        predicate, args = fluent[0], fluent[1:]
        if predicate in {p.lower() for p in ['AtBConf', 'AtAConf', 'AtGConf']}:
            q, = args
            q.assign()
        elif predicate == 'AtAngle'.lower():
            j, a = args
            a.assign()
            link = child_link_from_joint(a.joints[0])
            obstacles.update(get_descendant_obstacles(a.body, link))
        elif predicate in 'AtWorldPose'.lower():
            # TODO: conditional effects are not being correctly updated in pddlstream
            #b, p = args
            #if isinstance(p, SurfaceDist):
            #    continue
            #p.assign()
            #obstacles.update(get_link_obstacles(world, b))
            raise RuntimeError()
        elif predicate in 'AtRelPose'.lower():
            pass
        elif predicate == 'AtGrasp'.lower():
            pass
        else:
            raise NotImplementedError(predicate)

    attachments = []
    for fluent in fluents:
        predicate, args = fluent[0], fluent[1:]
        if predicate in {p.lower() for p in ['AtBConf', 'AtAConf', 'AtGConf']}:
            pass
        elif predicate == 'AtAngle'.lower():
            pass
        elif predicate in 'AtWorldPose'.lower():
            raise RuntimeError()
        elif predicate in 'AtRelPose'.lower():
            o1, rp, o2 = args
            rp.assign()
            obstacles.update(get_link_obstacles(world, o1))
        elif predicate == 'AtGrasp'.lower():
            o, g = args
            if o is not None:
                attachments.append(g.get_attachment())
                attachments[-1].assign()
        else:
            raise NotImplementedError(predicate)
    return attachments, obstacles

def restore_default(world, trial_times = 25):
    robot_saver = BodySaver(world.robot)
    for trial_time in range(trial_times):
        restore_path = path_free_joint_space(world, world.default_conf)
        if restore_path is not None:
            cmd = Sequence(context=State(world, [robot_saver]),
                           commands=[Trajectory(world, world.robot, world.robot_ik_joints, restore_path),
                                     gripper_close_motion(world),
                                     ],
                           name='restore')
            yield cmd
    print("Failed to restore default")
    return None
def gripper_grasp_motion(world, gq1=None, gq2=None, grasp=None):
    resolutions = GRIPPER_RESOLUTION * np.ones(len(world.robot_gripper_joints))
    if grasp is not None:
        gq1 = world.gripper_confs[-1]
        gq2 = grasp.get_gripper_conf()
    extend_fn = get_extend_fn(gq2.body, gq2.joints, resolutions=resolutions)
    path = [gq1.values] + list(extend_fn(gq1.values, gq2.values))
    trajectory = Trajectory(world, gq2.body, gq2.joints, path)
    # world.gripper_confs.pop(gq1)
    world.gripper_confs.append(gq2)
    return trajectory

def gripper_open_motion(world):
    resolutions = GRIPPER_RESOLUTION * np.ones(len(world.robot_gripper_joints))
    gq1 = world.gripper_confs[-1]
    gq2 = world.open_gq
    extend_fn = get_extend_fn(gq1.body, gq1.joints, resolutions=resolutions)
    path = [gq1.values] + list(extend_fn(gq1.values, gq2.values))
    trajectory = Trajectory(world, gq2.body, gq2.joints, path)
    # world.gripper_confs.pop(gq1)
    world.gripper_confs.append(gq2)
    return trajectory

def gripper_close_motion(world):
    resolutions = GRIPPER_RESOLUTION * np.ones(len(world.robot_gripper_joints))
    gq1 = world.gripper_confs[-1]
    gq2 = world.closed_gq
    extend_fn = get_extend_fn(gq1.body, gq1.joints, resolutions=resolutions)
    path = [gq1.values] + list(extend_fn(gq1.values, gq2.values))
    trajectory = Trajectory(world, gq2.body, gq2.joints, path)
    # world.gripper_confs.pop(gq1)
    world.gripper_confs.append(gq2)
    return trajectory

