import numpy as np

from math import *

from pybullet_planning import BodySaver

from ..franka_ik import get_proper_ik_solution, check_ik_reachable
from ..path_planning import path_grasp, path_free_joint_space, path_approach, path_grasp_after_approaching
from ..command import Command, Sequence, Trajectory, State, Detach, ApproachTrajectory, Attach, AttachGripper
from ..grasp import get_grasps, gen_proper_grasp
from .move import gripper_open_motion, gripper_close_motion, gripper_grasp_motion

MOVE_ARM = 1
GRIPPER_RESOLUTION = 0.01
TRIAL_TIMES = 100

PRINT_FAILURES = 0

def plan_pick(world, grasp, **kwargs):
    '''
    generate a sequence of detailed actions to fulfill "pick" action
    :return action sequence
    '''
    robot_saver = BodySaver(world.robot)
    obj_saver = BodySaver(grasp.obj)
            # TODO: Need this after adding objects supporting
            # attachment = create_obj_attachment(world, grasp.obj) # create attachment between obj and supporting thing of obj
            # gripper_attachment = create_attachment(world.robot, world.robot_tool_link, grasp.obj)

            # if world.get_gripper_pose() is not (0.0, 0.0):
                    #     cmd.commands.pop(0)
                    # state.savers = [robot_saver, obj_saver]
                    # state.attachments[gripper_attachment.child] = gripper_attachment
    pregrasp_check, grasp.pregrasp_conf = check_ik_reachable(world, grasp.pregrasp_pose_world)
    grasp_check, grasp.grasp_conf = check_ik_reachable(world, grasp.grasp_pose_world)
    grasp_ik_check = pregrasp_check and grasp_check
    if not grasp_ik_check:
        if PRINT_FAILURES: print("Grasp ik check failed!")
        return None

    approach_path = path_approach(world, grasp)
    grasp_after_approaching_path = path_grasp_after_approaching(world, grasp)
    if (approach_path is None) or (grasp_after_approaching_path is None):
        if PRINT_FAILURES: print("Failed to find a grasp path!")
        return None
    grasp_path = approach_path + grasp_after_approaching_path
    cmd = Sequence(context=State(world, savers=[robot_saver, obj_saver], attachments=[]),
                   commands=[gripper_open_motion(world),
                             Trajectory(world, world.robot, world.robot_ik_joints, grasp_path),
                             gripper_grasp_motion(world, grasp=grasp),
                             # Detach(world, attachment.parent, attachment.parent_link, attachment.child),
                             AttachGripper(world, grasp.obj, grasp=grasp),
                             Trajectory(world, world.robot, world.robot_ik_joints, reversed(grasp_path))],
                    name='pick')
    return cmd

def gen_pick(world, obj, grasp_type, trial_times=25, **kwargs):
    # q_init = get_joint_positions(world.robot, world.robot_ik_joints)
    # world.obstacles.remove(obj)

    for failures in range(trial_times):
        grasp = next(get_grasps(world, obj, grasp_type, **kwargs))
        # set_joint_positions(world.robot, world.robot_ik_joints, q_init)
        cmd = plan_pick(world, grasp, **kwargs)

        if cmd is not None:
            print("Succeed getting an available pick plan after {} failures".format(failures))
            world.add_obj_grasp(obj, grasp)
            yield cmd

    print("Failed to get an available pick plan after {} attempts".format(trial_times))
    # world.obstacles.append(obj)
    return None






