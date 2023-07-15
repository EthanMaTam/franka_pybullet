from ..grasp import get_grasp_presses, get_top_presses, get_x_presses
from ..command import State, Sequence, WorldSaver, Trajectory, ApproachTrajectory, Wait
from ..utils import FConf
from ..franka_ik import check_ik_reachable
from ..path_planning import path_approach, path_grasp_after_approaching
from .move import gripper_close_motion
from pybullet_planning import BodySaver, get_sample_fn, set_joint_positions, multiply, invert, get_moving_links, pairwise_collision, link_from_name,\
                            get_pose, get_link_pose, cycle, INF
PRINT_FAILURES = True
def plan_press(world, grasp, **kwargs):
    robot_saver = BodySaver(world.robot)
    #ik check
    pregrasp_check, grasp.pregrasp_conf = check_ik_reachable(world, grasp.pregrasp_pose_world)
    grasp_check, grasp.grasp_conf = check_ik_reachable(world, grasp.grasp_pose_world)
    grasp_ik_check = pregrasp_check and grasp_check
    if not grasp_ik_check:
        if PRINT_FAILURES: print("Press ik check failed!")
        return None

    approach_path = path_approach(world, grasp)
    grasp_after_approaching_path = path_grasp_after_approaching(world, grasp)
    if (approach_path is None) or (grasp_after_approaching_path is None):
        if PRINT_FAILURES: print("Failed to find a press path!")
        return None
    grasp_path = approach_path + grasp_after_approaching_path
    cmd = Sequence(context=State(world, savers=[robot_saver], attachments=[]),
                   commands=[gripper_close_motion(world),
                             Trajectory(world, world.robot, world.robot_ik_joints, grasp_path),
                             Trajectory(world, world.robot, world.robot_ik_joints, reversed(grasp_path)),
                             Wait(world, duration=1)],
                   name='press')
    return cmd

# def plan_press(world, knob_name, pose, grasp, base_conf, obstacles, randomize=True, **kwargs):
#     base_conf.assign()
#     world.close_gripper()
#     robot_saver = BodySaver(world.robot)
#
#     if randomize:
#         sample_fn = get_sample_fn(world.robot, world.arm_joints)
#         set_joint_positions(world.robot, world.arm_joints, sample_fn())
#     else:
#         world.carry_conf.assign()
#     gripper_pose = multiply(pose, invert(grasp.grasp_pose))  # w_f_g = w_f_o * (g_f_o)^-1
#     #set_joint_positions(world.gripper, get_movable_joints(world.gripper), world.closed_gq.values)
#     #set_tool_pose(world, gripper_pose)
#     full_grasp_conf = world.solve_inverse_kinematics(gripper_pose)
#     #wait_for_user()
#     if full_grasp_conf is None:
#         # if PRINT_FAILURES: print('Grasp kinematic failure')
#         return
#     robot_obstacle = (world.robot, frozenset(get_moving_links(world.robot, world.arm_joints)))
#     if any(pairwise_collision(robot_obstacle, b) for b in obstacles):
#         #if PRINT_FAILURES: print('Grasp collision failure')
#         return
#     approach_pose = multiply(pose, invert(grasp.pregrasp_pose))
#     approach_path = plan_approach(world, approach_pose, obstacles=obstacles, **kwargs)
#     if approach_path is None:
#         return
#     aq = FConf(world.robot, world.arm_joints, approach_path[0]) if MOVE_ARM else world.carry_conf
#
#     #gripper_motion_fn = get_gripper_motion_gen(world, **kwargs)
#     #finger_cmd, = gripper_motion_fn(world.open_gq, world.closed_gq)
#     objects = []
#     cmd = Sequence(State(world, savers=[robot_saver]), commands=[
#         #finger_cmd.commands[0],
#         ApproachTrajectory(objects, world, world.robot, world.arm_joints, approach_path),
#         ApproachTrajectory(objects, world, world.robot, world.arm_joints, reversed(approach_path)),
#         #finger_cmd.commands[0].reverse(),
#         Wait(world, duration=1.0),
#     ], name='press')
#     yield (aq, cmd,)

################################################################################

def gen_press(world, knob, trial_times=25, **kwargs):
    world.obstacles.remove(knob)
    for trial_time in range(trial_times):
        press = next(get_grasp_presses(world, knob, **kwargs))
        press_plan = plan_press(world, press, **kwargs)
        if press_plan is not None:
            print("Succeed getting an available press plan after {} failures".format(trial_time))
            world.obstacles.append(knob)
            yield press_plan

    print("Failed to get an available press plan after {} attempts".format(trial_times))
    world.obstacles.append(knob)
    return None


# def get_fixed_press_gen_fn(world, max_attempts=25, collisions=True, teleport=False, **kwargs):
#
#     def gen(knob_name, base_conf):
#         knob_link = link_from_name(world.kitchen, knob_name)
#         pose = get_link_pose(world.kitchen, knob_link)
#         presses = cycle(get_grasp_presses(world, knob_name))
#         max_failures = FIXED_FAILURES if world.task.movable_base else INF
#         failures = 0
#         while failures <= max_failures:
#             for i in range(max_attempts):
#                 grasp = next(presses)
#                 randomize = (random.random() < P_RANDOMIZE_IK)
#                 ik_outputs = next(plan_press(world, knob_name, pose, grasp, base_conf, world.static_obstacles,
#                                              randomize=randomize, **kwargs), None)
#                 if ik_outputs is not None:
#                     print('Fixed press succeeded after {} attempts'.format(i))
#                     yield ik_outputs
#                     break  # return
#             else:
#                 if PRINT_FAILURES: print('Fixed pull failure after {} attempts'.format(max_attempts))
#                 yield None
#                 max_failures += 1
#     return gen
