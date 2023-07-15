import numpy as np

from math import *

from pybullet_planning import plan_joint_motion, plan_direct_joint_motion, get_moving_links



BASE_LINK = -1

PRINT_FAILURES = True
SELF_COLLISIONS = True
MOVE_ARM = True

GRIPPER_RESOLUTION = 0.01
ARM_RESOLUTION = 0.05
MAX_CONF_DISTANCE = 0.75
NEARBY_APPROACH = 0.75


def path_free_joint_space(world, end_conf, attachments=[]):
    robot = world.robot
    ik_joints = world.robot_ik_joints
    resolutions = ARM_RESOLUTION * np.ones(len(world.robot_ik_joints))
    return plan_joint_motion(robot,
                             ik_joints,
                             end_conf,
                             attachments=attachments,
                             self_collisions=SELF_COLLISIONS,
                             obstacles=world.obstacles,
                             disabled_collisions=world.disabled_collisions,
                             extra_disabled_collisions=world.gripper_collisions_list,
                             iterations=50,
                             smooth=50,
                             resolutions=resolutions)


def path_approach(world, grasp, attachments=[],  **kwargs):
    approach_conf = grasp.pregrasp_conf
    if approach_conf is None:
        print('Kinematic failure!')
        return None
    obstacles = world.obstacles
    # distance_fn = get_distance_fn(world.robot, world.robot_ik_joints)
    aq = world.carry_conf
    moving_links = get_moving_links(world.robot, world.robot_ik_joints)
    robot_obstacle = (world.robot, frozenset(moving_links))
    # distance = distance_fn(grasp_conf, approach_conf)
    resolutions = ARM_RESOLUTION * np.ones(len(world.robot_ik_joints))
    # if any(pairwise_collision(robot_obstacle, b) for b in obstacles):  # TODO: | {obj}
    #     if PRINT_FAILURES: print('Pregrasp collision failure')
    #     return None
    # if MAX_CONF_DISTANCE < distance:
    #     if PRINT_FAILURES: print('Pregrasp proximity failure (distance={:.5f})'.format(distance))
    #     return None
    aq.assign()
    approach_path = plan_joint_motion(world.robot, world.robot_ik_joints, approach_conf,
                                      attachments=attachments,
                                      obstacles=obstacles,
                                      self_collisions=SELF_COLLISIONS,
                                      disabled_collisions=[],
                                      extra_disabled_collisions=[((world.robot, world.robot_tool_link), (grasp.obj, BASE_LINK)),
                                                                 ((world.robot, 10), (grasp.obj, BASE_LINK)),
                                                                 ((world.robot, 9), (grasp.obj, BASE_LINK))],
                                      custom_limits=world.custom_limits, resolutions=resolutions,
                                      restarts=2, iterations=25, smooth=25)
    if approach_path is None:
        if PRINT_FAILURES: print('Approach path failure')
        return None
    return approach_path

def path_grasp_after_approaching(world, grasp, attachments=[], **kwargs):
    grasp_conf = grasp.grasp_conf
    obstacles = world.obstacles
    resolutions = ARM_RESOLUTION * np.ones(len(world.robot_ik_joints))
    # set_joint_positions(world.robot, world.robot_ik_joints, grasp.pregrasp_conf)
    grasp_path = plan_direct_joint_motion(world.robot, world.robot_ik_joints, grasp_conf,
                                          start_conf=grasp.pregrasp_conf,
                                          attachments=attachments, obstacles=obstacles,
                                          self_collisions=SELF_COLLISIONS,
                                          disabled_collisions=[],
                                          extra_disabled_collisions=[
                                              ((world.robot, world.robot_tool_link), (grasp.obj, BASE_LINK)),
                                              ((world.robot, 10), (grasp.obj, BASE_LINK)),
                                              ((world.robot, 9), (grasp.obj, BASE_LINK))],
                                          custom_limits=world.custom_limits, resolutions=resolutions / 4.)

    if grasp_path is None:
        if PRINT_FAILURES: print('Grasp path failure')
        return None

    return grasp_path

def path_grasp(world, grasp, attachments=[], **kwargs):
    '''
    get a path to take robot gripper to the grasp_conf.
    This function is the main part for generating "pick" action, involved path planning part but not driving gripper.
    :param world: contain all information of simulation world -> franka_tools.world_description.World
    :param grasp: information about grasp, we mainly use grasp.grasp_conf and grasp.pregrasp_conf -> franka_tools.grasp
    :return: a set of robot joints pose,
            including path to pre-grasp pose and path from pre-grasp pose to grasp pose
    '''

    approach_path = path_approach(world, grasp, attachments=attachments, **kwargs)
    grasp_after_approaching_path = path_grasp_after_approaching(world, grasp, attachments=attachments, **kwargs)
    grasp_path = approach_path.extend(grasp_after_approaching_path)

    return grasp_path




def path_place(world, place_conf, **kwargs):

    obstacles = world.obstacles
    aq = world.carry_conf
    resolutions = ARM_RESOLUTION * np.ones(len(world.robot_ik_joints))
    aq.assign()
    place_path = plan_joint_motion(world.robot, world.robot_ik_joints, place_conf,
                                   obstacles=obstacles, attachments=[],
                                   self_collisions=SELF_COLLISIONS, disabled_collisions=world.disabled_collisions,
                                   resolutions=resolutions, iterations=25, smooth=25)

    return place_path
