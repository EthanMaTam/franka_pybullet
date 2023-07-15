import os
import numpy as np
import string
import math
import random

from itertools import cycle
from collections import namedtuple

from pybullet_planning import joints_from_names, joint_from_name, Attachment, link_from_name, get_unit_vector, \
    unit_pose, BodySaver, multiply, Pose, INF, adjust_path, waypoints_from_path, get_max_velocity, \
    get_link_subtree, clone_body, get_all_links,  set_color, LockRenderer, get_body_name,  BASE_LINK, get_link_descendants, create_attachment, \
    flatten_links, has_link,  get_pairs, get_difference, safe_zip, clip, \
    get_joint_positions,  sample_placement, get_pose, set_joint_positions, get_moving_links

from franka_tools.franka_ik import check_ik_reachable

FRANKA_PANDA = 'panda'
FRANKA_GRIPPER_LINK = 'panda_link7'

PLACE_TRIAL_TIMES = np.inf

def get_descendant_obstacles(body, link=BASE_LINK):
    # TODO: deprecate?
    return {(body, frozenset([link]))
            for link in get_link_subtree(body, link)}

def get_link_obstacles(world, link_name):
    if link_name in world.movable:
        return flatten_links(world.get_body(link_name))
    elif has_link(world.kitchen, link_name):
        link = link_from_name(world.kitchen, link_name)
        return flatten_links(world.kitchen, get_link_subtree(world.kitchen, link)) # subtree?
    return set()

class Conf(object):
    def __init__(self, body, joints, values=None, init=False):
        self.body = body
        self.joints = joints
        if values is None:
            values = get_joint_positions(self.body, self.joints)
        self.values = tuple(values)
        self.init = init
    @property
    def bodies(self): # TODO: misnomer
        return flatten_links(self.body, get_moving_links(self.body, self.joints))
    def assign(self):
        set_joint_positions(self.body, self.joints, self.values)
    def iterate(self):
        yield self
    def __repr__(self):
        return 'q{}'.format(id(self) % 1000)


class FConf(Conf):
    def __repr__(self):
        if len(self.joints) == 2:
            prefix = 'gq'
        elif len(self.joints) == 7:
            prefix = 'aq'
        else:
            prefix = 'q'
        return '{}{}'.format(prefix, id(self) % 1000)
def get_gripper_link(robot):
    robot_name = get_body_name(robot)
    if robot_name == FRANKA_PANDA:
        return FRANKA_GRIPPER_LINK
    #elif robot_name == EVE:
    #    #return EVE_GRIPPER_LINK.format(a='l') # TODO: issue copying *.dae
    #    return EVE_GRIPPER_LINK.format(arm=DEFAULT_ARM)
    raise ValueError(robot_name)


def create_obj_attachment(world, obj):
    parent = world.robot
    parent_link = world.robot_tool_link
    child = obj
    return create_attachment(parent, parent_link, child, child_link=BASE_LINK)


def create_gripper(robot, visual=False):
    gripper_link = link_from_name(robot, get_gripper_link(robot))
    links = get_link_descendants(robot, gripper_link) # get_link_descendants | get_link_subtree
    with LockRenderer():
        # Actually uses the parent of the first link
        gripper = clone_body(robot, links=links, visual=False, collision=True)  # TODO: joint limits
        if not visual:
            for link in get_all_links(gripper):
                set_color(gripper, np.zeros(4), link)
    #dump_body(robot)
    #dump_body(gripper)
    #user_input()
    return gripper

def get_place_conf(world, obj, base, **kwargs):
    '''
    Get proper placement pose from world
    :param obj: objects being placed
    :param base: support obj
    :return: placement pose from world
    '''
    # bottom_link = link_from_name(base, 'base_link')
    i = 0
    while i<=PLACE_TRIAL_TIMES:
        place_pose = sample_placement(obj, base, bottom_link=BASE_LINK)
        grasp = world.grasp_dict[obj]
        place_tool_pose = multiply(place_pose, grasp.grasp_pose)

        # place_tool_pose = multiply(place_pose, invert(transfer_tool_obj))
        check, place_conf = check_ik_reachable(world, place_tool_pose)
        i+=1
        if check:
            return place_conf
    return None

def get_obj_to_tool(obj, grasp):
    '''get obj to tool_link, ensure obj has been grasped'''
    assert obj == grasp.obj, 'Wrong input obj'
    obj_to_tool = grasp.grasp_pose
    return obj_to_tool

def get_max_velocities(body, joints):
    return tuple(get_max_velocity(body, joint) for joint in joints)

