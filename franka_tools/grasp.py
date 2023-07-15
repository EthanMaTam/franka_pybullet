import numpy as np
import math
import pybullet as p
import random

from pybullet_planning import joints_from_names, joint_from_name, Attachment, link_from_name, get_unit_vector, quat_from_euler, \
    unit_pose, BodySaver, multiply, Pose, get_extend_fn, get_moving_links, set_joint_positions, approximate_as_cylinder, \
    get_link_subtree,set_pose, randomize, unit_point, get_difference_fn, pairwise_collision, get_max_limit, get_min_limit, \
    Euler, approximate_as_prism, body_from_end_effector, get_pose, get_link_pose, get_collision_data, point_from_pose, get_collision_fn, get_quat
from franka_tools.utils import FConf
from franka_tools.franka_ik import check_ik_reachable

SIDE_GRASP_TIMES = 200
MAX_GRASP_WIDTH = 0.08
SIDE_HEIGHT_OFFSET = 0.03
GRASP_LENGTH = 0.0535
APPROACH_DISTANCE = 0.075
FINGER_EXTENT = np.array([0.02, 0.01, 0.02])
PRESS_OFFSET = FINGER_EXTENT[0]/2 + 5e-3
TRIAL_TIMES = 300
PRINT_INFORMATION = True

FINGER_EXTENT = np.array([0.02, 0.01, 0.02]) # 2cm x 1cm x 2cm
FRANKA_GRIPPER_LINK = 'panda_link7' # panda_link7 | panda_link8 | panda_hand
FRANKA_GRASP_JOINTS = ['panda_finger_joint{}'.format(1+i) for i in range(2)]


TOOL_POSE = unit_pose()
TOP_GRASP = 'top'
SIDE_GRASP = 'side'
GRASP_TYPES = [
    TOP_GRASP,
    SIDE_GRASP
]
SHAPE_TYPES = {
    p.GEOM_SPHERE: 'sphere',  # 2
    p.GEOM_BOX: 'box',  # 3
    p.GEOM_CYLINDER: 'cylinder',  # 4
    p.GEOM_MESH: 'mesh',  # 5
    p.GEOM_PLANE: 'plane',  # 6
    p.GEOM_CAPSULE: 'capsule',  # 7
    # p.GEOM_FORCE_CONCAVE_TRIMESH
}

class Attachment(object):
    def __init__(self, parent, parent_link, grasp_pose, child):
        self.parent = parent
        self.parent_link = parent_link
        self.grasp_pose = grasp_pose # gripper_from_object
        self.child = child
        #self.child_link = child_link # child_link=BASE_LINK
    @property
    def bodies(self):
        from pybullet_planning.interfaces.robots.collision import flatten_links
        return flatten_links(self.child) | flatten_links(self.parent, get_link_subtree(
            self.parent, self.parent_link))
    def assign(self):
        parent_link_pose = get_link_pose(self.parent, self.parent_link)
        child_pose = body_from_end_effector(parent_link_pose, self.grasp_pose)
        set_pose(self.child, child_pose)
        return child_pose
    def apply_mapping(self, mapping):
        self.parent = mapping.get(self.parent, self.parent)
        self.child = mapping.get(self.child, self.child)
    def to_data(self):
        from pybullet_planning.interfaces.robots.body import get_body_name, get_name
        from pybullet_planning.interfaces.robots.link import get_link_name
        data = {}
        data['parent_name'] = get_body_name(self.parent)
        data['parent_link_name'] = get_link_name(self.parent, self.parent_link)
        data['grasp_pose'] = self.grasp_pose
        child_name =  get_body_name(self.child)
        data['child_name'] = get_name(self.child) if child_name == '' else child_name
        return data
    @classmethod
    def from_data(cls, data, parent=None, child=None):
        from pybullet_planning.interfaces.env_manager.simulation import is_connected
        from pybullet_planning.interfaces.robots.body import body_from_name, get_bodies
        from pybullet_planning.interfaces.robots.link import link_from_name
        if parent is None:
            parent = body_from_name(data['parent_name'])
        else:
            assert parent in get_bodies()
        if child is None:
            child = body_from_name(data['child_name'])
        else:
            assert child in get_bodies()
        parent_link = link_from_name(parent, data['parent_link_name'])
        grasp_pose = data['grasp_pose']
        return cls(parent, parent_link, grasp_pose, child)

    def __repr__(self):
        return '{}({},{})'.format(self.__class__.__name__, self.parent, self.child)


class Grasp(object):
    def __init__(self, world, obj, grasp_type, index, grasp_pose, pregrasp_pose,
                 grasp_conf=None, pregrasp_conf=None,
                 grasp_width=None):
        self.world = world
        self.obj = obj
        self.grasp_type = grasp_type
        self.index = index
        self.grasp_pose = grasp_pose
        self.pregrasp_pose = pregrasp_pose
        self.grasp_pose_world = multiply(get_pose(self.obj), self.grasp_pose)
        self.pregrasp_pose_world = multiply(get_pose(self.obj), self.pregrasp_pose)
        # experimental
        self.grasp_conf = grasp_conf
        self.pregrasp_conf = pregrasp_conf
        self.grasp_width = grasp_width
    def assign(self):
        attachment = self.get_attachment()
        attachment.assign()
    def get_attachment(self):
        return Attachment(self.world.robot, self.world.robot_tool_link,
                          self.grasp_pose, self.obj)
    def get_gripper_conf(self):
        conf = [self.grasp_width] * len(self.world.robot_gripper_joints)
        return FConf(self.world.robot, self.world.robot_gripper_joints, conf)
    def set_gripper(self):
        return self.get_gripper_conf().assign()
    def __repr__(self):
        return '{}({}, {})'.format(self.__class__.__name__, self.grasp_type, self.index)

##################################################################################
def get_top_grasps(body, under=False, tool_pose=TOOL_POSE, body_pose=unit_pose(),
                   max_width=MAX_GRASP_WIDTH, grasp_length=GRASP_LENGTH):
    # TODO: rename the box grasps
    center, (w, l, h) = approximate_as_prism(body, body_pose=body_pose)
    reflect_z = Pose(euler=[0, math.pi, 0])
    translate_z = Pose(point=[0, 0, h / 2 - grasp_length])
    translate_center = Pose(point=point_from_pose(body_pose)-center)
    grasps = []
    if w <= max_width:
        for i in range(1 + under):
            rotate_z = Pose(euler=[0, 0, math.pi / 2 + i * math.pi])
            grasps += [multiply(tool_pose, translate_z, rotate_z,
                                reflect_z, translate_center, body_pose)]
    if l <= max_width:
        for i in range(1 + under):
            rotate_z = Pose(euler=[0, 0, i * math.pi])
            grasps += [multiply(tool_pose, translate_z, rotate_z,
                                reflect_z, translate_center, body_pose)]
    return grasps

def get_side_grasps(body, under=False, tool_pose=TOOL_POSE, body_pose=unit_pose(),
                    max_width=MAX_GRASP_WIDTH, grasp_length=GRASP_LENGTH, top_offset=SIDE_HEIGHT_OFFSET):
    # TODO: compute bounding box width wrt tool frame
    center, (w, l, h) = approximate_as_prism(body, body_pose=body_pose)
    translate_center = Pose(point=point_from_pose(body_pose)-center)
    grasps = []
    #x_offset = 0
    x_offset = h/2 - top_offset
    for j in range(1 + under):
        swap_xz = Pose(euler=[0, -math.pi / 2 + j * math.pi, -math.pi / 2])
        if w <= max_width:
            translate_z = Pose(point=[x_offset, 0, l / 2 - grasp_length])
            for i in range(2):
                rotate_z = Pose(euler=[math.pi / 2 + i * math.pi, 0, 0])
                grasps += [multiply(tool_pose, translate_z, rotate_z, swap_xz,
                                    translate_center, body_pose)]  # , np.array([w])
        if l <= max_width:
            translate_z = Pose(point=[x_offset, 0, w / 2 - grasp_length])
            for i in range(2):
                rotate_z = Pose(euler=[i * math.pi, 0, 0])
                grasps += [multiply(tool_pose, translate_z, rotate_z, swap_xz,
                                    translate_center, body_pose)]  # , np.array([l])
    return grasps

def get_side_cylinder_grasps(body, under=False, tool_pose=TOOL_POSE, body_pose=unit_pose(),
                             max_width=MAX_GRASP_WIDTH, grasp_length=GRASP_LENGTH,
                             top_offset=SIDE_HEIGHT_OFFSET):
    center, (diameter, height) = approximate_as_cylinder(body, body_pose=body_pose)
    translate_center = Pose(point_from_pose(body_pose)-center)
    grasps = []
    #x_offset = 0
    x_offset = height/2 - top_offset
    if max_width < diameter:
        return
    while True:
        thetas = np.arange(0, 2 * np.pi, np.pi/18)
        for j in range(1 + under):
            for i in range(36):
                theta = thetas[i]
                translate_rotate = ([x_offset, 0, diameter / 2 - grasp_length], quat_from_euler([0, 0, theta]))
                swap_xz = Pose(euler=[0, -math.pi / 2 + j * math.pi, 0])
                grasps += [multiply(tool_pose, translate_rotate, swap_xz, translate_center, body_pose)]
            return grasps


def close_until_collision(robot, gripper_joints, bodies=[], open_conf=None, closed_conf=None, num_steps=25, **kwargs):
    if not gripper_joints:
        return None
    if open_conf is None:
        open_conf = [get_max_limit(robot, joint) for joint in gripper_joints]
    if closed_conf is None:
        closed_conf = [get_min_limit(robot, joint) for joint in gripper_joints]
    resolutions = np.abs(np.array(open_conf) - np.array(closed_conf)) / num_steps
    extend_fn = get_extend_fn(robot, gripper_joints, resolutions=resolutions)
    close_path = [open_conf] + list(extend_fn(open_conf, closed_conf))
    collision_links = frozenset(get_moving_links(robot, gripper_joints))

    for i, conf in enumerate(close_path):
        set_joint_positions(robot, gripper_joints, conf)
        if any(pairwise_collision((robot, collision_links), body, **kwargs) for body in bodies):
            if i == 0:
                return None
            return close_path[i-1]
    return close_path[-1]

def get_grasps(world, obj, grasp_type, pre_distance=APPROACH_DISTANCE, **kwargs):

    # fraction = 0.25
    data = get_collision_data(obj)
    obj_type = data[0][2]
    obj_pose = unit_pose()
    center, extent = approximate_as_prism(obj, obj_pose)
    # assert is_valid_grasp_type(name, grasp_type)
    if obj_type == 4 and grasp_type == SIDE_GRASP:
        #     TODO: filter first
        top_offset = extent[2] / 2
        grasp_length = 1.5 * FINGER_EXTENT[2]
        # x, z = pre_distance * get_unit_vector([3, -1])

        generator = get_side_cylinder_grasps(obj, under=False, tool_pose=TOOL_POSE, body_pose=obj_pose, max_width=np.inf,
                                             grasp_length=grasp_length, top_offset=top_offset)
        generator = (multiply(Pose(euler=Euler(yaw=yaw)), grasp)
                     for grasp in generator for yaw in [0, np.pi])
        grasp_poses = randomize(list(generator))
        for i, grasp_pose in enumerate(grasp_poses):
            point = grasp_pose[0]
            x = point[0]
            y = point[1]
            radius = data[0][3][1]
            x_to_point = x + pre_distance * x/radius
            y_to_point = pre_distance*y/radius
            pre_direction = (y_to_point, x_to_point, 0)
            # post_direction = (0, z, 0)
            # pregrasp_pose = multiply(pre_direction, grasp_pose,
            #                          )
            pregrasp_pose = multiply(Pose(point=pre_direction), grasp_pose)
            grasp = Grasp(world, obj, grasp_type, i, grasp_pose, pregrasp_pose, **kwargs)
            with BodySaver(obj):
                grasp.get_attachment().assign()
                with BodySaver(world.robot):
                    grasp.grasp_width = data[0][3][1]
            yield grasp
        # grasp_poses = []
        # for i in range(SIDE_GRASP_TIMES):
        #     grasp_pose = randomize(generator)
        #     grasp_poses.append(grasp_pose)
        # for i, grasp_pose in enumerate(grasp_poses):

    else:
        if grasp_type == TOP_GRASP:
            grasp_length = 1.5 * FINGER_EXTENT[2]  # fraction = 0.5
            pre_direction = pre_distance * get_unit_vector([0, 0, 1])
            post_direction = unit_point()
            generator = get_top_grasps(obj, under=True, tool_pose=TOOL_POSE, body_pose=obj_pose,
                                        grasp_length=grasp_length, max_width=np.inf, **kwargs)
        elif grasp_type == SIDE_GRASP:
            # Take max of height and something
            grasp_length = 1.75 * FINGER_EXTENT[2]  # No problem if pushing a little
            x, z = pre_distance * get_unit_vector([3, -1])
            pre_direction = [x, 0, 0]
            post_direction = [z, 0, 0]
            # TODO:Remind below when in specific scene
            top_offset = extent[2] / 2 #if obj_type in MID_SIDE_GRASPS else 1.0 * FINGER_EXTENT[0]
            # Under grasps are actually easier for this robot
            # TODO: bug in under in that it grasps at the bottom
            generator = get_side_grasps(obj, under=False, tool_pose=TOOL_POSE, body_pose=obj_pose,
                                        grasp_length=grasp_length, top_offset=top_offset, max_width=np.inf,
                                        **kwargs)
            # if world.robot_name == FRANKA_CARTER else unit_pose()
            generator = (multiply(Pose(euler=Euler(yaw=yaw)), grasp)
                            for grasp in generator for yaw in [0, np.pi])
        else:
            raise ValueError(grasp_type)
        grasp_poses = randomize(list(generator))
        # print(grasp_poses)

            # print(grasp_poses)
            # grasp_poses = (multiply(grasp_pose, Pose(euler=Euler(
            #     yaw=random.uniform(-math.pi, math.pi)))) for grasp_pose in cycle(grasp_poses))
        for i, grasp_pose in enumerate(grasp_poses):
            pregrasp_pose = multiply(Pose(point=pre_direction), grasp_pose,
                                    Pose(point=post_direction))
            grasp = Grasp(world, obj, grasp_type, i, grasp_pose, pregrasp_pose, **kwargs)
            with BodySaver(obj):
                grasp.get_attachment().assign()
                with BodySaver(world.robot):
                    if obj_type == 3:
                        data = get_collision_data(obj)
                        grasp.grasp_width = data[0][3][1]/2
                    else:
                        grasp.grasp_width = close_until_collision(
                            world.robot, world.robot_gripper_joints, bodies=[obj])
            # print(get_joint_positions(world.robot, world.arm_joints)[-1])
            # draw_pose(unit_pose(), parent=world.robot, parent_link=world.tool_link)
            # grasp.get_attachment().assign()
            # wait_for_user()
            ##for value in get_joint_limits(world.robot, world.arm_joints[-1]):
            # for value in [-1.8973, 0, +1.8973]:
            #    set_joint_position(world.robot, world.arm_joints[-1], value)
            #    grasp.get_attachment().assign()
            #    wait_for_user()
            yield grasp

def gen_proper_grasp(world, obj, grasp_type, **kwargs):
    grasps = get_grasps(world, obj, grasp_type, pre_distance=APPROACH_DISTANCE, **kwargs)
    for grasp in grasps:
        grasp_check, grasp.grasp_conf = check_ik_reachable(world, world_from_target=grasp.grasp_pose_world)
        pregrasp_check, grasp.pregrasp_conf = check_ik_reachable(world, world_from_target=grasp.pregrasp_pose_world)
        if grasp_check and pregrasp_check:
            return grasp
    print('Failed to find proper grasp fulfilling ik constraint!')
    return None

############################################################################################

def get_x_presses(body, max_orientations=1, body_pose=unit_pose(), top_offset=PRESS_OFFSET):
    # gripper_from_object
    # TODO: update
    center, (w, _, h) = approximate_as_prism(body, body_pose=body_pose)
    translate_center = Pose(-center)
    press_poses = []
    for j in range(max_orientations):
        swap_xz = Pose(euler=[0, -math.pi / 2 + j * math.pi, 0])
        translate = Pose(point=[0, 0, w / 2 + top_offset])
        press_poses += [multiply(TOOL_POSE, translate, swap_xz, translate_center, body_pose)]
    return press_poses

def get_top_presses(body, tool_pose=TOOL_POSE, body_pose=unit_pose(), top_offset=PRESS_OFFSET, **kwargs):
    center, (_, height) = approximate_as_cylinder(body, body_pose=body_pose, **kwargs)
    reflect_z = Pose(euler=[0, math.pi, 0])
    translate_z = Pose(point=[0, 0, height / 2 + top_offset])
    translate_center = Pose(point=point_from_pose(body_pose)-center)
    while True:
        theta = random.uniform(0, 2*np.pi)
        rotate_z = Pose(euler=[0, 0, theta])
        yield multiply(tool_pose, translate_z, rotate_z,
                       reflect_z, translate_center, body_pose)

def get_grasp_presses(world, knob, pre_distance=APPROACH_DISTANCE):
    pre_direction = pre_distance * get_unit_vector([0, 0, 1])
    post_direction = unit_point()
    for i, grasp_pose in enumerate(get_top_presses(knob, link=-1,
                                                   tool_pose=TOOL_POSE, top_offset=FINGER_EXTENT[0]/2 + 5e-3)):
        pregrasp_pose = multiply(Pose(point=pre_direction), grasp_pose,
                                 Pose(point=post_direction))
        grasp = Grasp(world, knob, TOP_GRASP, i, grasp_pose, pregrasp_pose)
        yield grasp
