from pybullet import loadURDF, setAdditionalSearchPath
import pybullet_data
from math import *
from pybullet_planning import link_from_name, connect, joints_from_names, create_cylinder, create_box, get_max_limits,  \
                                get_min_limits, get_configuration, body_from_name, set_joint_positions, reset_simulation, \
                                wait_for_user, disconnect, set_joint_position, get_joint_positions, set_camera_pose, quat_from_euler,\
                              set_pose

from franka_tools.ikfast.franka_panda.ik import FRANKA_URDF, PANDA_INFO

from franka_tools.ikfast.ikfast import get_ik_joints

from franka_tools.utils import FConf, get_obj_to_tool


BASE_LINK = -1
USE_GUI = True
DEFAULT_ARM_CONF = [0.01200158428400755, -0.5697816014289856, 5.6801487517077476e-05,
                    -2.8105969429016113, -0.00025768374325707555, 3.0363450050354004, 0.7410701513290405]


CUSTOM_LIMITS = {}
info = PANDA_INFO
#creating_world_parameters
CLEAN_GREY = [0.8,0.8,0.8,1]
CLEAN_BLUE = [0.1,0.1,0.7,1]
CLEAN_RED = [0.5,0.1,0.1,1]
KNOB_COLOR = [0.9,0.1,0.1,1]
COLOR_BASE_CLEAN_BOX = [0.2,0.2,0.1,0.9]
COLOR_BASE_CLEAN_CYLINDER = [0.5,0.3,0.7,0.9]
COLOR_BASE_PLACEMENT = [0.9,0.1,0.3,0.9]
cylinder_radius = 0.03
cylinder_height = 0.2
box_radiu = 0.03
base_height = 0.1
base_width = 0.6
base_length = 0.2
placement_base_width = 0.5
knob_height = 0.01
knob_length = 0.04
class World(object):
    def __init__(self, use_gui = USE_GUI):
        self.client = connect(use_gui=use_gui)
        self.add_items()

        self.disabled_collisions = []

        self.custom_limits = CUSTOM_LIMITS
        set_camera_pose([1,-0.7,1])
        self.body_from_name = {}
        self.default_conf = DEFAULT_ARM_CONF
        self.set_initial_conf()
        # self.gripper = create_gripper(self.robot)
        self.carry_conf = FConf(self.robot, self.robot_ik_joints, self.default_conf)
        self.open_gq = FConf(self.robot, self.robot_gripper_joints,
                             get_max_limits(self.robot, self.robot_gripper_joints))
        self.closed_gq = FConf(self.robot, self.robot_gripper_joints,
                               get_min_limits(self.robot, self.robot_gripper_joints))
        self.gripper_confs = [self.closed_gq]

        self.obstacles = [self.plane, self.base_cylinder, self.base_box, self.base_place, self.box1, self.cylinder2, self.knob]
        # self.gripper_collisions_list = [((self.robot, self.robot_tool_link), (i, BASE_LINK) for i in self.obstacles),
        #                                 ((self.robot, 10), ((i, BASE_LINK) for i in self.obstacles)),
        #                                 ((self.robot, 9), ((i, BASE_LINK) for i in self.obstacles))]
        self.grasp_dict = {}

    def add_items(self):
        self.added_path = setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robot = loadURDF(FRANKA_URDF, useFixedBase=1)
        self.plane = loadURDF("plane.urdf", useFixedBase=1)
        self.cylinder2 = create_cylinder(cylinder_radius, cylinder_height, color=CLEAN_BLUE)
        self.box1 = create_box(box_radiu, box_radiu, cylinder_height, color=CLEAN_GREY)
        self.knob = create_box(knob_length, knob_length, knob_height, color=KNOB_COLOR)
        self.base_box = create_box(base_length, base_width, base_height, color=COLOR_BASE_CLEAN_BOX)
        self.base_cylinder = create_box(base_width, base_length, base_height, color=COLOR_BASE_CLEAN_CYLINDER)
        self.base_place = create_box(placement_base_width, base_length, base_height, color=COLOR_BASE_PLACEMENT)
    def set_initial_conf(self):
        set_joint_positions(self.robot, self.robot_ik_joints, self.default_conf)
        set_pose(self.base_box, ([0.5, 0, 0.05], [0,0,0,1]))
        set_pose(self.base_cylinder, ([-0.1,-0.5,0.05], [0,0,0,1]))
        set_pose(self.base_place, ([-0.1, 0.5, 0.05], [0,0,0,1]))
        set_pose(self.box1, ([0.5, 0.1, 0.2],quat_from_euler([0,0,-0.9])))
        set_pose(self.knob, ([0.56, 0., 0.1], quat_from_euler([0,0,0])))
        set_pose(self.cylinder2, ([0.5, -0.2, 0.2], [0, 0, 0, 1]))

    def close_gripper(self):
        self.closed_gq.assign()
    def open_gripper(self):
        self.open_gq.assign()
    def set_robot_default_conf(self):
        return set_joint_positions(self.robot, self.robot_ik_joints, self.default_conf)
    def get_body(self, name):
        return self.body_from_name[name]

    def get_gripper_pose(self):
        '''Get current gripper pose'''
        gripper_pose = get_joint_positions(self.robot, self.robot_gripper_joints)
        gripper_gq = FConf(self.robot, self.robot_gripper_joints, values=gripper_pose)
        return gripper_gq

    def open_gripper(self):
        self.open_gq.assign()

    def add_obj_grasp(self, obj, grasp):
        # obj_to_tool = get_obj_to_tool(obj, grasp)
        self.grasp_dict[obj] = grasp
    def remove_obj_grasp(self, obj):
        self.grasp_dict.pop(obj)
    @property
    def robot_tool_link(self):
        return link_from_name(self.robot, "tool_link")

    @property
    def robot_gripper_joints(self):
        joints_names = ['panda_finger_joint{}'.format(1+i) for i in range(2)]
        return joints_from_names(self.robot, joints_names)

    @property
    def robot_ik_joints(self):
        return get_ik_joints(self.robot, info, self.robot_tool_link)

    @property
    def robot_grasp_joints(self):
        '''
        After approaching, we don't need rotation of the seventh joint of franka, so we use 6-DOF waypoints instead 7-DOF
        '''
        joints_names = ['panda_joint{}'.format(i+1) for i in range(6)]
        return joints_from_names(self.robot, joints_names)
    def destroy(self):
        wait_for_user('Press enter to reset and terminate.')
        reset_simulation()
        disconnect()











