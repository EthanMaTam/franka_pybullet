import pybullet

from franka_tools.path_planning import path_grasp, path_approach, path_free_joint_space, path_place
from franka_tools.utils import create_obj_attachment
from franka_tools.world import World
from franka_tools.grasp import get_grasps, get_side_cylinder_grasps
from franka_tools.command import Trajectory, State, ApproachTrajectory, Sequence, simulate_commands, create_state, commands_from_plans
from franka_tools.franka_ik import get_proper_ik_solution
from franka_tools.common_action.pick import gripper_grasp_motion, gripper_open_motion, plan_pick, gripper_close_motion, gen_pick
from franka_tools.common_action.place import plan_place, gen_place
from franka_tools.common_action.move import restore_default
from franka_tools.common_action.press import gen_press
import pybullet_planning as pp

import numpy as np
VIDEO_TEMPLATE = 'logs/videos/grasp_demo02.mp4'

ARM_RESOLUTION = 0.05
def main():

    world = World()
    pick1 = gen_pick(world, world.cylinder2, "side")
    place1 = gen_place(world, world.cylinder2, world.base_cylinder)
    pick2 = gen_pick(world, world.box1, 'top')
    place2 = gen_place(world, world.box1, world.base_place)
    press = gen_press(world, world.knob)
    state = create_state(world)
    video_path = VIDEO_TEMPLATE
    video = pp.VideoSaver(video_path)

    plan_list = [pick2, place2, pick1, place1, press]
    commands = commands_from_plans(plan_list)
    world.set_robot_default_conf()
    simulate_commands(state, commands, speed=2)


    if video:
        video.restore()
    world.destroy()
if __name__ == '__main__':
    main()

