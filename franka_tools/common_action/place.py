from franka_tools.path_planning import path_place
from franka_tools.command import Detach, Trajectory, WorldSaver, State, Sequence, Attach
from pybullet_planning import BodySaver, WorldSaver, is_placement, create_attachment, get_joint_positions, set_joint_positions
from franka_tools.common_action.move import gripper_open_motion, gripper_close_motion
from franka_tools.utils import get_place_conf

PRINT_FAILURES = True

def plan_place(world, obj, base, **kwargs):
    robot_saver = BodySaver(world.robot)
    obj_saver = BodySaver(obj)
    # robot_saver = BodySaver(world.robot)
    # world_saver = WorldSaver(world)
    # gripper_attachment = None
    # for attachment in state.attachments:
    #     if attachment.child == grasp.obj and attachment.parent == world.robot:
    #         gripper_attachment = attachment
    #         break
    #     else: continue
    #
    # if gripper_attachment == None:
    #     print("No attachment between robot and obj!")
    #     return None
    place_conf = get_place_conf(world, obj, base)
    obj_grasp = world.grasp_dict[obj]
    attachments = obj_grasp.get_attachment()
    place_path = path_place(world, place_conf, attachments=attachments, **kwargs)
    if place_path is None:
        if PRINT_FAILURES: print('Failed to find a place path')
        return None
    cmd = Sequence(context=State(world, savers=[robot_saver, obj_saver], attachments=[]),
                   commands=[Trajectory(world, world.robot, world.robot_ik_joints, place_path),
                             Detach(world, world.robot, world.robot_tool_link, obj),
                             gripper_open_motion(world),
                             Trajectory(world, world.robot, world.robot_ik_joints, reversed(place_path)),
                             gripper_close_motion(world),
                             ],
                   name='place')
    return cmd

def gen_place(world, obj, base, trial_times=25, **kwargs):
    # q_init = get_joint_positions(world.robot, world.robot_ik_joints)
    for trial_time in range(trial_times):
        # set_joint_positions(world.robot, world.robot_ik_joints, q_init)
        place_plan = plan_place(world, obj, base, **kwargs)
        if place_plan is not None:
            print("Succeed getting an available place plan after {} failures".format(trial_time))
            world.remove_obj_grasp(obj)
            yield place_plan
    print("Failed to get an available place plan after {} attempts".format(trial_times))
    world.remove_obj_grasp()
    return None


# def pose2d_on_surface(world, entity_name, surface_name, pose2d=UNIT_POSE2D):
#     x, y, yaw = pose2d
#     body = world.get_body(entity_name)
#     surface_aabb = compute_surface_aabb(world, surface_name)
#     z = stable_z_on_aabb(body, surface_aabb)
#     pose = Pose(Point(x, y, z), Euler(yaw=yaw))
#     set_pose(body, pose)
#     return pose
#
# def sample_placement(world, entity_name, surface_name, min_distance=0.05, robust_radius=0.025, **kwargs):
#     entity_body = world.get_body(entity_name)
#     placement_gen = get_stable_gen(world, pos_scale=0, rot_scale=0, robust_radius=robust_radius, **kwargs)
#     for pose, in placement_gen(entity_name, surface_name):
#         x, y, z = point_from_pose(pose.get_world_from_body())
#         if x < MIN_PLACEMENT_X:
#             continue
#         pose.assign()
#         if not any(pairwise_collision(entity_body, obst_body, max_distance=min_distance) for obst_body in
#                    world.body_from_name.values() if entity_body != obst_body):
#             return pose
#     raise RuntimeError('Unable to find a pose for object {} on surface {}'.format(entity_name, surface_name))
#
# def test_supported(world, body, surface_name, collisions=True):
#     # TODO: is_center_on_aabb or is_placed_on_aabb
#     surface_aabb = compute_surface_aabb(world, surface_name)
#     # TODO: epsilon thresholds?
#     if not is_placed_on_aabb(body, surface_aabb):  # , above_epsilon=z_offset+1e-3):
#         return False
#     obstacles = world.static_obstacles | get_surface_obstacles(world, surface_name)
#     if not collisions:
#         obstacles = set()
#     #print([get_link_name(obst[0], list(obst[1])[0]) for obst in obstacles
#     #       if pairwise_collision(body, obst)])
#     return not any(pairwise_collision(body, obst) for obst in obstacles)