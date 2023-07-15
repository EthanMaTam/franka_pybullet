from .ikfast.franka_panda.ik import FRANKA_URDF, PANDA_INFO
from .ikfast.ikfast import get_ik_joints, check_ik_solver, ikfast_inverse_kinematics, \
                                            check_solution,is_ik_compiled, closest_inverse_kinematics

from pybullet_planning import set_joint_positions, get_configuration, compute_forward_kinematics, get_joint_positions, get_collision_fn , INF

#default ik parameters
info = PANDA_INFO
USE_IKFAST = True
MAX_IK_TIME = 0.001
# MAX_IK_ATTEMPTS = 1000
NEARBY_APPROACH = INF

def check_ikfast():
    ikfast_compiled = is_ik_compiled(PANDA_INFO)
    if ikfast_compiled:
        return True
    elif not ikfast_compiled:
        return False

def get_proper_ik_solution(world, world_from_target, use_ikfast = USE_IKFAST, robot_info=info,
                           max_time=MAX_IK_TIME, nearby_tolerance=NEARBY_APPROACH):
    robot_ik_joints = world.robot_ik_joints
    tool_link = world.robot_tool_link
    collsion_fn = get_collision_fn(world.robot, world.robot_ik_joints, world.obstacles)
    assert check_ikfast(), "Ikfast is not compiled!"
    if use_ikfast:
        solutions_list = list(closest_inverse_kinematics(world.robot,
                                                        ikfast_info=robot_info,
                                                        tool_link=tool_link,
                                                        world_from_target=world_from_target,
                                                        max_time=max_time,
                                                        max_distance=nearby_tolerance,
                                                        ))
    else:
        print('Please add ikfast!')
        return None
        # solutions_list = list(pybullet_inverse_kinematics(world.robot,
        #                                                   ikfast_info=info,
        #                                                   tool_link=tool_link,
        #                                                   world_from_target=world_from_target))
    # Get the first valid ik solution
    for i in range(len(solutions_list)):
        #check rationality and collision
        check_sol = check_solution(world.robot,
                               robot_ik_joints,
                               solutions_list[i],
                               tool_link=tool_link,
                               target_pose=world_from_target,
                               tolerance=1e-6)
        if check_sol:
            ik_solution = solutions_list[i]
            # set_joint_positions(world.robot, world.robot_ik_joints, ik_solution)
            # joints_pose = get_joint_positions(world.robot, world.robot_ik_joints)
            return ik_solution
        else: continue
    return None

def check_ik_reachable(world, world_from_target):
    '''
    check if target pose is reachable， and return conf
    '''
    check = True
    solution = get_proper_ik_solution(world, world_from_target)
    if solution is None:
        check = False
    return check, solution