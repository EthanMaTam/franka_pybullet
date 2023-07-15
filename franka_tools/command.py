import math
import numpy as np
import time

from pybullet_planning import get_moving_links, set_joint_positions, create_attachment, \
    wait_for_duration, flatten_links, remove_handles, \
    batch_ray_collision, draw_ray, wait_for_user, WorldSaver, adjust_path, waypoints_from_path
from .retime import interpolate_path, decompose_into_paths


DEFAULT_TIME_STEP = 0.02
DEFAULT_SLEEP = 0.5
FORCE = 50 # 20 | 50 | 100


class State(object):
    # TODO: rename to be world state?
    def __init__(self, world, savers=[], attachments=[]):
        # a part of the state separate from PyBullet
        self.world = world
        self.savers = tuple(savers)
        self.attachments = {attachment.child: attachment for attachment in attachments}
    @property
    def bodies(self):
        raise NotImplementedError()
        #return {saver.body for saver in self.savers} | set(self.attachments)
    def derive(self):
        for attachment in self.attachments.values():
            # Derived values
            # TODO: topological sort
            attachment.assign()
    def copy(self):
        return State(self.world, self.savers, self.attachments.values())
        #return copy.deepcopy(self)
    def assign(self):
        for saver in self.savers:
            saver.restore()
        self.derive()
    def __repr__(self):
        return '{}({}, {})'.format(self.__class__.__name__, list(self.savers), self.attachments)

def create_state(world):
    # TODO: support initially holding
    # TODO: would be better to explicitly keep the state around
    # world.initial_saver.restore()
    world_saver = WorldSaver()
    attachments = []
    # for obj_name in world.movable:
    #     surface_name = world.get_supporting(obj_name)
    #     if surface_name is not None:
    #         attachments.append(create_surface_attachment(world, obj_name, surface_name))
    return State(world, savers=[world_saver], attachments=attachments)
class Command(object):
    def __init__(self, world):
        self.world = world
    #@property
    #def robot(self):
    #    return self.world.robot
    @property
    def bodies(self):
        raise NotImplementedError()
    @property
    def cost(self):
        raise NotImplementedError()
    def reverse(self):
        raise NotImplementedError()
    def iterate(self, state):
        raise NotImplementedError()
    def simulate(self, state, time_per_step=DEFAULT_TIME_STEP, **kwargs):
        for j, _ in enumerate(self.iterate(state)):
            state.derive()
            if j != 0:
                wait_for_duration(time_per_step)
    def execute(self, interface):
        raise NotImplementedError()

class Sequence(object):
    def __init__(self, context, commands=[], name=None):
        self.context = context
        self.commands = commands
        self.name = self.__class__.__name__.lower() if name is None else name
    @property
    def bodies(self):
        bodies = set(self.context.bodies)
        for command in self.commands:
            bodies.update(command.bodies)
        return bodies
    @property
    def cost(self):
        return sum([0] + [command.cost for command in self.commands])
    def reverse(self):
        return Sequence(self.context, [command.reverse() for command in reversed(self.commands)], name=self.name)
    def __repr__(self):
        #return '[{}]'.format('->'.join(map(repr, self.commands)))
        return '{}({})'.format(self.name, len(self.commands))

class Trajectory(Command):
    def __init__(self, world, robot, joints, path, speed=1.0):
        super(Trajectory, self).__init__(world)
        self.robot = robot
        self.joints = tuple(joints)
        self.path = tuple(path)
        self.speed = speed
    @property
    def bodies(self):
        # TODO: decompose into dependents and moving?
        return flatten_links(self.robot, get_moving_links(self.robot, self.joints))
    @property
    def cost(self):
        return len(self.path)
    def reverse(self):
        return self.__class__(self.world, self.robot, self.joints, self.path[::-1])
    def iterate(self, state):
        #time_parameterization(self.robot, self.joints, self.path)
        for positions in self.path:
            set_joint_positions(self.robot, self.joints, positions)
            yield
    def simulate(self, state, real_per_sim=1, time_step=1./120, **kwargs):

        path = list(self.path)
        path = adjust_path(self.robot, self.joints, path)
        path = waypoints_from_path(path)
        if len(path) <= 1:
            return True
        for joints, path in decompose_into_paths(self.joints, path):
            positions_curve = interpolate_path(self.robot, joints, path)
            print('Following {} {}-DOF waypoints in {:.3f} seconds'.format(len(path), len(joints), positions_curve.x[-1]))
            for t in np.arange(positions_curve.x[0], positions_curve.x[-1], step=time_step):
                positions = positions_curve(t)
                set_joint_positions(self.robot, joints, positions)
                state.derive()
                wait_for_duration(real_per_sim*time_step)
        return True
    def __repr__(self):
        return '{}({}x{})'.format(self.__class__.__name__, len(self.joints), len(self.path))

class ApproachTrajectory(Trajectory):
    def __init__(self, objects=[], *args, **kwargs):
        super(ApproachTrajectory, self).__init__(*args, **kwargs)
        assert self.joints == self.world.robot_ik_joints
        self.speed = 0.25
        self.objects = set(objects)
    @property
    def bodies(self):
        bodies = set(super(ApproachTrajectory, self).bodies) # TODO: rename to bodies
        for name in self.objects:
            bodies.update(flatten_links(self.world.get_body(name)))
        return bodies
    def reverse(self):
        return self.__class__(self.objects, self.world, self.robot, self.joints, self.path[::-1])

class Attach(Command):
    def __init__(self, world, robot, link, body):
        # TODO: names or bodies?
        super(Attach, self).__init__(world)
        self.robot = robot
        self.link = link
        self.body = body
    @property
    def bodies(self):
        return set()
        #return {self.robot, self.body}
    @property
    def cost(self):
        return 0
    def reverse(self):
        return Detach(self.world, self.robot, self.link, self.body)
    def attach(self):
        return create_attachment(self.robot, self.link, self.body)
    def iterate(self, state):
        state.attachments[self.body] = self.attach()
        yield
    def execute(self, interface):
        return True
    def __repr__(self):
        return '{}'.format(self.__class__.__name__)
        #TODO : After adding names to objects
        # return '{}({})'.format(self.__class__.__name__, self.world.get_name(self.body))


class AttachGripper(Attach):
    def __init__(self, world, body, grasp=None):
        super(AttachGripper, self).__init__(world, world.robot, world.robot_tool_link, body)
        self.grasp = grasp

class Detach(Command):
    def __init__(self, world, robot, link, body):
        super(Detach, self).__init__(world)
        self.robot = robot
        self.link = link
        self.body = body
    @property
    def bodies(self):
        return set()
        #return {self.robot, self.body}
    @property
    def cost(self):
        return 0
    def reverse(self):
        return Attach(self.world, self.robot, self.link, self.body)
    def iterate(self, state):
        assert self.body in state.attachments
        del state.attachments[self.body]
        yield
    def __repr__(self):
        return '{}'.format(self.__class__.__name__)
        # TODO : After adding names to objects
        # return '{}({})'.format(self.__class__.__name__, self.world.get_name(self.body))

def iterate_commands(state, commands, time_step=DEFAULT_TIME_STEP, pause=False):
    if commands is None:
        return False
    for i, command in enumerate(commands):
        print('\nCommand {:2}/{:2}: {}'.format(i + 1, len(commands), command))
        # TODO: skip to end
        # TODO: downsample
        for j, _ in enumerate(command.iterate(state)):
            state.derive()
            if j == 0:
                continue
            if time_step is None:
                wait_for_duration(1e-2)
                wait_for_user('Command {:2}/{:2} | step {:2} | Next?'.format(i + 1, len(commands), j))
            elif time_step == 0:
                pass
            else:
                wait_for_duration(time_step)
        if pause:
            wait_for_user('Continue?')
    return True

class Wait(Command):
    def __init__(self, world, steps=1, duration=1.0):
        super(Wait, self).__init__(world)
        self.steps = steps
        self.duration = duration
    @property
    def bodies(self):
        return set()
    @property
    def cost(self):
        return 0
    def reverse(self):
        return self
    def iterate(self, state):
        for _ in range(self.steps+1):
            yield
    def simulate(self, state, **kwargs):
        wait_for_duration(self.duration)
    def execute(self, interface):
        time.sleep(self.duration)
        #import rospy
        #rospy.sleep(self.duration)
        return True
    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, self.steps)


def simulate_commands(state, commands, **kwargs):
    if commands is None:
        return False
    # TODO: simulate commands simultaneously
    for i, command in enumerate(commands):
        print('\nCommand {:2}/{:2}: {}'.format(i + 1, len(commands), command))
        command.simulate(state, **kwargs)
    return True

def combine_commands(commands):
    combined_commands = []
    for command in commands:
        if not combined_commands:
            combined_commands.append(command)
            continue
        # prev_command = combined_commands[-1]
        # if isinstance(prev_command, Trajectory) and isinstance(command, Trajectory) and \
        #         (prev_command.joints == command.joints):
        #     prev_command.path = (prev_command.path + command.path)
        else:
            combined_commands.append(command)
    return combined_commands

def commands_from_plans(plans_gen):
    commands = []
    for plan_gen in plans_gen:
        plan = next(plan_gen)
        for command in plan.commands:
            commands.append(command)
    return commands
# def commands_from_plan(world, plan):
#     if plan is None:
#         return None
#     # TODO: propagate the state
#     commands = []
#     for action, params in plan:
#         # TODO: break if the action is a StreamAction
#         if action in ['move_base', 'move_arm', 'move_gripper', 'pick', 'pull', 'pour', 'press-on', 'press-off']:
#             commands.extend(params[-1].commands)
#         elif action == 'detect':
#             commands.append(params[-1])
#         elif action == 'place':
#             commands.extend(params[-1].reverse().commands)
#         elif action in ['cook', 'calibrate']:
#             # TODO: calibrate action that uses fixed_base_suppressor
#             #steps = int(math.ceil(2.0 / DEFAULT_TIME_STEP))
#             steps = 0
#             commands.append(Wait(world, steps=steps))
#         else:
#             raise NotImplementedError(action)
#     return commands