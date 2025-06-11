#!/usr/bin/env python  
import warnings
warnings.filterwarnings("ignore")
import rospy
import math
from copy import deepcopy as dc
import tf2_ros
import time
import asyncio
import numpy as np
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from moveit_commander.robot import RobotCommander
# from expressive_motion_generation import ExpressivePlanner
from expressive_motion_generation.expressive_planner import ExpressivePlanner, Task
from expressive_motion_generation.utils import make_point_at_task_from, convert_animation_to_relative
from expressive_motion_generation.effects import *

# this variable determines whether the purely functional or the expressive scenario is executed
expressive = True

robot = RobotCommander()
robot.get_group('panda_arm').set_end_effector_link('panda_hand')
robot.get_group('panda_arm').set_pose_reference_frame('world')
robot.get_group('panda_arm').set_goal_orientation_tolerance(0.01)
planner = ExpressivePlanner(robot, publish_topic="/joint_command_desired")

def print_plan():
    print("======== Current Motion Plan ========")
    for task in planner.plan:
        print(task)

def move_gripper_to(width, grab=False):
    state = JointState()
    # state.name = robot.get_group('panda_hand').get_active_joints()
    # state.position = robot.get_group('panda_hand').get_current_joint_values()
    # state.position[0] = width
    state.name = ['panda_finger_joint1', 'panda_finger_joint2']
    state.position = [width, width]
    state.header.stamp = rospy.Time()
    state.header.frame_id = 'panda_link0'
    
    pub = rospy.Publisher('/joint_command_desired', JointState, queue_size=5)
    pub.publish(state)
    pub.unregister

    if grab:
        pub = rospy.Publisher('/joint_command', JointState, queue_size=1)
        time.sleep(0.7)
        state.position = []
        state.effort = [-2.0, -2.0]
        pub.publish(state)

        pub.unregister()

# get TF data
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

# wait for tf data
while not rospy.is_shutdown_requested():
    try:
        trans = tfBuffer.lookup_transform('main_camera', 'Object', rospy.Time())
        break
    except Exception as e:
        continue

transform_camera = tfBuffer.lookup_transform('panda_link0', 'main_camera', rospy.Time()).transform.translation
transform_block = tfBuffer.lookup_transform('panda_link0', 'Object', rospy.Time()).transform.translation
transform_block_world = tfBuffer.lookup_transform('world', 'Object', rospy.Time()).transform.translation
transform_p1 = tfBuffer.lookup_transform('world', 'p_01', rospy.Time()).transform.translation
transform_p2 = tfBuffer.lookup_transform('world', 'p_02', rospy.Time()).transform.translation
transform_p3 = tfBuffer.lookup_transform('world', 'p_03', rospy.Time()).transform.translation

camera = np.array([transform_camera.x, transform_camera.y, transform_camera.z])
block  = np.array([transform_block.x, transform_block.y, transform_block.z])
block_world  = np.array([transform_block_world.x, transform_block_world.y, transform_block_world.z])
p1     = np.array([transform_p1.x, transform_p1.y, transform_p1.z])
p2     = np.array([transform_p2.x, transform_p2.y, transform_p2.z])
p3     = np.array([transform_p3.x, transform_p3.y, transform_p3.z])

orientation = np.array([-0.465, 0.885, 0.0, 0.001])
orientation /= np.linalg.norm(orientation)
pose = Pose()
pose.orientation.x = orientation[0]
pose.orientation.y = orientation[1]
pose.orientation.z = orientation[2]
pose.orientation.w = orientation[3]

pose_p1 = deepcopy(pose)
pose_p1.position.x = p1[0]
pose_p1.position.y = p1[1]
pose_p1.position.z = p1[2]

pose_p2 = deepcopy(pose)
pose_p2.position.x = p2[0]
pose_p2.position.y = p2[1]
pose_p2.position.z = p2[2]

pose_p3 = deepcopy(pose)
pose_p3.position.x = p3[0]
pose_p3.position.y = p3[1]
pose_p3.position.z = p3[2]

pose_block = deepcopy(pose)
pose_block.position.x = block_world[0]
pose_block.position.y = block_world[1]
pose_block.position.z = block_world[2]

robot.get_group('panda_arm').set_named_target('ready')
robot.get_group('panda_arm').set_max_velocity_scaling_factor(1.0)
robot.get_group('panda_arm').set_max_acceleration_scaling_factor(1.0)
robot.get_group('panda_arm').go()

# ************************************************************************************************************************ #
# ========================================== SCENARIO START ============================================================== #
# ************************************************************************************************************************ #

# 1. move to P1 and look at after that (E only)
move_gripper_to(0.04)
if expressive:
    planner.new_plan()
    planner.plan_target(pose_p1, 'panda_arm', 0.8, 0.8, 'pose')
    planner.bake()
    task = make_point_at_task_from(robot, 'panda_arm',
                                    block,
                                    'panda_hand',
                                    planner.get_last_joint_state(),
                                    1.5)
    
    # take just the pointing state and create new trajectory from it
    state = task.target.positions[-1]
    
    planner.new_plan()
    animation = Animation()
    animation.times = np.array([0, 2])
    animation.positions = np.array([robot.get_group('panda_arm').get_current_joint_values(),
                                    state])
    animation.move_group = 'panda_arm'
    animation.joint_names = robot.get_group('panda_arm').get_active_joints()
    animation.frame_id = 'panda_link0'
    animation.name = 'look1'
    animation.original_indices = range(len(animation.times))
    animation.add_bezier(0, 1, [0.3, 0.2], [0.7, 0.8])
    animation._reload_trajectory()
    planner.add_task(Task(animation))
    planner.plan_pause(3.0)
    planner.bake()
    # planner.execute()

# 2. move to P2 and look at object all the time (E only)
if expressive:
    planner.plan_target(pose_p2, 'panda_arm', 0.8, 0.8, 'pose')
    # planner.get_task_at(2).add_effects(GazeEffect(block,
    #                                             'panda_hand',
    #                                             'panda_arm',
    #                                             movable=['panda_joint3', 'panda_joint5', 
    #                                                      'panda_joint6', 'panda_joint7']))
    planner.bake()
    task = make_point_at_task_from(robot, 'panda_arm',
                                    block,
                                    'panda_hand',
                                    planner.get_last_joint_state(),
                                    1.5)
    # take just the pointing state and create new trajectory from it
    state = task.target.positions[-1]
    
    planner.plan.pop(-1)
    animation = Animation()
    animation.times = np.array([0, 2])
    animation.positions = np.array([planner.get_last_joint_state(),
                                    state])
    animation.move_group = 'panda_arm'
    animation.joint_names = robot.get_group('panda_arm').get_active_joints()
    animation.frame_id = 'panda_link0'
    animation.name = 'look2'
    animation.original_indices = range(len(animation.times))
    animation._reload_trajectory()
    planner.add_task(Task(animation))
    planner.add_task(Task(2.0))
    planner.bake()

    planner.execute(debug_output=True)

# 3. move above object, grab it
planner.new_plan()

if not expressive:
    pose_above = deepcopy(pose_block)
    pose_above.position.z += 0.4
    planner.plan_target(pose_above, 'panda_arm', 0.7, 0.7)
    planner.plan_pause(1.0)

pose_block.position.z += 0.2 + 0.05
planner.plan_target(pose_block, 'panda_arm', 0.3, 0.3)
planner.plan_pause(1.0)

planner.execute(debug_output=True)

planner.new_plan()
pose_block.position.z -= 0.037 + 0.05
planner.plan_target(pose_block, 'panda_arm', 0.5, 0.5)
planner.execute(debug_output=True)

# grab
time.sleep(0.5)
move_gripper_to(0.0265)
time.sleep(1.0)

planner.new_plan()
pose_block.position.z += 0.037
planner.plan_target(pose_block, 'panda_arm', 0.5, 0.5)
planner.execute(debug_output=True)


# 4. move it over the gap to the table
planner.new_plan()
planner.plan_target(pose_p3, 'panda_arm', 0.5, 0.5)
planner.execute(debug_output=True)


# 5. let it fall down


# 6. look at the viewer for help (E only)
if expressive:
    planner.new_plan()
    planner.add_task(make_point_at_task_from(robot, 'panda_arm',
                                            camera,
                                            'panda_hand',
                                            robot.get_group('panda_arm').get_current_joint_values(),
                                            1.5))
    planner.get_task_at(0).target.add_bezier(0, 1, (0.2, 0.5), (0.8, 0.96))
    planner.plan_pause(1.0)

    # look at the block again
    planner.add_task(make_point_at_task_from(robot, 'panda_arm',
                                            block,
                                            'panda_hand',
                                            planner.get_last_joint_state(),
                                            1.5))
    planner.get_task_at(0).target.add_bezier(0, 1, (0.5, 0.3), (0.5, 0.7))
    planner.plan_pause(1.0)

planner.bake()
planner.execute()

# 7. be sad (E only)
upper_limits = [joint.max_bound() for joint in (robot.get_joint(joint_name) for joint_name in robot.get_active_joint_names('panda_arm'))]
lower_limits = [joint.min_bound() for joint in (robot.get_joint(joint_name) for joint_name in robot.get_active_joint_names('panda_arm'))]

if expressive:
    robot.get_group('panda_arm').set_named_target('ready')
    robot.get_group('panda_arm').set_max_velocity_scaling_factor(0.6)
    robot.get_group('panda_arm').set_max_acceleration_scaling_factor(0.6)
    robot.get_group('panda_arm').go()

    planner.new_plan()
    planner.plan_animation("/home/mwiebe/noetic_ws/IsaacSim-ros_workspaces/noetic_ws/panda_animations/animation_mistake_sad.yaml")
    planner.get_task_at(0).add_effects(ExtentEffect(-0.03,
                                                    ['g','n','g','p','m','p','i','i'],
                                                    upper_limits,
                                                    lower_limits
                                                    ))
    planner.execute()
