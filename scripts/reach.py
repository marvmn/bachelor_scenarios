#!/usr/bin/env python  
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
        trans = tfBuffer.lookup_transform('main_camera', 'green_block', rospy.Time())
        break
    except Exception as e:
        continue

transform_camera = tfBuffer.lookup_transform('panda_link0', 'main_camera', rospy.Time()).transform.translation
transform_block = tfBuffer.lookup_transform('panda_link0', 'green_block', rospy.Time()).transform.translation

camera = np.array([transform_camera.x, transform_camera.y, transform_camera.z])
block  = np.array([transform_block.x, transform_block.y, transform_block.z])

stretched_position = np.array([0.545, 0.545, 0.36])  # could be calculated automatically with block position and panda reach?
robot.get_group('panda_arm').set_named_target('ready')
robot.get_group('panda_arm').set_max_velocity_scaling_factor(1.0)
robot.get_group('panda_arm').set_max_acceleration_scaling_factor(1.0)
robot.get_group('panda_arm').go()

# ************************************************************************************************************************
# ========================================== SCENARIO START ==============================================================
# ************************************************************************************************************************

# 1. stretch to the block
positions_before = [2.215, -1.085, -0.260, -2.461, -0.233, 1.395, -2.737]
positions_streched = [1.154, 1.351, -0.701, -0.588, 0.674, 2.517, 1.128]
positions_wait = [0.996, 0.267, -0.322, -1.716, 1.240, 2.637, -0.119]
positions_normal = robot.get_group('panda_arm').get_current_joint_values()

planner.new_plan()
# planner.plan_target(stretched_position, 'panda_arm', 1.0, 0.7, 'position')

planner.plan_target(positions_before, 'panda_arm', 1.0, 1.0,
                    'joint')
planner.plan_target('ready', 'panda_arm', 1.0, 1.0, 'name')

animation = Animation()
#                           0, 1, 2, 3, 4, 5, 6, 7   8
animation.times = np.array([0, 2, 3, 5, 6, 7, 8, 10, 10.5]) * 0.7
animation.positions = np.array([positions_normal,   # 0
                                positions_streched, # 1
                                positions_streched, # 2
                                positions_wait,     # 3
                                positions_wait,     # 4
                                positions_streched, # 5
                                positions_streched, # 6
                                positions_wait,     # 7
                                positions_wait])    # 8
animation.move_group = 'panda_arm'
animation.joint_names = robot.get_group('panda_arm').get_active_joints()
animation.frame_id = 'panda_link0'
animation.name = 'stretch'
animation.original_indices = range(len(animation.times))

if not expressive:
    # functional scenario
    animation.times = animation.times[0:2]
    animation.positions = animation.positions[0:2]
    animation.original_indices = animation.original_indices[0:2]

animation._reload_trajectory()
planner.add_task(Task(animation))

if expressive:
    planner.get_task_at(2).add_effects(JitterEffect(0.1, 20, 1, 3))
    planner.get_task_at(2).add_effects(JitterEffect(0.1, 20, 5, 7))
    for i in [1, 2, 5, 6, 8]:
        print('*')
        planner.get_task_at(2).add_effects(GazeEffect(block,
                                                    'panda_hand',
                                                    'panda_arm',
                                                    start_index=i,
                                                    stop_index=i))
    
planner.bake()
print(planner.get_task_at(0).trajectory_planner.times)

print(planner.get_task_at(1).trajectory_planner.times)
planner.execute()


# 2. Look at viewer (Expressive only)
planner.new_plan()

if expressive:
    planner.add_task(make_point_at_task_from(robot, 'panda_arm',
                                            camera,
                                            'panda_hand',
                                            robot.get_group('panda_arm').get_current_joint_values(),
                                            1.5))
    planner.get_task_at(0).bake()
    planner.get_task_at(0).trajectory_planner.apply_bezier_at(0, -1, (0.2, 0.5), (0.8, 0.96))

# 3. Open (and close) gripper
if not expressive:
    move_gripper_to(0.04)

else:
    planner.plan_animation("/home/mwiebe/noetic_ws/IsaacSim-ros_workspaces/noetic_ws/panda_animations/animation_handover_joy_grippy.yaml")
    planner.execute()
    planner.execute()
