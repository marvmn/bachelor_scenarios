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
transform_block = tfBuffer.lookup_transform('panda_link0', 'Object', rospy.Time()).transform.translation
transform_p1 = tfBuffer.lookup_transform('panda_link0', 'p_01', rospy.Time()).transform.translation
transform_p2 = tfBuffer.lookup_transform('panda_link0', 'p_02', rospy.Time()).transform.translation

camera = np.array([transform_camera.x, transform_camera.y, transform_camera.z])
block  = np.array([transform_block.x, transform_block.y, transform_block.z])
p1     = np.array([transform_p1.x, transform_p1.y, transform_p1.z])
p2     = np.array([transform_p2.x, transform_p2.y, transform_p2.z])

robot.get_group('panda_arm').set_named_target('ready')
robot.get_group('panda_arm').set_max_velocity_scaling_factor(1.0)
robot.get_group('panda_arm').set_max_acceleration_scaling_factor(1.0)
robot.get_group('panda_arm').go()

# ************************************************************************************************************************ #
# ========================================== SCENARIO START ============================================================== #
# ************************************************************************************************************************ #

# 1. move to P1 and look at after that (E only)
planner.new_plan()
planner.plan_target(p1, 'panda_arm', 0.8, 0.8, 'position')
planner.add_task(make_point_at_task_from(robot, 'panda_arm',
                                            block,
                                            'panda_hand',
                                            robot.get_group('panda_arm').get_current_joint_values(),
                                            1.5))

# 2. move to P2 and look at object all the time (E only)
planner.plan_target(p2, 'panda_arm', 0.8, 0.8, 'position')
planner.get_task_at(2).add_effects(GazeEffect(block,
                                              'panda_hand',
                                              'panda_arm',
                                              movable=['panda_joint5', 'panda_joint6', 'panda_joint7']))


# 3. move above object, grab it


# 4. move it over the gap to the table


# 5. let it fall down


# 6. look at the viewer for help (E only)
if expressive:
    planner.new_plan()
    planner.add_task(make_point_at_task_from(robot, 'panda_arm',
                                            camera,
                                            'panda_hand',
                                            robot.get_group('panda_arm').get_current_joint_values(),
                                            1.5))
    planner.get_task_at(0).bake()
    planner.get_task_at(0).trajectory_planner.apply_bezier_at(0, -1, (0.2, 0.5), (0.8, 0.96))
