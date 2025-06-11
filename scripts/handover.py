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
from expressive_motion_generation.expressive_planner import ExpressivePlanner
from expressive_motion_generation.utils import make_point_at_task_from, convert_animation_to_relative
from expressive_motion_generation.effects import *

# this variable determines whether the purely functional or the expressive scenario is executed
expressive = False


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

orientation = np.array([-0.465, 0.885, 0.0, 0.001])
orientation /= np.linalg.norm(orientation)

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

# wait for tf data
while not rospy.is_shutdown_requested():
    try:
        trans = tfBuffer.lookup_transform('p1', 'p2', rospy.Time())
        break
    except Exception as e:
        continue

transform_p1     = tfBuffer.lookup_transform('p1',           'world', rospy.Time()).transform.translation
transform_p2     = tfBuffer.lookup_transform('p2',           'world', rospy.Time()).transform.translation
transform_yellow = tfBuffer.lookup_transform('yellow_block', 'world', rospy.Time()).transform.translation
transform_yellow_robot = tfBuffer.lookup_transform('panda_link0', 'yellow_block', rospy.Time()).transform.translation
transform_cam    = tfBuffer.lookup_transform('panda_link0',  'main_camera', rospy.Time()).transform.translation

p1     = np.array([-transform_p1.x,     -transform_p1.y,     -transform_p1.z    ])
p2     = np.array([-transform_p2.x,     -transform_p2.y,     -transform_p2.z    ])
# yellow = np.array([-transform_yellow.x, -transform_yellow.y, -transform_yellow.z])
yellow_robot = np.array([transform_yellow_robot.x, transform_yellow_robot.y, transform_yellow_robot.z])
yellow = np.array([0.40566, 3.08276, 1.3729])
p3     = np.array([-0.2450, 3.3694,  1.5455])

#camera = np.array([-transform_cam.x, -transform_cam.y, -transform_cam.z])
# camera = np.array([1.4, -1.37, 0.58])
camera = np.array([transform_cam.x, transform_cam.y, transform_cam.z])

pose = Pose()
pose.orientation.x = orientation[0]
pose.orientation.y = orientation[1]
pose.orientation.z = orientation[2]
pose.orientation.w = orientation[3]

# make motion plan
planner.new_plan()


# 1. Pretend like arm is working on something
for i in range(2):
    pose.position.x = p2[0]
    pose.position.y = p2[1]
    pose.position.z = p2[2]
    planner.plan_target(dc(pose), velocity_scaling=1.0, acceleration_scaling=1.0, target_type='pose')

    pose.position.z += 0.10
    planner.plan_target(dc(pose))

    pose.position.x = p1[0]
    pose.position.y = p1[1]
    planner.plan_target(dc(pose))

    pose.position.z = p1[2]
    planner.plan_target(dc(pose))

    pose.position.x = p2[0]
    pose.position.y = p2[1]
    pose.position.z = p2[2] + 0.1
    planner.plan_target(dc(pose))

planner.bake()

planner.execute()

# wait a second
time.sleep(0.5)


# 2. Look at the viewer
if expressive:
    planner.new_plan()
    planner.add_task(make_point_at_task_from(robot, 'panda_arm', camera, 'panda_hand', 
                                            robot.get_group('panda_arm').get_current_joint_values(), 
                                            1.2))
    planner.bake()

# 3. Play joy animation and look at object
if expressive:
    planner.plan_animation("/home/mwiebe/noetic_ws/IsaacSim-ros_workspaces/noetic_ws/panda_animations/animation_handover_joy.yaml")
    planner.plan_animation("/home/mwiebe/noetic_ws/IsaacSim-ros_workspaces/noetic_ws/panda_animations/animation_handover_joy_grippy.yaml")

    convert_animation_to_relative(planner.get_task_at(1).target)
    convert_animation_to_relative(planner.get_task_at(2).target)
    planner.get_task_at(1).target._reload_trajectory(planner.get_task_at(0).get_last_joint_state())
    planner.get_task_at(2).target._reload_trajectory([0.0])

    planner.bake()
    planner.execute()

    time.sleep(0.2)
    planner.new_plan()
    planner.add_task(make_point_at_task_from(robot, 'panda_arm', yellow_robot, 'panda_hand', 
                                            robot.get_group('panda_arm').get_current_joint_values(), 
                                            1.2))
    planner.bake()

move_gripper_to(0.04)

# 4. Pick object
pose.position.x = yellow[0]
pose.position.y = yellow[1]
pose.position.z = yellow[2] + 0.06
planner.plan_target(dc(pose), move_group="panda_arm")
planner.execute()

time.sleep(0.5)

pose.position.z -= 0.1
planner.plan_target(dc(pose), move_group="panda_arm")
planner.execute()

time.sleep(0.5)

move_gripper_to(0.022, True)

time.sleep(1.5)

planner.new_plan()
pose.position.z += 0.15
planner.plan_target(dc(pose), move_group="panda_arm")
planner.execute()


# 5. Move cube to viewer
planner.new_plan()
pose.position.x = p3[0]
pose.position.y = p3[1]
pose.position.z = p3[2]

planner.plan_target(dc(pose), move_group="panda_arm",
                    velocity_scaling=0.4, acceleration_scaling=0.2)

planner.get_task_at(0).add_effects(GazeEffect(camera, 'panda_hand', 'panda_arm',
                                              [1, 0, 0], ['panda_joint7'], -1, -1))

if expressive:
    # add irregular jitter to show excitement
    planner.get_task_at(0).add_effects(JitterEffect(amount=0.01, fill=10))

planner.execute()

# 6. Rotate to direct the viewer's gaze
planner.new_plan()
# planner.plan_animation('/home/mwiebe/noetic_ws/IsaacSim-ros_workspaces/noetic_ws/panda_animations/animation_handover_wiggle.yaml')
# convert_animation_to_relative(planner.get_task_at(0).target)

# # DEBUG
# for i, pos in enumerate(planner.get_task_at(0).target.positions):
#     for j, val in enumerate(pos):
#         if abs(val) > 0.0001 and abs(val) < 0.01:
#             planner.get_task_at(0).target.positions[i, j] = 0
#             # print(f"RELATIVE\tKF {i} Jnt {j}\t{val}")

# # print("RELATIVE:", planner.get_task_at(0).target.positions)
# planner.get_task_at(0).target._reload_trajectory(robot.get_group('panda_arm').get_current_joint_values())
# planner.execute()

# pose = robot.get_group('panda_arm').get_current_pose().pose
# pose.orientation.x = 0.841
# pose.orientation.y = -0.425
# pose.orientation.z = 0.214
# pose.orientation.w = 0.256

# planner.plan_target(pose, 'panda_arm', 0.7, 0.7)
# planner.bake()

if expressive:
    # task = make_point_at_task_from(robot, 'panda_arm', camera, 'panda_hand', 
    #                                 robot.get_group('panda_arm').get_current_joint_values(), 
    #                                 1, axis=[0.96, 0, 0.2])

    # # # modify animation to make it wiggle back and forth
    # task.target.times = np.array(range(0, 4))
    # task.target.positions = np.array(task.target.positions.tolist() * 2)
    # task.target._reload_trajectory()

    # planner.add_task(task)

    planner.plan_animation('/home/mwiebe/noetic_ws/IsaacSim-ros_workspaces/noetic_ws/panda_animations/animation_handover_wiggle3.yaml')

    convert_animation_to_relative(planner.get_task_at(0).target)
    planner.get_task_at(0).target._reload_trajectory(robot.get_group('panda_arm').get_current_joint_values())
    # planner.get_task_at(0).target.trajectory_planner.scale_global_speed(1)

    planner.execute()

