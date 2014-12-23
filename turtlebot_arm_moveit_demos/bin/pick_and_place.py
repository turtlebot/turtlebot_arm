#!/usr/bin/env python

"""
    pick_and_place.py - Version 0.1 2014-08-01
    
    Command the gripper to grasp a target object and move it to a new
    location, all while avoiding simulated obstacles.
    
      Before running, set environment variable TURTLEBOT_ARM1 to either:
       turtlebot - for original turtlebot arm
       pincher - for PhantomX Pincher arm
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel. All rights reserved.
    
    Adapted to the Turtlebot arm by Jorge Santos

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys, tf
import moveit_commander
from math import *
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation
from moveit_msgs.msg import MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy

GROUP_NAME_ARM = 'arm'
GROUP_NAME_GRIPPER = 'gripper'

GRIPPER_FRAME = 'gripper_link'
GRIPPER_JOINT_NAMES = ['gripper_joint']
GRIPPER_EFFORT = [1.0]
GRIPPER_PARAM = '/gripper_controller'

REFERENCE_FRAME = '/base_link'
ARM_BASE_FRAME = '/arm_base_link'

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node('moveit_demo')
               
        self.gripper_opened = [rospy.get_param(GRIPPER_PARAM + "/max_opening") ]
        self.gripper_closed = [rospy.get_param(GRIPPER_PARAM + "/min_opening") ]
        self.gripper_neutral = [rospy.get_param(GRIPPER_PARAM + "/neutral") ]
        
        self.gripper_tighten = rospy.get_param(GRIPPER_PARAM + "/tighten") 

        # We need a tf listener to convert poses into arm reference base
        self.tf_listener = tf.TransformListener()
        
        # Use the planning scene object to add or remove objects
        scene = PlanningSceneInterface()

        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)

        # Create a publisher for displaying gripper poses
        self.gripper_pose_pub = rospy.Publisher('target_pose', PoseStamped, queue_size=10)

        # Create a dictionary to hold object colors
        self.colors = dict()

        # Initialize the move group for the right arm
        arm = MoveGroupCommander(GROUP_NAME_ARM)

        # Initialize the move group for the right gripper
        gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)

        # Get the name of the end-effector link
        end_effector_link = arm.get_end_effector_link()

        # Allow some leeway in position (meters) and orientation (radians)
        arm.set_goal_position_tolerance(0.04)
        arm.set_goal_orientation_tolerance(0.1)

        # Allow replanning to increase the odds of a solution
        arm.allow_replanning(True)

        # Set the right arm reference frame
        arm.set_pose_reference_frame(REFERENCE_FRAME)

        # Allow 5 seconds per planning attempt
        arm.set_planning_time(5)

        # Set a limit on the number of pick attempts before bailing
        max_pick_attempts = 3

        # Set a limit on the number of place attempts
        max_place_attempts = 3
        rospy.loginfo("Scaling for MoveIt timeout=" + str(rospy.get_param('/move_group/trajectory_execution/allowed_execution_duration_scaling')))

        # Give the scene a chance to catch up
        rospy.sleep(2)

        # Give each of the scene objects a unique name
        table_id = 'table'
        box1_id = 'box1'
        box2_id = 'box2'
        target_id = 'target'
        tool_id = 'tool'

        # Remove leftover objects from a previous run
        scene.remove_world_object(table_id)
        scene.remove_world_object(box1_id)
        scene.remove_world_object(box2_id)
        scene.remove_world_object(target_id)
        scene.remove_world_object(tool_id)

        # Remove any attached objects from a previous session
        scene.remove_attached_object(GRIPPER_FRAME, target_id)

        # Give the scene a chance to catch up
        rospy.sleep(1)

        # Start the arm in the "arm_up" pose stored in the SRDF file
        rospy.loginfo("Set Arm: right_up")
        arm.set_named_target('right_up')
        if arm.go() != True:
            rospy.logwarn("  Go failed")
        rospy.sleep(2)

        # Move the gripper to the closed position
        rospy.loginfo("Set Gripper: Close " + str(self.gripper_closed ) )
        gripper.set_joint_value_target(self.gripper_closed)   
        if gripper.go() != True:
            rospy.logwarn("  Go failed")
        rospy.sleep(2)
        
        # Move the gripper to the neutral position
        rospy.loginfo("Set Gripper: Neutral " + str(self.gripper_neutral) )
        gripper.set_joint_value_target(self.gripper_neutral)
        if gripper.go() != True:
            rospy.logwarn("  Go failed")
        rospy.sleep(2)

        # Move the gripper to the open position
        rospy.loginfo("Set Gripper: Open " +  str(self.gripper_opened))
        gripper.set_joint_value_target(self.gripper_opened)
        if gripper.go() != True:
            rospy.logwarn("  Go failed")
        rospy.sleep(2)
            
        # Set the height of the table off the ground
        table_ground = 0.4

        # Set the dimensions of the scene objects [l, w, h]
        table_size = [0.2, 0.7, 0.01]
        box1_size = [0.1, 0.05, 0.05]
        box2_size = [0.05, 0.05, 0.15]

        # Set the target size [l, w, h]
        target_size = [0.02, 0.005, 0.12]

        # Add a table top and two boxes to the scene
        table_pose = PoseStamped()
        table_pose.header.frame_id = REFERENCE_FRAME
        table_pose.pose.position.x = 0.36
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = table_ground + table_size[2] / 2.0
        table_pose.pose.orientation.w = 1.0
        scene.add_box(table_id, table_pose, table_size)

        box1_pose = PoseStamped()
        box1_pose.header.frame_id = REFERENCE_FRAME
        box1_pose.pose.position.x = table_pose.pose.position.x - 0.04
        box1_pose.pose.position.y = 0.0
        box1_pose.pose.position.z = table_ground + table_size[2] + box1_size[2] / 2.0
        box1_pose.pose.orientation.w = 1.0
        scene.add_box(box1_id, box1_pose, box1_size)

        box2_pose = PoseStamped()
        box2_pose.header.frame_id = REFERENCE_FRAME
        box2_pose.pose.position.x = table_pose.pose.position.x - 0.06
        box2_pose.pose.position.y = 0.2
        box2_pose.pose.position.z = table_ground + table_size[2] + box2_size[2] / 2.0
        box2_pose.pose.orientation.w = 1.0
        scene.add_box(box2_id, box2_pose, box2_size)

        # Set the target pose in between the boxes and on the table
        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.pose.position.x = table_pose.pose.position.x - 0.03
        target_pose.pose.position.y = 0.1
        target_pose.pose.position.z = table_ground + table_size[2] + target_size[2] / 2.0
        target_pose.pose.orientation.w = 1.0

        # Add the target object to the scene
        scene.add_box(target_id, target_pose, target_size)

        # Make the table red and the boxes orange
        self.setColor(table_id, 0.8, 0, 0, 1.0)
        self.setColor(box1_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box2_id, 0.8, 0.4, 0, 1.0)

        # Make the target yellow
        self.setColor(target_id, 0.9, 0.9, 0, 1.0)

        # Send the colors to the planning scene
        self.sendColors()

        # Set the support surface name to the table object
        arm.set_support_surface_name(table_id)

        # Specify a pose to place the target after being picked up
        place_pose = PoseStamped()
        place_pose.header.frame_id = REFERENCE_FRAME
        place_pose.pose.position.x = table_pose.pose.position.x - 0.03
        place_pose.pose.position.y = -0.15
        place_pose.pose.position.z = table_ground + table_size[2] + target_size[2] / 2.0
        place_pose.pose.orientation.w = 1.0

        # Initialize the grasp pose to the target pose
        grasp_pose = target_pose

        # Shift the grasp pose by half the width of the target to center it
        grasp_pose.pose.position.y -= target_size[1] / 2.0

        # Generate a list of grasps
        grasps = self.make_grasps(grasp_pose, [target_id], [target_size[1] - self.gripper_tighten])

        # Track success/failure and number of attempts for pick operation
        result = MoveItErrorCodes.FAILURE
        n_attempts = 0

        # Repeat until we succeed or run out of attempts
        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_pick_attempts:
            rospy.loginfo("Pick attempt #" + str(n_attempts))
            for grasp in grasps:
                # Publish the grasp poses so they can be viewed in RViz
                self.gripper_pose_pub.publish(grasp.grasp_pose)
                rospy.sleep(0.2)

                result = arm.pick(target_id, grasps)
                if result == MoveItErrorCodes.SUCCESS:
                    break

            n_attempts += 1
            rospy.sleep(0.2)

        # If the pick was successful, attempt the place operation
        if result == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("  Pick: Done!")
            # Generate valid place poses
            places = self.make_places(place_pose)

            success = False
            n_attempts = 0

            # Repeat until we succeed or run out of attempts
            while not success and n_attempts < max_place_attempts:
                rospy.loginfo("Place attempt #" + str(n_attempts))
                for place in places:
                    # Publish the place poses so they can be viewed in RViz
                    self.gripper_pose_pub.publish(place)
                    rospy.sleep(0.2)

                    success = arm.place(target_id, place)
                    if success:
                        break
                
                n_attempts += 1
                rospy.sleep(0.2)

            if not success:
                rospy.logerr("Place operation failed after " + str(n_attempts) + " attempts.")
            else:
                rospy.loginfo("  Place: Done!")
        else:
            rospy.logerr("Pick operation failed after " + str(n_attempts) + " attempts.")

        # Return the arm to the "resting" pose stored in the SRDF file (passing through right_up)
        arm.set_named_target('right_up')
        arm.go()
        
        arm.set_named_target('resting')
        arm.go()

        # Open the gripper to the neutral position
        gripper.set_joint_value_target(self.gripper_neutral)
        gripper.go()

        rospy.sleep(1)

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()

        # Exit the script
        moveit_commander.os._exit(0)

    # Get the gripper posture as a JointTrajectory
    def make_gripper_posture(self, joint_positions):
        # Initialize the joint trajectory for the gripper joints
        t = JointTrajectory()

        # Set the joint names to the gripper joint names
        t.joint_names = GRIPPER_JOINT_NAMES

        # Initialize a joint trajectory point to represent the goal
        tp = JointTrajectoryPoint()

        # Assign the trajectory joint positions to the input positions
        tp.positions = joint_positions

        # Set the gripper effort
        tp.effort = GRIPPER_EFFORT

        tp.time_from_start = rospy.Duration(1.0)

        # Append the goal point to the trajectory points
        t.points.append(tp)

        # Return the joint trajectory
        return t

    # Generate a gripper translation in the direction given by vector
    def make_gripper_translation(self, min_dist, desired, vector):
        # Initialize the gripper translation object
        g = GripperTranslation()

        # Set the direction vector components to the input
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]

        # The vector is relative to the gripper frame
        g.direction.header.frame_id = GRIPPER_FRAME

        # Assign the min and desired distances from the input
        g.min_distance = min_dist
        g.desired_distance = desired

        return g

    # Generate a list of possible grasps
    def make_grasps(self, initial_pose_stamped, allowed_touch_objects, grasp_opening=[0]):
        # Initialize the grasp object
        g = Grasp()

        # Set the pre-grasp and grasp postures appropriately;
        # grasp_opening should be a bit smaller than target width
        g.pre_grasp_posture = self.make_gripper_posture(self.gripper_opened)
        g.grasp_posture = self.make_gripper_posture(grasp_opening)

        # Set the approach and retreat parameters as desired
        g.pre_grasp_approach = self.make_gripper_translation(0.01, 0.1, [1.0, 0.0, 0.0])
        g.post_grasp_retreat = self.make_gripper_translation(0.1, 0.15, [0.0, -1.0, 1.0])

        # Set the first grasp pose to the input pose
        g.grasp_pose = initial_pose_stamped

        # Pitch angles to try
        pitch_vals = [0, 0.1, -0.1, 0.2, -0.2, 0.4, -0.4]

        # Yaw angles to try; given the limited dofs of turtlebot_arm, we must calculate the heading
        # from arm base to the object to pick (first we must transform its pose to arm base frame)
        target_pose_arm_ref = self.tf_listener.transformPose(ARM_BASE_FRAME, initial_pose_stamped)
        x = target_pose_arm_ref.pose.position.x
        y = target_pose_arm_ref.pose.position.y

        self.pick_yaw = atan2(y, x)   # check in make_places method why we store the calculated yaw
        yaw_vals = [self.pick_yaw]

        # A list to hold the grasps
        grasps = []

        # Generate a grasp for each pitch and yaw angle
        for yaw in yaw_vals:
            for pitch in pitch_vals:
                # Create a quaternion from the Euler angles
                q = quaternion_from_euler(0, pitch, yaw)

                # Set the grasp pose orientation accordingly
                g.grasp_pose.pose.orientation.x = q[0]
                g.grasp_pose.pose.orientation.y = q[1]
                g.grasp_pose.pose.orientation.z = q[2]
                g.grasp_pose.pose.orientation.w = q[3]

                # Set and id for this grasp (simply needs to be unique)
                g.id = str(len(grasps))

                # Set the allowed touch objects to the input list
                g.allowed_touch_objects = allowed_touch_objects

                # Don't restrict contact force
                g.max_contact_force = 0

                # Degrade grasp quality for increasing pitch angles
                g.grasp_quality = 1.0 - abs(pitch)

                # Append the grasp to the list
                grasps.append(deepcopy(g))

        # Return the list
        return grasps

    # Generate a list of possible place poses
    def make_places(self, init_pose):
        # Initialize the place location as a PoseStamped message
        place = PoseStamped()

        # Start with the input place pose
        place = init_pose

        # A list of x shifts (meters) to try
        x_vals = [0, 0.005, -0.005] #, 0.01, -0.01, 0.015, -0.015]

        # A list of y shifts (meters) to try
        y_vals = [0, 0.005, -0.005, 0.01, -0.01] #, 0.015, -0.015]

        # A list of pitch angles to try
        pitch_vals = [0] #, 0.005, -0.005, 0.01, -0.01, 0.02, -0.02]

        # A list to hold the places
        places = []

        # Generate a place pose for each angle and translation
        for pitch in pitch_vals:
            for dy in y_vals:
                for dx in x_vals:
                    place.pose.position.x = init_pose.pose.position.x + dx
                    place.pose.position.y = init_pose.pose.position.y + dy
    
                    # Yaw angle: given the limited dofs of turtlebot_arm, we must calculate the heading from
                    # arm base to the place location (first we must transform its pose to arm base frame)
                    target_pose_arm_ref = self.tf_listener.transformPose(ARM_BASE_FRAME, place)
                    x = target_pose_arm_ref.pose.position.x
                    y = target_pose_arm_ref.pose.position.y
                    yaw = atan2(y, x) - self.pick_yaw;
                    # Note that we subtract the yaw we calculated for pick, as the picked object "carries"
                    # with him the orientation of the arm at pickup time. More details in this moveit-users
                    # group thread:  https://groups.google.com/forum/#!topic/moveit-users/-Eie-wLDbu0 
    
                    # Create a quaternion from the Euler angles
                    q = quaternion_from_euler(0, pitch, yaw)
    
                    # Set the place pose orientation accordingly
                    place.pose.orientation.x = q[0]
                    place.pose.orientation.y = q[1]
                    place.pose.orientation.z = q[2]
                    place.pose.orientation.w = q[3]
    
                    # Append this place pose to the list
                    places.append(deepcopy(place))

        # Return the list
        return places

    # Set the color of an object
    def setColor(self, name, r, g, b, a=0.9):
        # Initialize a MoveIt color object
        color = ObjectColor()

        # Set the id to the name given as an argument
        color.id = name

        # Set the rgb and alpha values given as input
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a

        # Update the global color dictionary
        self.colors[name] = color

    # Actually send the colors to MoveIt!
    def sendColors(self):
        # Initialize a planning scene object
        p = PlanningScene()

        # Need to publish a planning scene diff
        p.is_diff = True

        # Append the colors from the global color dictionary
        for color in self.colors.values():
            p.object_colors.append(color)

        # Publish the scene diff
        self.scene_pub.publish(p)

if __name__ == "__main__":
    MoveItDemo()
