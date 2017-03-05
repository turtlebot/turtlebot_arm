TurtleBot Arm
=============

## Updated for Kinetic!

Kinetic version of turtlebot arm code. I didn't try on Jade, but it should easily work with minor changes. It's mostly the same code as on Indigo, with some improvements on turtlebot_arm_block_manipulation (see updated wiki <span style="background-color: #00FFFF">TODO</span>) and specially [turtlebot_arm_object_manipulation](https://github.com/turtlebot/turtlebot_arm/tree/kinetic-devel/turtlebot_arm_object_manipulation): a fully revised version of that demo using OKR, SMACH and MoveIt! pick and place.

## Selecting Arm Type
By default this will work with the original white/green TurtleBot arm.  To use the PhantomX Pincher, set environment variable "TURTLEBOT_ARM1" to pincher. You will need arbotix_ros version 0.11.0 or higher for PhantomX Pincher.

## Attaching the Arm to a Robot
Open your xacro-macro-magic URDF, and add something like:

       <include filename="$(find turtlebot_arm_description)/urdf/$(optenv TURTLEBOT_ARM1 turtlebot)_arm.xacro" />
       <turtlebot_arm parent="base_link" color="white" gripper_color="green"
                 joints_vlimit="1.571" pan_llimit="-2.617" pan_ulimit="2.617">
          <origin xyz="0 0 1"/>
       </turtlebot_arm>

This will attach a turtlebot arm to your robot. Replace base_link with whatever link you want to attach to, and change the origin as needed. Apart from color, we can configure joints velocity limit and lower/upper limits for the first joint (arm_shoulder_pan) to allow accessing to different operational areas, e.g. left handed vs. right handed robot

## Running MoveIt
Ensure you have all the required dependencies by running (you probably did it before compiling):

       cd turtlebot_arm
       rosdep install --from-paths -i -y src

Before start playing, I highly recommend to calibrate your camera extrinsic parameters following [this tutorial](http://wiki.ros.org/turtlebot_kinect_arm_calibration/Tutorials/CalibratingKinectToTurtleBotArm)

The turtlebot_arm_moveit_config/demo.launch is a bit wonky, but the move_group action underlying it works fine. To test your installation, run one of the demos (well, demo by now) from turtlebot_arm_moveit_demos, e.g.:

       rosrun turtlebot_arm_moveit_demos pick_and_place.py
