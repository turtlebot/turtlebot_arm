TurtleBot Arm
=============

Indigo version of turtlebot arm code. It should easily work on Hydro, too. Packages turtlebot_block_manipulation and turtlebot_kinect_arm_calibration have been temporally removed, as they are not compatible with Hydro and Indigo. Part of the functionality available there will appear here (hopefully) in the following months. Meanwhile, the turtlebot_arm_moveit_demos provides use examples to start playing with the arm on MoveIt!

## Attaching the Arm to a Robot
Open your xacro-macro-magic URDF, and add something like:

       <include filename="$(find turtlebot_arm_description)/urdf/arm.xacro" />
       <turtlebot_arm parent="base_link" color="white" gripper_color="green"
                 joints_vlimit="1.571" pan_llimit="-2.617" pan_ulimit="2.617">
          <origin xyz="0 0 1"/>
        </turtlebot_arm>

This will attach a turtlebot arm to your robot. Replace base_link with whatever link you want to attach to, and change the origin as needed. Apart from color, we can configure joints velocity limit and lower/upper limits for the first joint (arm_shoulder_pan) to allow accessing to different operational areas, e.g. left handed vs. right handed robot

## Running MoveIt
Ensure you have all the required dependencies by running (you probably did it before compiling):

       cd turtlebot_arm
       rosdep install --from-paths -i -y src

The turtlebot_arm_moveit_config/demo.launch is a bit wonky, but the move_group action underlying it works fine. To test your installation, run one of the demos (well, demo by now) from turtlebot_arm_moveit_demos, e.g.:

       rosrun turtlebot_arm_moveit_demos pick_and_place.py
