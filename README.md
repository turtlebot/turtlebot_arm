TurtleBot Arm
=============

Indigo version of turtlebot arm code. It should easily work on Hydro, too. Package turtlebot_arm_moveit_demos provides use examples to start playing with the arm on MoveIt!, while the recovered on indigo turtlebot_arm_block_manipulation provides a more complete and interesting demo.

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

Before start playing, I highly recommend to calibrate your camera extrinsic parameters following [this tutorial](http://wiki.ros.org/turtlebot_kinect_arm_calibration/Tutorials/CalibratingKinectToTurtleBotArm)

The turtlebot_arm_moveit_config/demo.launch is a bit wonky, but the move_group action underlying it works fine. To test your installation, run one of the demos (well, demo by now) from turtlebot_arm_moveit_demos, e.g.:

       rosrun turtlebot_arm_moveit_demos pick_and_place.py
