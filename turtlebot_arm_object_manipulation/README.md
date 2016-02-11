turtlebot_arm_object_manipulation
=================================

This package contains an experimental full overhaul of the turtlebot_arm_block_manipulation demo.
The later worked mostly with custom pieces of code done for demonstrate the turtlebot arm:

 * A pcl-based detector of cubes known size, assuming an horizontal surface at a known height
 * Interactive markers for choosing the targets
 * Pick and place using "move to target pose" move group commands (prior to indigo, it used its own controller, instead of MoveIt!)
 * Basic state machine implemented as a C++ program

On this version, I wanted to combine existing powerful tools to make the demo much more flexible and generalist (and learn about those tools in the process!).
So here we use:

 * ORK tabletop for table segmentation and database stored objects recognition. The table pose don't need to be known in advance, though I still assume it is horizontal.
   ___Do not forget to add the objects you want to recognize to the database! (see an example below)___
 * The interactive markers for choosing the targets remains pretty the same, though I now identify them with the object name retrieved from the database.
 * I use now pick and place move group commands. The gripper closed position is obtained from the object mesh, asuming it can be retrieved (that's why we start the object_recognition_ros/object_information_server)
 * More sophisticated control using a SMACH state machine. New features are:
   * User can interact with the SMACH, as it is wrapped with an action server. Valid commands are: start, reset, fold [the arm] and quit
   * Fold the arm if no objects are found, to increase camera vision field
   * Clear the octomap if a motion plan fails. *HINT: I removed this because it requires to add 3D sensors to MoveIt!, what is not the default turtlebot_arm configuration.
     You can take the modified SMACH from my own robot packages [thorp_smach](https://github.com/corot/thorp/tree/master/thorp_smach).*
   * Shutdown all on quit

See [here](https://drive.google.com/file/d/0BzYjlgbSZJSWaVRVQmVKTVczY00/view?usp=sharing) a demo video.

##### Collision-free arm trajectories
turtlebot_arm_moveit_config doesn't include 3D sensors on MoveIt!, so you will not see the octomap collision map unless you include it by yourself as explained [here](http://docs.ros.org/indigo/api/pr2_moveit_tutorials/html/planning/src/doc/perception_configuration.html).

##### Add objects to recognize to the database
To add the 2.5 cm side cube that comes on this package, type:

```
rosrun object_recognition_core object_add.py -n cube_2_5 -d "2.5 cm side cube" --commit
rosrun object_recognition_core mesh_add.py YOUR_OBJECT_ID `rospack find turtlebot_arm_object_manipulation`/meshes/cube_2_5.stl --commit
```

### WARNING
This demo requires some code changes still not released:
 * New planning scene interface methods, implemented on this [pending PR](https://github.com/ros-planning/moveit_ros/pull/641). Upon merging, the demo will not compile.
 * Fix [this issue](https://github.com/vanadiumlabs/arbotix_ros/issues/5) with the gripper controller. You can use [my fork](https://github.com/corot/arbotix_ros) meanwhile.
