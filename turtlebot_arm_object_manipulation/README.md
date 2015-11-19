turtlebot_arm_object_manipulation
=================================

This package contains an experimental full overhaul of the turtlebot_arm_block_manipulation demo.
The later worked mostly with custom pieces of code done for demonstrate the turtlebot arm:

 * A pcl-based detector of cubes known size, assuming an horizontal surface at a known height
 * Interactive markers for choosing the targets
 * Pick and place using "move to target pose" move group commands (prior to indigo, it used its own controller, instead of MoveIt!)
 * Basic state machine implemented as a C++ program

On this version, I wanted to combine existing powerful tools to make the demo much more flexible and generalist (and learn them in the process!).
So here we use:

 * ORK tabletop for table segmentation and database stored objects recognition. The table pose don't need to be known in advance, though I still assume it is horizontal
 * The interactive markers for choosing the targets remains pretty the same, though I now identify them with the object name retrieved from the database
 * I use now pick and place move group commands. The planing scene comprehends an octomap and collision objects, so motion plans *should be* collision free
 * More sophisticated control using a SMACH state machine. New features are:
   * User commands are: start, reset, fold [the arm] and quit
   * Fold the arm if no objects are found, to increase camera vision field
   * Clear the octomap if a motion plan fails
   * Shutdown all on quit

See [here](https://drive.google.com/file/d/0BzYjlgbSZJSWaVRVQmVKTVczY00/view?usp=sharing) a demo video.


**WARN:** This is still in alpha status, as most configuration and launch files are still on my own robot's packages [thorp_bringup](https://github.com/corot/thorp/tree/master/thorp_bringup),
 [thorp_obj_rec](https://github.com/corot/thorp/tree/master/thorp_obj_rec) and [thorp_smach](https://github.com/corot/thorp/tree/master/thorp_smach).