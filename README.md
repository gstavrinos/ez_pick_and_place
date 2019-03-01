# EZ Pick and Place

<img src="https://raw.githubusercontent.com/gstavrinos/ez_pick_and_place/master/doc/media/ez.gif">

## Why use it

<img src="https://raw.githubusercontent.com/gstavrinos/ez_pick_and_place/master/doc/media/demo.gif">

ez_pick_and_place allows you to pick and place any* object, without the hassle of having to generate grasp poses. Just set up your planning scene, and then name an object and a target position!



<sub>*given that you have a 3D model of it</sub>

## How to use it

What you need to run:

TODO

<!--     * Your arm's driver or simulation: For our UR3 arm the command is 

    * Your gripper's driver or simulation

    * Your arm's+gripper's MoveIt planning execution

    * Your gripper's GraspIt planning service

    * ez_pick_and_place -->

With everything up and running ez_pick_and_place offers two services:

* `EzSceneSetup` is used to setup the MoveIt and GraspIt planning scenes. Populating the EzSceneSetup request:

    * `EzModel[] objects`/`EzModel[] obstacles`: Based on the `EzModel` message, you specify the name of your objects and obstacles, and where the model files for MoveIt and GraspIt can be found. The model files are different for MoveIt and GraspIt. We discuss this matter later in this doc. Finally, you have to specify the pose for each of your objects and obstacles.

    * `EzModel gripper`: Based again on the `EzModel` message, you specify the gripper's name and the GraspIt file that accompanies it. The `move_file` field has no effect here.

    * `string[] finger_joint_names`: Provide a string array containing the names of the gripper's finger joints.

    * `string gripper_frame`: Provide the gripper's base_link.

    * `float64 pose_factor`: Provide the factor difference between MoveIt and GraspIt. This should not be used under normal circumstances, but in case you are using a, exotic version of GraspIt, it is here to adapt to it.

* `EzStartPlanning` is used to request a plan. Populating the EzStartPlanning request:

    * `string graspit_target_object`: Provide the name of the object you previously added in the planning scene.

    * `geometry_msgs/PoseStamped target_place`: The pose of the place location. Keep in mind that in v1.0.0 only the x and y of the position are taken into account. The algorithm assumes that the place surface is at the same height as the pick one, and complete disregards the orientation. If you are interested in how ez_pick_and_place works, reading the following section.

    * `string arm_move_group`: Provide the name of the arm move group.

    * `string gripper_move_group`: Provide the name of the gripper move group.

    * `bool allow_replanning`: Not used (yet).

For further info on how to use the package, you can refer to the `test2_ez_pnp2.py` script under the `test` directory of this repository, which was used to create the the first animation of this doc.


## How it works

So now you know how to use the package, but you are curious on how it works internally. Well, then this is the section for you.

ez_pick_and_place is a package that integrates MoveIt and GraspIt, to provide a much easier experience for the end user. When you set up your planning scene with all the obstacles and objects present, you can then request for an object to be picked and placed on a specific location. 

When you make such request, the algorithm first makes a service call to the GraspIt planning process. After (quite) a while, GraspIt will return a series of candidate grasp poses. 

The ez_pick_and_place algorithm first makes the necessary translations between the MoveIt and GraspIt worlds and then evaluates the translated candidate poses. As soon as a possible solution is found based on the inverse kinematics of a candidate grasp pose, ez_pick_and_place run the `pick()` function, which sends a series of commands to the robot that enables it to pick the requested object. 

With the object picked by the robot, the algorithm starts computing the place pose, based on the location provided by the user. This step ensures that regardless of the arm's joints, the picked object's pitch and roll rotation is preserved. The algorithm generates arm poses that preserve the pitch and roll rotation of the object but not the yaw rotation, assuming that placing the object with the same way as it was picked will result in the object not falling over, since it will rest on its base. Apart from the _gyration_ (modification of the object's yaw), the algorithm experiments with a series of place heights too. Starting from the same height as the pick location and going as high as 5cm from the start one, the algorithm tries to find a place pose that has an inverse kinematics solution for both that height and gyration state. As soon as a solution is found, ez_pick_and_place runs the `place()` function, which sends a series of commands to the robot that enables it to place the object to the specified location.

For more geeky stuff, refer directly to the source code and especially the (lightly) documented `ez_tools.py` inside the `src` directory.

## Preparing your robot for GraspIt and MoveIt
This section is here to redirect you to already awesome documentation regarding MoveIt and GraspIt robot setup.

[Setup your robot for MoveIt integration](https://ros-planning.github.io/moveit_tutorials/doc/setup_assistant/setup_assistant_tutorial.html).


[Setup your robot (just the gripper) for GraspIt integration](https://github.com/JenniferBuehler/graspit-pkgs/wiki/urdf2graspit).


## Creating 3D models for GraspIt and MoveIt

TODO

## Recommendations, reminders and extra info

* In order for GraspIt to provide the algorithm with enough candidate grasp poses, we are using this [configuration file](https://github.com/Roboskel-Manipulation/manos_graspit_config/blob/master/config/graspit_planner_opt.yaml). You can experiment with your own based on your needs, but always refer to the one provided.

* Under normal conditions, do not use the `pose_factor` field of the `EzSceneSetup` service.

* ALWAYS check the robot you have set up for GraspIt using the `graspit_simulator` commands, and then loading a default world with your robot.

* The best way I found to create the contact points was to edit my robot's description file, and not a method based on GUI since all of them seemed to crash on me. Depending on your gripper's complexity, this might be from fairly easy to pretty difficult...

* ALWAYS check your MoveIt objects using `rViz` to ensure that their size is the expected one.

* ALWAYS check your GraspIt objects using `graspit_simulator` to ensure that their size is the expected one.

* Keep in mind the 0.001 factor difference between MoveIt and GraspIt during model creation.

* Also, keep in mind that this factor is taken into account by ez_pick_and_place, so you don't have to worry about it while using it.

* Submit issues, pull requests and have fun!

## Thanks to
Awesome projects that without them, ez_pick_and_place would not be possible:
* [MoveIt](https://moveit.ros.org/): The ultimate ROS-friendly arm (and not only) planning software.
* [GraspIt](https://graspit-simulator.github.io/): The feature-rich grasp pose generator and simulator.
* [graspit-pkgs](https://github.com/JenniferBuehler/graspit-pkgs): The ultimate package for GraspIt/ROS integration.
* [ivcon](https://github.com/ros/ivcon): The small-but-effective tool to convert your 3D models to all the required formats for MoveIt, GraspIt and beyond.
