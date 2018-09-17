# The EZ Pick and Place package
The ez_pick_and_place (ez_pnp) package tries to streamline the process of picking and placing objects in a MoveIt! scene. You choose the order of your arm and gripper moves and ez_pnp will make all the necessary MoveIt! calls. Apart from moving and gripping, ez_pnp also helps you add objects to your planning scene as well as attach them to your robot.

# How to use ez_pnp

In the example folder you can find a working illustration of the package (using a Schunk PG70 gripper). Most probably this is not enough to fully understand how the package works, so here is a simple list of things you will have to do in order to use ez_pnp:

1. Initialize an EzPnP object.
2. Initialize GripperMove and ArmMove objects. 
3. Use the addMove function to add GripperMove and ArmMove objects to the EzPnP routine. WARNING: Order matters.
4. Implement a move function that will move your gripper based on a GripperMove.
5. Optionally use the addSphere/addBox/addMesh/AddPlance functions to add objects to your planning scene.
6. Optionally use the attachBox/attachMesh functions to attach objects to your robot.
7. Use the start function to make your robot perform the desired routine.

##

## In detail...
### EzPnP
The `EzPnP` constructor takes the following arguments:

1. The name of the node that will handle the MoveIt! calls. [::String]
2. The maximum planning time of the MoveIt! planners. [::Float64]
3. The maximum planning attempts of the MoveIt! planners. [::Int64]
4. The waiting time after the arm makes a move. [::Float64]
5. The waiting time after the gripper makes a move. [::Float64]
6. The group name of the robot's arm. [::String]
7. The group name of the robot's gripper. [::String]
8. The name of a default position (used to reset in case of a planner failure). [::String]
9. Whether the robot should reset to its default pose if a failure occurs. [::Bool]
10. Whether status messages should be printed [::Bool]

`EzPnP` fields:

1. `node_name`::String (initialized by the user using the constructor)
2. `planning_time`::Float64 (initialized by the user using the constructor)
3. `planning_attempts`::Int32 (initialized by the user using the constructor)
4. `time_between_arm_moves`::Float64 (initialized by the user using the constructor)
5. `time_between_grips`::Float64 (initialized by the user using the constructor)
6. `arm_group_name`::String (initialized by the user using the constructor)
7. `gripper_group_name`::String (initialized by the user using the constructor)
8. `default_position_name`::String (initialized by the user using the constructor)
9. `reset_on_failure`::Bool (initialized by the user using the constructor)
10. `print_status`::Bool (initialized by the user using the constructor)
11. `moves`::Array{EzMove} (empty on initialization, the user defined moves are stored here, using the addMove function)
12. `robot_commander`::PyObject (the MoveIt! RobotCommander Python object. Generally the user should not use this.)
13. `scene_interface`::PyObject (the MoveIt! PlanningSceneInterface Python object. Generally the user should not use this.)
14. `arm_move_group`::PyObject (the MoveIt! MoveGroupCommander Python object. Generally the user should not use this.)
15. `ota`::Array{ObjectToAttach} (empty on initialization, the user defined objects to be attached are stored here, using the attachBox/attachMesh functions)
##
### ArmMove
The `ArmMove` constructor takes the following arguments:

1. The name of the move. Only useful if `print_status` == `true`. [::String]
2. The pose of the robot. [::PoseStamped]

`ArmMove` fields:

1. name::String (initialized by the user using the constructor)
2. pose::PoseStamped (initialized by the user using the constructor)
##
### GripperMove
The `GripperMove` constructor takes the following arguments:

1. The name of the move. Only useful if `print_status` == `true`. [::String]
2. The position of the fingers. [::Float64]
3. The velocity of the move. [::Float64]
4. The acceleration of the move. [::Float64]
5. Whether the move is opening or closing the gripper. [::Bool]

`GripperMove` fields:

1. name::String (initialized by the user using the constructor)
2. position::Float64 (initialized by the user using the constructor)
3. velocity::Float64 (initialized by the user using the constructor)
4. acceleration::Float64 (initialized by the user using the constructor)
5. grip::Bool (initialized by the user using the constructor)

##
# Functions you are expected to use (and how to do so effectively)

##### `addMove`(ep::EzPnP, gm::GripperMove) and `addMove`(ep::EzPnP, am::ArmMove)
The `addMove` functions push a `GripperMove` or `ArmMove` to the `queue` of moves that the robot's arm has to perform.

##### `addSphere`(ep::EzPnP, name::String, pose::PoseStamped, radius::Float64), `addBox`(ep::EzPnP, name::String, pose::PoseStamped, s::Tuple), `addMesh`(ep::EzPnP, name::String, pose::PoseStamped, filename::String, s::Tuple), `addPlane`(ep::EzPnP, name::String, pose::PoseStamped, normal::Tuple, offset::Float64)

The `addSphere`, `addBox`, `addMesh` and `addPlane` functions add objects on the planning scene, just like their Python equivalents. For more info on these functions, consult the MoveIt! API for Python.

##### `attachBox`(ep::EzPnP, link::String, name::String, pose::PoseStamped, s::Tuple, touch_links::Array{String}), `attachMesh`(ep::EzPnP, link::String, name::String, pose::PoseStamped, filename::String, s::Tuple, touch_links::Array{String})

The `attachBox` and `attachMesh` functions are used to inform ez_pnp that the robot will grip an object, and that object will be attached to it. The functions' arguments follow the same rationale as their Python equivalents, but do not immediately attach the object, but add them on a `queue` that will be used only if `grip` == `true` of a `GripperMove` is found. For more info on the arguments of these functions, consult the MoveIt! API for Python.

##### start(ep::EzPnP)

The `start` function signals the end of your control over the node. After the `start` has been called, the `ez_pnp` magic will start. `start` works like this:

```
Get the first move added by the addMove functions.
    If the move is successful:
        If the successful move is a GripperMove:
            If it is a GripperMove and grip == true:
                Attach the first object added by the attachBox/attachMesh functions.
            Elseif it is a GripperMove and grip == false and had already gripping an object:
                Detach the first object added by the attachBox/attachMesh functions, and remove it from the queue.
            Remove all moves from the move queue that lead to the last gripping move.
        If the successful move is an ArmMove:
            Move on to the next move in the queue.
   If the move is unsuccessful and reset_on_failure == true
        Move the arm to the default position, based on default_position_name, and set the next move, back to the start of the queue.
```
###### Note1: Using the `attachBox`/`attachMesh` functions you ONLY inform the routine that you need the object(s) to be attached. You don't immediately attach them, since ez_pnp takes full control of the process after you call `start`. The objects to be attached are added in a `queue` and will automatically be attached one by one after successive "pick" (`grip` == `true`) and "place" (`grip` == `false`) moves. (When `grip` == `true` the first object on the objects to be attached `queue` is attached and when `grip` == `false`, the first object on the objects to be attached `queue` is detached and removed from the `queue`.


###### Note2: It is implied in the above pseudocode, that moves are removed from the `queue` only when a gripping move is successful, in order to ensure that the whole routine is performed successfully. For example, if the arm fails to approach an object while to trying to pick it and resets back to its default position, it will start from the beginning of the whole routine. Contradictory, if the robot is already holding an object and fails during the placing procedure (where `grip` == `false`) the routine will start just after the robot took hold of the object.




