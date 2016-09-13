# The HiQP Control Framework
Copyright (C) 2016 Marcus A Johansson

HiQP is an optimal control framework targeted at robotics. It is based on the task function approach.




# Things needed to work with yumi:
- activation/deactivation of controller. default should be deactivated at startup
- activation/deactivation of tasks
- a way to get the current joint values of yumi to load them into simulation for testing






# Currently available features
- A ROS joint velocity controller implementation with full supoprt of the framework
- 6 geometric primitives: point, line, plane, box, cylinder and sphere
- ROS service calls for adding/removing tasks and primitives
- Visualization of tasks and primitives in rViz
- Ability to preload joint limitations, geometric primitives and tasks via .yaml files
- Ability to subscribe to ROS topics and customize the callback function with access to the framework






# Proposals for further development
Add a monitoring tag to addTask service call so that only selected tasks are monitored. This way monitoring output can be made more readable.

Add a dynamics controller.

Add a service call that lists current existing primitives and tasks. (how about a ros node that can interactively list different infos on the status of the controller at runtime?)

The addtask and updatetask task dynamics interfaces are not consistent. Change this so that no special handling of the behaviour parameters is made!
One could also exhange teh add_task and update_task service calls with a set_task service call. Remove would then be unset_task.

Add grouping of tasks and calling activation/monitoring/visibility changes on task groups.

Add a capsule geometric primtive, that is useful for encapsulating robot links to make the optimizer to not produce controls that will result in self-collision and shutdown.







# Known issues
Running the joint limitstion tasks on priority 1, and a whole-body config task on priority 3, results in gurobi failing with message "Unable to find value 'ObjVal'". This does not happen if one runs "rostopic hz /yumi/joint_states". Why is this happening? Is this a threading issue?

Geometric projection for point on box is not working when the box is rotated.

Removing primitives does not work properly.

Running a projection task where both primitives are attached to the same frame, which results in a zero jacobian, throws an error in gurobi.
This can be solved by checking both frames upon task initialization and only add the task is the frames differ.

Minimal jerk task dynamics: the e_dot_star value is correctly calculated, but giving it to the optimizer does not result in a correct task evolution. The controls produced by the controller are too small.





# How to suppress gurobi printouts from casadi

In ``casadi/casadi/interfaces/gurobi/gurobi_interface.cpp`` at line 116, add ``GRBsetintparam(m->env, "OutputFlag", 0);``


