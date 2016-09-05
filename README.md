# The HiQP Control Framework
Copyright (C) 2016 Marcus A Johansson

HiQP is an optimal control framework targeted at robotics. It is based on the task function approach.


# Currently available features
- A ROS joint velocity controller implementation with full supoprt of the framework
- 6 geometric primitives: point, line, plane, box, cylinder and sphere
- ROS service calls for adding/removing tasks and primitives
- Visualization of tasks and primitives in rViz
- Ability to preload joint limitations (position and velocity) and geometric primitives via .yaml files



# Proposals for further development
Add a monitoring tag to addTask service call so that only selected tasks are monitored. This way monitoring output can be made more readable.

Add a dynamics controller.

Add a service call that lists current existing primitives and tasks.




# Known issues
Geometric projection for point on box is not working when the box is rotated.

Geometric alignment for line with cylinder is not working, probably the jacobian is miscalculated.

Removing primitives does not work properly.

Running a projection task where both primitives are attached to the same frame, which results in a zero jacobian, throws an error in gurobi.
This can be solved by checking both frames upon task initialization and only add the task is the frames differ.

Minimal jerk task dynamics is not working properly.

Preloading tasks via .yaml craches in segfault.