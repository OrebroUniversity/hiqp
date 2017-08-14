# The HiQP Control Framework
Copyright (C) 2016-2017 Marcus A. Johansson

HiQP is a whole-body control framework for computing inverse kinematics (and dynamics in future releases) involving multiple objectives which can be incompatible. It uses the inequality task function approach in [1] and is based on hierarchical least-squares optimization. Lower-ranked tasks are fulfilled as good as possible (in the least-square sense) in the null-space of higher-ranked tasks. More details are given in [2] - available at http://www.diva-portal.org/smash/get/diva2:1056500/FULLTEXT01.pdf. 

## Acknowledgements
Robert Krug and Todor Stoyanov at the AASS Research Institute at Örebro University, Sweden, have been important contributors to this project. Thank you.

## Upcoming features/changes
- A Wiki/user guide will be created as a tutorial on how to use HiQP with ROS. In the meantime, we refer to the hiqp_demos package for examples of how to use HiQP: git@github.com:OrebroUniversity/hiqp_demos.git
- The list of available service calls will be revised and extended.
- A capsule primitive will be added to the set of geometric primitives.
- The framework will be extended to compute controls on a joint acceleration level in order to enable inverse dynamics control. 

## Installation note

HiQP relies on Gurobi to solve Quadratic Programs (QPs). For academic use, a free license can be obtained at http://www.gurobi.com/. Make sure to set the corresponding environment variables as described in http://www.gurobi.com/documentation/6.5/quickstart_linux/software_installation_guid.html. 

## How to cite HiQP

A publication presenting HiQP is in preparation. In the meantime, please refer to [2].

</br>
[1] ... O. Kanoun, F. Lamiraux and P.-B. Wieber, Kinematic control of redundant manipulators: Generalizing the task-priority framework to inequality task. IEEE T-RO, 27(4):785-792, 2011.
</br>
[2] ... M. A. Johansson, Online whole-body control using hierarchical quadratic programming: implementation and evaluation of the HiQP control framework. MSc Thesis, 2016.

## TODO

* Rename TaskDynamics to TaskControllers to better reflect their purpose
   -> rename TDynLinear to PDController
   -> rename Task::getDynamics() to Task::getControls() or Task::getTaskSpaceControls()
   -> rename Task::getValue() to Task::getError()
   -> rename Task::getValueDerivative() to Task::getErrorDerivative()
* Renamed TaskManager::getVelocityControls(...) to TaskManager::getAccelerationControls(...) 
* Reworked the TaskMeasure class and corresponding message: contains now the task function value (de), task function derivative (dde) and the desired task acceleration dde as computed by the task space control law (TaskDynamics)
* Need to rework Task::checkConsistency(RobotStatePtr robot_state)
* To control in acceleration, the TaskDynamics have to be a function of e & de
* TaskDynamics::update(...) should not need the Jacobian/Jacobian derivative as parameter, as the dynamics (controllers) live in task space
* What's tasks::TDefMetaTask?
* Renamed TaskDefinition::getInitialValue() to TaskDefinition::getInitialTaskValue()
* Both TaskDefinition and TaskDynamics have a member variable performance_measures_ which seems nowhere to be used
* Should rename e_ddot to dde and e_dot to de everywhere for consistency
* Remove ROS-specific stuff from hiqp_core (error printouts ...)
* Disabled measured sampling time update in the controler base class as it proved to be fragile (could yield 0.0)
* kNamespace in the ROS visualizer is hardcoded to "/yumi" - should be loaded from config/launch file
* changed getVelocityJacobianForTwoPoints(...) to changeJacReferencePoint(...) and modified it to return the full matrix 
* Should implement Jacobian derivative computation with the official KDL implementation once its added to the ROS package in order to increase efficiency opposed to the current naive impementation which necessiates to compute n_Joints Jacobians at each time step
* Should implement relativeJacobianDerivative computation using the previously calculated Jacobians rather than in a separate loop

