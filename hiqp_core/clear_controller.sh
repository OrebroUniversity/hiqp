#remove all tasks
rosservice call /amici/hiqp_joint_velocity_controller/remove_all_tasks 

#remove all task primitives
rosservice call /amici/hiqp_joint_velocity_controller/remove_all_primitives
