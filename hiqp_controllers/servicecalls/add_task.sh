rosservice call /yumi/hiqp_kinematic_controller/addTask \
'task_spec: {task: TaskPoP, behaviour: TaskBehFO, behaviour_parameters: ["1"], priority: 1, visibility: 1, parameters: [gripper_r_base, "0", "0", "1", "0.2"]}'
