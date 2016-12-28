rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'task_jntconfig'
priority: 3
visible: 1
active: 1
monitored: 1
def_params: ['TDefJntConfig', 'yumi_link_5_l', '-1.0']
dyn_params: ['TDynLinear', '1.0']"
