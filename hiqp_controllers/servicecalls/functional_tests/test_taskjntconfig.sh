rosservice call /yumi/hiqp_kinematics_controller/set_task \
"name: 'mynewtask'
priority: 3
visible: 1
active: 1
def_params: ['TDefJntConfig', 'yumi_link_5_l', '1.0']
dyn_params: ['TDynFirstOrder', '1.0']"