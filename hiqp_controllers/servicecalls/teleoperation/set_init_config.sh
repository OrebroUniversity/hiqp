rosservice call /yumi/hiqp_kinematics_controller/remove_task \
"task_name: 'task_jntconfig_r1'"
rosservice call /yumi/hiqp_kinematics_controller/remove_task \
"task_name: 'task_jntconfig_r2'"
rosservice call /yumi/hiqp_kinematics_controller/remove_task \
"task_name: 'task_jntconfig_r3'"
rosservice call /yumi/hiqp_kinematics_controller/remove_task \
"task_name: 'task_jntconfig_r4'"
rosservice call /yumi/hiqp_kinematics_controller/remove_task \
"task_name: 'task_jntconfig_r5'"
rosservice call /yumi/hiqp_kinematics_controller/remove_task \
"task_name: 'task_jntconfig_r6'"
rosservice call /yumi/hiqp_kinematics_controller/remove_task \
"task_name: 'task_jntconfig_r7'"
rosservice call /yumi/hiqp_kinematics_controller/remove_task \
"task_name: 'task_jntconfig_l1'"
rosservice call /yumi/hiqp_kinematics_controller/remove_task \
"task_name: 'task_jntconfig_l2'"
rosservice call /yumi/hiqp_kinematics_controller/remove_task \
"task_name: 'task_jntconfig_l3'"
rosservice call /yumi/hiqp_kinematics_controller/remove_task \
"task_name: 'task_jntconfig_l4'"
rosservice call /yumi/hiqp_kinematics_controller/remove_task \
"task_name: 'task_jntconfig_l5'"
rosservice call /yumi/hiqp_kinematics_controller/remove_task \
"task_name: 'task_jntconfig_l6'"
rosservice call /yumi/hiqp_kinematics_controller/remove_task \
"task_name: 'task_jntconfig_l7'"

rosservice call /yumi/hiqp_kinematics_controller/set_task \
"name: 'task_jntconfig_r1'
priority: 3
visible: 1
active: 1
def_params: ['TDefJntConfig', 'yumi_link_1_r', '0.25']
dyn_params: ['TDynFirstOrder', '1.0']"

rosservice call /yumi/hiqp_kinematics_controller/set_task \
"name: 'task_jntconfig_r2'
priority: 3
visible: 1
active: 1
def_params: ['TDefJntConfig', 'yumi_link_2_r', '-0.93']
dyn_params: ['TDynFirstOrder', '1.0']"

rosservice call /yumi/hiqp_kinematics_controller/set_task \
"name: 'task_jntconfig_r3'
priority: 3
visible: 1
active: 1
def_params: ['TDefJntConfig', 'yumi_link_3_r', '-0.93']
dyn_params: ['TDynFirstOrder', '1.0']"

rosservice call /yumi/hiqp_kinematics_controller/set_task \
"name: 'task_jntconfig_r4'
priority: 3
visible: 1
active: 1
def_params: ['TDefJntConfig', 'yumi_link_4_r', '-0.11']
dyn_params: ['TDynFirstOrder', '1.0']"

rosservice call /yumi/hiqp_kinematics_controller/set_task \
"name: 'task_jntconfig_r5'
priority: 3
visible: 1
active: 1
def_params: ['TDefJntConfig', 'yumi_link_5_r', '3.3']
dyn_params: ['TDynFirstOrder', '1.0']"

rosservice call /yumi/hiqp_kinematics_controller/set_task \
"name: 'task_jntconfig_r6'
priority: 3
visible: 1
active: 1
def_params: ['TDefJntConfig', 'yumi_link_6_r', '-0.85']
dyn_params: ['TDynFirstOrder', '1.0']"

rosservice call /yumi/hiqp_kinematics_controller/set_task \
"name: 'task_jntconfig_r7'
priority: 3
visible: 1
active: 1
def_params: ['TDefJntConfig', 'yumi_link_7_r', '0.0']
dyn_params: ['TDynFirstOrder', '1.0']"

rosservice call /yumi/hiqp_kinematics_controller/set_task \
"name: 'task_jntconfig_l1'
priority: 3
visible: 1
active: 1
def_params: ['TDefJntConfig', 'yumi_link_1_l', '-1.8']
dyn_params: ['TDynFirstOrder', '1.0']"

rosservice call /yumi/hiqp_kinematics_controller/set_task \
"name: 'task_jntconfig_l2'
priority: 3
visible: 1
active: 1
def_params: ['TDefJntConfig', 'yumi_link_2_l', '0.26']
dyn_params: ['TDynFirstOrder', '1.0']"

rosservice call /yumi/hiqp_kinematics_controller/set_task \
"name: 'task_jntconfig_l3'
priority: 3
visible: 1
active: 1
def_params: ['TDefJntConfig', 'yumi_link_3_l', '-1.15']
dyn_params: ['TDynFirstOrder', '1.0']"

rosservice call /yumi/hiqp_kinematics_controller/set_task \
"name: 'task_jntconfig_l4'
priority: 3
visible: 1
active: 1
def_params: ['TDefJntConfig', 'yumi_link_4_l', '0.72']
dyn_params: ['TDynFirstOrder', '1.0']"

rosservice call /yumi/hiqp_kinematics_controller/set_task \
"name: 'task_jntconfig_l5'
priority: 3
visible: 1
active: 1
def_params: ['TDefJntConfig', 'yumi_link_5_l', '0.0']
dyn_params: ['TDynFirstOrder', '1.0']"

rosservice call /yumi/hiqp_kinematics_controller/set_task \
"name: 'task_jntconfig_l6'
priority: 3
visible: 1
active: 1
def_params: ['TDefJntConfig', 'yumi_link_6_l', '0.0']
dyn_params: ['TDynFirstOrder', '1.0']"

rosservice call /yumi/hiqp_kinematics_controller/set_task \
"name: 'task_jntconfig_l7'
priority: 3
visible: 1
active: 1
def_params: ['TDefJntConfig', 'yumi_link_7_l', '0.0']
dyn_params: ['TDynFirstOrder', '1.0']"

