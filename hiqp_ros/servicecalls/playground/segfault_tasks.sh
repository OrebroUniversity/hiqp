rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'point1'
type: 'point'
frame_id: 'yumi_body'
visible: true
color: [0.0, 1.0, 0.0, 0.9]
parameters: [0.5, 0, 0.5]"

rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'line1'
type: 'line'
frame_id: 'gripper_r_base'
visible: true
color: [0.0, 1.0, 0.0, 0.9]
parameters: [0, -1, 0, 0, 0, 0]"

rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'plane1'
type: 'plane'
frame_id: 'gripper_l_base'
visible: true
color: [1.0, 0.0, 0.0, 0.9]
parameters: [0, 0, 1, 0.1]"

rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'segfault_task'
priority: 3
visible: 1
active: 1
def_params: ['TDefGeomProj', 'point', 'line', 'point1 = line1']
dyn_params: ['TDynFirstOrder', '1.0']"

sleep 2.0 

rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'segfault_task'
priority: 3
visible: 1
active: 1
def_params: ['TDefGeomProj', 'point', 'plane', 'point1 = plane1']
dyn_params: ['TDynFirstOrder', '1.0']"