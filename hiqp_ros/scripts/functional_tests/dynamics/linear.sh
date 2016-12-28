rosservice call /yumi/hiqp_joint_velocity_controller/set_primitive \
"name: 'mypoint3'
type: 'point'
frame_id: 'gripper_l_base'
visible: true
color: [1.0, 0.0, 0.0, 0.9]
parameters: [0.0, 0, 0.3]"

rosservice call /yumi/hiqp_joint_velocity_controller/set_primitive \
"name: 'mypoint4'
type: 'point'
frame_id: 'gripper_r_base'
visible: true
color: [0.0, 1.0, 0.0, 0.9]
parameters: [0.0, 0, 0.3]"

sleep 1.0

rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'task_linear'
priority: 3
visible: 1
active: 1
def_params: ['TDefGeomProj', 'point', 'point', 'mypoint3 = mypoint4']
dyn_params: ['TDynLinear', '1.0']"