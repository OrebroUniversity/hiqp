rosservice call /yumi/hiqp_joint_velocity_controller/set_primitive \
"name: 'mypoint'
type: 'point'
frame_id: 'gripper_l_base'
visible: true
color: [1.0, 0.0, 1.0, 0.9]
parameters: [0.0, 0, 0.2]"

rosservice call /yumi/hiqp_joint_velocity_controller/set_primitive \
"name: 'mybox'
type: 'box'
frame_id: 'yumi_body'
visible: true
color: [0.0, 1.0, 0.0, 0.9]
parameters: [0.6, 0, 0.1, 0.1, 0.1, 0.1, 0.77, 0.5, 0.1]"

sleep 1.0

rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'task_pointonbox'
priority: 3
visible: 1
active: 1
monitored: 1
def_params: ['TDefGeomProj', 'point', 'box', 'mypoint = mybox']
dyn_params: ['TDynLinear', '1.0']"