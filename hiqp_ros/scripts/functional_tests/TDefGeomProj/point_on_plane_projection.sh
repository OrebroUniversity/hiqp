rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'mypoint1'
type: 'point'
frame_id: 'gripper_l_base'
visible: true
color: [0.0, 0.0, 1.0, 0.9]
parameters: [0.0, 0, 0.1]"

rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'mypoint2'
type: 'point'
frame_id: 'world'
visible: true
color: [0.0, 1.0, 0.0, 0.9]
parameters: [0.5, 0.05, 0]"

sleep 1.0

rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'task_test_taskjntconfig'
priority: 3
visible: 1
active: 1
def_params: ['TDefGeomProj', 'point', 'point', 'mypoint1 = mypoint2']
dyn_params: ['TDynFirstOrder', '1.0']"