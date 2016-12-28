rosservice call /yumi/hiqp_joint_velocity_controller/set_primitive \
"name: 'myframe1'
type: 'frame'
frame_id: 'gripper_r_base'
visible: true
color: [0, 0, 0, 1]
parameters: [0, 0, 0.1, 0, 0, 0]"

rosservice call /yumi/hiqp_joint_velocity_controller/set_primitive \
"name: 'myframe2'
type: 'frame'
frame_id: 'world'
visible: true
color: [0, 0, 0, 1]
parameters: [0.5, 0, 0.3, 1.57079, 0, 0]"

sleep 1.0

rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'task_framewithframe'
priority: 3
visible: 1
active: 1
monitored: 1
def_params: ['TDefGeomAlign', 'frame', 'frame', 'myframe1 = myframe2', '0']
dyn_params: ['TDynLinear', '1.0']"