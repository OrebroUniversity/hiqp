rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'myline1'
type: 'line'
frame_id: 'gripper_l_base'
visible: true
color: [0, 1, 0, 1]
parameters: [0, 1, 0, 0, 0, 0.15]"

rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'myplane1'
type: 'plane'
frame_id: 'yumi_body'
visible: true
color: [0, 0, 1, 1]
parameters: [0, 0, 1, 0]"

sleep 1.0

rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'line_on_line_projection'
priority: 1
visible: 1
active: 1
def_params: ['TDefGeomAlign', 'line', 'plane', 'myline1 = myplane1', '0']
dyn_params: ['TDynFirstOrder', '1.0']"