rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'object_vertical_axis'
type: 'line'
frame_id: 'yumi_body'
visible: true
color: [1.0, 0.0, 0.0, 0.9]
parameters: [0, 0, 1, 0.35, 0, 0.1]"

rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'gripper_approach_axis'
type: 'line'
frame_id: 'gripper_r_base'
visible: true
color: [0.0, 1.0, 0.0, 0.9]
parameters: [0, 0, 1, 0, 0, 0]"

rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'gripper_vertical_axis'
type: 'line'
frame_id: 'gripper_r_base'
visible: true
color: [0.0, 1.0, 0.0, 0.9]
parameters: [0, -1, 0, 0, 0, 0]"

rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'yumi_x_axis'
type: 'line'
frame_id: 'world'
visible: true
color: [1.0, 0.0, 0.0, 0.9]
parameters: [1, 0, 0, 0, 0, 0]"

rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'align_gripper_approach_axis_yumi_x_axis'
priority: 3
visible: 1
active: 1
def_params: ['TDefGeomAlign', 'line', 'line', 'yumi_x_axis=gripper_approach_axis', '0']
dyn_params: ['TDynFirstOrder', '1.0']"




