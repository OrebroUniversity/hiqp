rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'teleop_point'
type: 'point'
frame_id: 'yumi_body'
visible: true
color: [0.0, 0.0, 1.0, 0.9]
parameters: [0, 0, 0.1]"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'teleop_sphere'
type: 'sphere'
frame_id: 'yumi_body'
visible: true
color: [1.0, 0.0, 0.0, 0.6]
parameters: [0, 0, 0.1, 0.05]"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'teleop_point2'
type: 'point'
frame_id: 'gripper_r_base'
visible: true
color: [0.0, 1.0, 0.0, 0.9]
parameters: [0, 0, 0.1]"