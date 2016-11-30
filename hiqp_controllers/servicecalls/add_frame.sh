rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'myframeprimitive'
type: 'frame'
frame_id: 'gripper_r_base'
visible: true
color: [1.0, 0.0, 0.0, 0.9]
parameters: [0, 0, 0.1, 1, 0, 0, 0]"