rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'mypoint1'
type: 'point'
frame_id: 'gripper_r_base'
visible: true
color: [1.0, 0.0, 0.0, 0.9]
parameters: ['0', '0', '0.1']"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'mypoint2'
type: 'point'
frame_id: 'gripper_l_base'
visible: true
color: [1.0, 0.0, 0.0, 0.9]
parameters: ['0.2', '0', '0.1']"

# rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
# "name: 'myplane'
# type: 'plane'
# frame_id: 'gripper_l_base'
# visible: true
# color: [1.0, 0.0, 0.0, 0.9]
# parameters: ['1', '0', '0', '-0.2']"
