rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'teleop_wintracker_frame'
type: 'frame'
frame_id: 'yumi_body'
visible: true
color: [0, 0, 0, 1]
parameters: [0, 0, 0, 1, 0, 0, 0]"

# rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
# "name: 'teleop_sphere'
# type: 'sphere'
# frame_id: 'yumi_body'
# visible: true
# color: [1.0, 0.0, 0.0, 0.75]
# parameters: [0, 0, 0.1, 0.03]"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'teleop_gripper_frame'
type: 'frame'
frame_id: 'gripper_r_base'
visible: true
color: [0, 0, 0, 1]
parameters: [0, 0, 0.1, 1, 0, 0, 0]"

sleep 1.0

rosservice call /yumi/hiqp_kinematics_controller/set_task \
"name: 'task_teleop_frame_alignment'
priority: 3
visible: 1
active: 1
def_params: ['TDefGeomAlign', 'frame', 'frame', 'teleop_wintracker_frame = teleop_gripper_frame', '0']
dyn_params: ['TDynFirstOrder', '1.0']"

rosservice call /yumi/hiqp_kinematics_controller/set_task \
"name: 'task_teleop_frame_projection'
priority: 4
visible: 1
active: 1
def_params: ['TDefGeomProj', 'frame', 'frame', 'teleop_wintracker_frame = teleop_gripper_frame']
dyn_params: ['TDynFirstOrder', '1.0']"