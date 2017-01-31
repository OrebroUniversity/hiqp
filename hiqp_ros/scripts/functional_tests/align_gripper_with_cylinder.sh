rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'myline2'
type: 'line'
frame_id: 'gripper_l_base'
visible: true
color: [1, 1, 0, 1]
parameters: [0, 0, 1, 0, 0, 0]"

rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'mycylinder2'
type: 'cylinder'
frame_id: 'yumi_body'
visible: true
color: [0.0, 1.0, 0.0, 1]
parameters: [0, 0, 1, 0.5, 0.0, 0, 0.05, 0.1]"

# rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
# "name: 'upper_cylinder_plane'
# type: 'plane'
# frame_id: 'yumi_body'
# visible: false
# color: [0.0, 1.0, 0.0, 1]
# parameters: [0, 0, 1, 0.1]"

# rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
# "name: 'lower_cylinder_plane'
# type: 'plane'
# frame_id: 'yumi_body'
# visible: false
# color: [0.0, 1.0, 0.0, 1]
# parameters: [0, 0, 1, 0]"



sleep 1.0

rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'line_on_cylinder_projection'
priority: 3
visible: 1
active: 1
def_params: ['TDefGeomAlign', 'line', 'cylinder', 'myline2 = mycylinder2', '0']
dyn_params: ['TDynFirstOrder', '1.0']"

# rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
# "name: 'line_inside_upper_bound'
# priority: 1
# visible: 1
# active: 1
# def_params: ['TDefGeomProj', 'line', 'plane', 'myline2 < upper_cylinder_plane']
# dyn_params: ['TDynFirstOrder', '1.0']"

# rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
# "name: 'line_inside_lower_bound'
# priority: 1
# visible: 1
# active: 1
# def_params: ['TDefGeomProj', 'line', 'plane', 'myline2 > lower_cylinder_plane']
# dyn_params: ['TDynFirstOrder', '1.0']"
