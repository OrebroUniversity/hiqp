rosservice call /yumi/hiqp_joint_velocity_controller/set_primitive \
"name: 'myline2'
type: 'line'
frame_id: 'gripper_l_base'
visible: true
color: [1, 1, 0, 1]
parameters: [0, 0, 1, 0, 0, 0]"

rosservice call /yumi/hiqp_joint_velocity_controller/set_primitive \
"name: 'mycylinder2'
type: 'cylinder'
frame_id: 'yumi_body'
visible: true
color: [0.0, 1.0, 0.0, 1]
parameters: [0, 0, 1, 0.5, 0.0, 0, 0.05, 0.1]"

rosservice call /yumi/hiqp_joint_velocity_controller/set_primitive \
"name: 'upper_cylinder_plane'
type: 'plane'
frame_id: 'yumi_body'
visible: false
color: [1.0, 0.0, 0.0, 1]
parameters: [0, 0, 1, 0.1]"

rosservice call /yumi/hiqp_joint_velocity_controller/set_primitive \
"name: 'grasp_plane'
type: 'plane'
frame_id: 'yumi_body'
visible: false
color: [0.0, 1.0, 0.0, 1]
parameters: [0, 0, 1, 0.5]"

#Add a point geometric primitive attached to the base frame of the right gripper. Color for visualization is parametrized by rgb and alpha value, the point x/y/z coordinates are expressed in 'frame_id'
rosservice call /yumi/hiqp_joint_velocity_controller/set_primitive \
"name: 'test_point'
type: 'point'
frame_id: 'gripper_l_base'
visible: true
color: [0.0, 0.0, 1.0, 1.0]   
parameters: [0.0, 0.0, 0.15]" 



#Add a sphere geometric primitive attached to the base frame of the right gripper. The sphere is parametrized by center point coordinates x/y/z and radius
# rosservice call /yumi/hiqp_joint_velocity_controller/set_primitive \
# "name: 'avoidance_sphere'
# type: 'sphere'
# frame_id: 'gripper_l_base'
# visible: true
# color: [1.0, 0.0, 0.0, 0.4]   
# parameters: [0.0, 0.0, 0.1, 0.4]" 


sleep 1.0

rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'line_on_cylinder_projection'
priority: 3
visible: 1
active: 1
def_params: ['TDefGeomAlign', 'line', 'cylinder', 'myline2 = mycylinder2', '0']
dyn_params: ['TDynLinear', '1.0']"

#Add a task constraining the existing primitive test_point to target_plane, the point will converge onto the plane with a normal velocity component governed by the linear first-order dynamics v_n=-1.5*d, where d is the normal distance of target_point to the plane. The task runs on a lower priority level as the avoidance task and thus will only be accomplished as good as possible in the null-space of the higher-ranked task.
rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'point_plane_projection'
priority: 2
visible: 1
active: 1
monitored: 1
def_params: ['TDefGeomProj', 'point', 'plane', 'test_point < upper_cylinder_plane']
dyn_params: ['TDynLinear', '1']"

#Add a task constraining the existing primitive test_point to target_plane, the point will converge onto the plane with a normal velocity component governed by the linear first-order dynamics v_n=-1.5*d, where d is the normal distance of target_point to the plane. The task runs on a lower priority level as the avoidance task and thus will only be accomplished as good as possible in the null-space of the higher-ranked task.
rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'point_plane_projection2'
priority: 2
visible: 1
active: 1
monitored: 1
def_params: ['TDefGeomProj', 'point', 'plane', 'test_point > lower_cylinder_plane']
dyn_params: ['TDynLinear', '1']"

#Add an avoidance task forcing the avoidance sphere to stay in the positive half-space of the avoidance plane
rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'sphere_cylinder_avoidance'
priority: 1
visible: 1
active: 1
monitored: 1
def_params: ['TDefGeomProj', 'sphere', 'cylinder', 'avoidance_sphere > mycylinder2']
dyn_params: ['TDynLinear', '10.0']"
