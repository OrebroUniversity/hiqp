#Axis along the y-axis of the gripper, used to align it with the normal to the table plane
rosservice call /yumi/hiqp_joint_velocity_controller/set_primitive \
"name: 'gripper_y-axis'
type: 'line'
frame_id: 'gripper_l_base'
visible: true
color: [0, 1, 0, 1]
parameters: [0, 1, 0, 0, 0, 0.15]"

#Plane along the table
rosservice call /yumi/hiqp_joint_velocity_controller/set_primitive \
"name: 'tablePlane'
type: 'plane'
frame_id: 'yumi_body'
visible: true
color: [0, 0, 1, 1]
parameters: [0, 0, 1, 0]"

#Axis through the center of the cylinder
rosservice call /yumi/hiqp_joint_velocity_controller/set_primitive \
"name: 'grasp_target_axis'
type: 'line'
frame_id: 'yumi_body'
visible: true
color: [0.0, 1.0, 0.0, 1.0]
parameters: [0, 0, 1, 0.45, 0, 0]"

#Axis along the z-axis of the gripper, used to align it ẃith the target object
rosservice call /yumi/hiqp_joint_velocity_controller/set_primitive \
"name: 'gripper_z-axis'
type: 'line'
frame_id: 'gripper_l_base'
visible: true
color: [1, 1, 0, 1]
parameters: [0, 0, 1, 0, 0, 0]"

# Point on the gripper projected onto the cylinder 
rosservice call /yumi/hiqp_joint_velocity_controller/set_primitive \
"name: 'grapsing_point'
type: 'point'
frame_id: 'gripper_l_base'
visible: true
color: [0.0, 0.0, 1.0, 1.0]   
parameters: [0.0, 0.0, 0.15]" 

#Cylinder encircling the object to be grasped
rosservice call /yumi/hiqp_joint_velocity_controller/set_primitive \
"name: 'object_cylinder'
type: 'cylinder'
frame_id: 'yumi_body'
visible: true
color: [0.0, 1.0, 0.0, 0.5]
parameters: [0, 0, 1, 0.45, 0.0, 0, 0.05, 0.2]"

#Plane describing the grasping height 
rosservice call /yumi/hiqp_joint_velocity_controller/set_primitive \
"name: 'grasp_plane'
type: 'plane'
frame_id: 'yumi_body'
visible: true
color: [1.0, 0.0, 0.0, 0.7]
parameters: [0, 0, 1, 0.1]"

# #Sphere encircling the gripper describing the volume of the gripper 
# rosservice call /yumi/hiqp_joint_velocity_controller/set_primitive \
# "name: 'gripper_sphere'
# type: 'sphere'
# frame_id: 'gripper_l_base'
# visible: true
# color: [1.0, 0.0, 0.0, 0.4]   
# parameters: [0.0, 0.0, 0.1, 0.038]" 

sleep 1.0

# #Task forcing the projection of the sphere on the gripper onto the cylinder to not coincide  
# rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
# "name: 'gripper_cylinder_avoidance'
# priority: 1
# visible: 1
# active: 1
# monitored: 1
# def_params: ['TDefGeomProj', 'sphere', 'cylinder', 'gripper_sphere < object_cylinder']
# dyn_params: ['TDynLinear', '10.0']"

#Task projecting the grasping point onto the grasp plane forcing the gripper to be at a specific height defined by the grapsing plane
rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'point_plane_projection'
priority: 1
visible: 1
active: 1
monitored: 1
def_params: ['TDefGeomProj', 'point', 'plane', 'grapsing_point = grasp_plane']
dyn_params: ['TDynLinear', '10']"

#Task aligning the y-axis of the gripper with the center axis of the cylinder
rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'line_on_line_alignment'
priority: 2
visible: 1
active: 1
def_params: ['TDefGeomAlign', 'line', 'line', 'gripper_y-axis = grasp_target_axis', '0.1']
dyn_params: ['TDynLinear', '1.0']"

#Task aligning the z-axis of the gripper onto the normal of the cylinder forcing the gripper to be pointed towards the center of the object
rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'line_on_cylinder_alignment'
priority: 2
visible: 1
active: 1
def_params: ['TDefGeomAlign', 'line', 'cylinder', 'gripper_z-axis = object_cylinder', '0.1']
dyn_params: ['TDynLinear', '1.0']"

# Task projecting the grasping point onto the cylinder forcing the gripper to be close to the object such that it can actually grasp the object
rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'point_on_cylinder_projection'
priority: 3
visible: 1
active: 1
def_params: ['TDefGeomProj', 'point', 'line', 'grapsing_point = grasp_target_axis']
dyn_params: ['TDynLinear', '5.0']"

