#
# Available types and parameter list syntaxes
# point:        [x, y, z]
# line:         [x,   y,  z, nx, ny, nz, l]
# line:         [x1, y1, z1, x2, y2, z2]
# plane:        [x, y, z, nx, ny, nz]
# box:          [x1, y1, z1, x2, y2, z2]
# box:          [x1, y1, z1, x2, y2, z2, nx, ny, nz, angle]
# cylinder:     [nx, ny, nz,  x,  y,  z, height, radius]
# cylinder:     [x1, y1, z1, x2, y2, z2, radius]
# sphere:       [x, y, z, radius]
#

rosservice call /yumi/hiqp_kinematic_controller/addGeomPrim \
"name: 'myline'
type: 'line'
frame_id: 'gripper_r_base'
visible: true
color: [0.9, 0.5, 0.9, 0.9]
parameters: ['0', '0', '0', '0', '0', '1', 'Inf']"

# rosservice call /yumi/hiqp_kinematic_controller/addGeomPrim \
# "name: 'myplane'
# type: 'plane'
# frame_id: 'gripper_r_base'
# visible: true
# color: [0.9, 0.9, 0.5, 0.9]
# parameters: ['1', '0', '0', '-0.2']"

# rosservice call /yumi/hiqp_kinematic_controller/addGeomPrim \
# "name: 'mybox'
# type: 'box'
# frame_id: 'gripper_r_base'
# visible: true
# color: [0.9, 0.9, 0.9, 0.9]
# parameters: ['1', '0', '0', '0', '0', '0']"