#
# Available types and parameter list syntaxes
# point:        [x, y, z]
# line:         [nx, ny, nz,  x,  y,  z, l]
# line:         [x1, y1, z1, x2, y2, z2]
# plane:        [x, y, z, nx, ny, nz]
# box:          [x1, y1, z1, x2, y2, z2]
# box:          [x1, y1, z1, x2, y2, z2, nx, ny, nz, angle]
# cylinder:     [nx, ny, nz,  x,  y,  z, height, radius]
# cylinder:     [x1, y1, z1, x2, y2, z2, radius]
# sphere:       [x, y, z, radius]
#

# rosservice call /yumi/hiqp_kinematic_controller/addGeomPrim \
# "name: 'myline1'
# type: 'line'
# frame_id: 'yumi_body'
# visible: true
# color: [0.9, 0.0, 0.0, 0.9]
# parameters: ['1', '0', '0', '1.2', '0.1', '0', '2.4']"

# rosservice call /yumi/hiqp_kinematic_controller/addGeomPrim \
# "name: 'myline2'
# type: 'line'
# frame_id: 'yumi_body'
# visible: true
# color: [0.0, 0.9, 0.0, 0.9]
# parameters: ['0', '1', '0', '0', '1.2', '0.1', '2.4']"

# rosservice call /yumi/hiqp_kinematic_controller/addGeomPrim \
# "name: 'myline3'
# type: 'line'
# frame_id: 'yumi_body'
# visible: true
# color: [0.0, 0.0, 0.9, 0.9]
# parameters: ['0', '0', '1', '0.1', '0', '1.2', '2.4']"




rosservice call /yumi/hiqp_kinematic_controller/addGeomPrim \
"name: 'myline4'
type: 'line'
frame_id: 'yumi_body'
visible: true
color: [0.9, 0.9, 0.0, 0.9]
parameters: ['0.1', '0.1', '0', '0.5', '0.5', '0']"

rosservice call /yumi/hiqp_kinematic_controller/addGeomPrim \
"name: 'myline5'
type: 'line'
frame_id: 'yumi_body'
visible: true
color: [0.0, 0.9, 0.9, 0.9]
parameters: ['0', '0.1', '0.1', '0', '0.5', '0.5']"

rosservice call /yumi/hiqp_kinematic_controller/addGeomPrim \
"name: 'myline6'
type: 'line'
frame_id: 'yumi_body'
visible: true
color: [0.9, 0.0, 0.9, 0.9]
parameters: ['0.1', '0', '0.1', '0.5', '0', '0.5']"





# rosservice call /yumi/hiqp_kinematic_controller/addGeomPrim \
# "name: 'myline7'
# type: 'line'
# frame_id: 'yumi_body'
# visible: true
# color: [0.9, 0.9, 0.9, 0.9]
# parameters: ['0.4', '0.3', '0.2', '-0.1', '-0.2', '-0.3']"