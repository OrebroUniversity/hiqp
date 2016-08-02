#
# Available types and parameter list syntaxes
# point:        [x, y, z]
# line:         [nx, ny, nz,  x,  y,  z,]
# plane:        [x, y, z, nx, ny, nz]
# box:          [x1, y1, z1, x2, y2, z2]
# box:          [x1, y1, z1, x2, y2, z2, nx, ny, nz, angle]
# cylinder:     [nx, ny, nz,  x,  y,  z, radius, height]
# sphere:       [x, y, z, radius]
#




rosservice call /yumi/hiqp_kinematics_controller/addGeomPrim \
"name: 'mycylinder1'
type: 'cylinder'
frame_id: 'yumi_body'
visible: true
color: [0.9, 0.9, 0.0, 0.9]
parameters: ['1', '1', '0', '0.3', '0.3', '0', '0.06', '0.3']"

rosservice call /yumi/hiqp_kinematics_controller/addGeomPrim \
"name: 'mycylinder2'
type: 'cylinder'
frame_id: 'yumi_body'
visible: true
color: [0.9, 0.0, 0.9, 0.9]
parameters: ['1', '0', '1', '0.3', '0', '0.3', '0.04', '0.5']"

rosservice call /yumi/hiqp_kinematics_controller/addGeomPrim \
"name: 'mycylinder3'
type: 'cylinder'
frame_id: 'yumi_body'
visible: true
color: [0.0, 0.9, 0.9, 0.9]
parameters: ['0', '1', '1', '0', '0.3', '0.3', '0.02', '0.6']"






rosservice call /yumi/hiqp_kinematics_controller/addGeomPrim \
"name: 'mycylinder4'
type: 'cylinder'
frame_id: 'yumi_body'
visible: true
color: [0.9, 0, 0.0, 0.9]
parameters: ['-1', '0', '0', '-0.3', '0', '0', '0.06', 'INF']"

rosservice call /yumi/hiqp_kinematics_controller/addGeomPrim \
"name: 'mycylinder5'
type: 'cylinder'
frame_id: 'yumi_body'
visible: true
color: [0.0, 0.9, 0, 0.9]
parameters: ['0', '-1', '0', '0', '-0.3', '0', '0.04', 'Inf']"

rosservice call /yumi/hiqp_kinematics_controller/addGeomPrim \
"name: 'mycylinder6'
type: 'cylinder'
frame_id: 'yumi_body'
visible: true
color: [0, 0.0, 0.9, 0.9]
parameters: ['0', '0', '-1', '0', '0', '-0.3', '0.02', 'inf']"

