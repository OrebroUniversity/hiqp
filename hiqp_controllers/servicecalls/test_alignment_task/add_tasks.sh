rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'geomalignment1'
type: 'TaskGeometricAlignment'
behaviour: ['DynamicsFirstOrder', '0.5']
priority: 1
visibility: 0
parameters: ['line', 'line', 'myline1 = myline2', '0']"