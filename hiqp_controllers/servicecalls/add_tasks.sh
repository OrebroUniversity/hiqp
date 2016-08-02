rosservice call /yumi/hiqp_kinematics_controller/addTask \
"name: 'mytask'
type: 'TaskGeometricProjection'
behaviour: ['DynamicsFirstOrder', '10']
priority: 1
visibility: 0
parameters: ['point', 'point', 'mypoint1 = mypoint2']"