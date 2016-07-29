rosservice call /yumi/hiqp_kinematic_controller/addTask \
"name: 'mytask'
type: 'TaskGeometricProjection'
behaviour: ['TaskBehFO', '10']
priority: 1
visibility: 0
parameters: ['point', 'plane', 'mypoint = myplane']"