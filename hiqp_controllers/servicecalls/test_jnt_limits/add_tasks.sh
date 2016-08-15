rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'jntlimits1'
type: 'TaskJntLimits'
behaviour: ['DynamicsJntLimits', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1']
priority: 1
visibility: 0
parameters: ['<']"



rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'geomproj1'
type: 'TaskGeometricProjection'
behaviour: ['DynamicsFirstOrder', '10']
priority: 2
visibility: 0
parameters: ['point', 'point', 'mypoint1 = mypoint2']"


