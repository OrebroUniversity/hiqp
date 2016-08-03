# rosservice call /yumi/hiqp_kinematics_controller/add_task \
# "name: 'geomproj1'
# type: 'TaskGeometricProjection'
# behaviour: ['DynamicsFirstOrder', '10']
# priority: 1
# visibility: 0
# parameters: ['point', 'point', 'mypoint1 = mypoint2']"

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'jntconfig1'
type: 'TaskJntConfig'
behaviour: ['DynamicsFirstOrder', '10']
priority: 1
visibility: 0
parameters: []"

# rosservice call /yumi/hiqp_kinematics_controller/add_task \
# "name: 'jntconfig2'
# type: 'TaskJntConfig'
# behaviour: ['DynamicsFirstOrder', '10']
# priority: 1
# visibility: 0
# parameters: ['0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1']"