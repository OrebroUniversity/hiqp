# rosservice call /yumi/hiqp_kinematics_controller/add_task \
# "name: 'jntconfig1'
# type: 'TaskJntConfig'
# behaviour: ['DynamicsFirstOrder', '10']
# priority: 1
# visibility: 0
# parameters: ['0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1']"

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'jntconfig1'
type: 'TaskJntConfig'
behaviour: ['DynamicsMinimalJerk', '5.0']
priority: 1
visibility: 0
parameters: ['0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1']"

# rosservice call /yumi/hiqp_kinematics_controller/add_task \
# "name: 'jntconfig2'
# type: 'TaskJntConfig'
# behaviour: ['DynamicsFirstOrder', '1']
# priority: 1
# visibility: 0
# parameters: []"