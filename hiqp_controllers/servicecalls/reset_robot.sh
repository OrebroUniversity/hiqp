rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'jntconfig1'
type: 'TaskJntConfig'
behaviour: ['DynamicsFirstOrder', '10']
priority: 1
visibility: 0
parameters: []"