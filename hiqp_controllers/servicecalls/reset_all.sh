./remove_all_tasks.sh

sleep .15

./remove_all_primitives.sh

sleep .15

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'jntconfig1'
type: 'TaskJntConfig'
behaviour: ['DynamicsFirstOrder', '10']
priority: 1
visibility: 0
parameters: []"

sleep .15