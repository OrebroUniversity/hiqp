rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'jntlimits1'
type: 'TaskJntConfigOne'
behaviour: ['DynamicsFirstOrder', '1.0']
priority: 3
visibility: 0
parameters: ['yumi_link_6_r_joint', '1.0']"