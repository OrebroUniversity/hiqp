

## Keep the spheres outside of each other!
rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'yumi_joint_limits'
type: 'TaskJntLimits'
behaviour: ['DynamicsJntLimits']
priority: 1
visibility: 0
parameters: ['yumi_link_4_r', '1.3090', '-2.16', '1.4']"



