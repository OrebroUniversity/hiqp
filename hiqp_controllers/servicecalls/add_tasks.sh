rosservice call /yumi/hiqp_kinematic_controller/addTask \
"name: 'mytask'
type: 'TaskGeometricProjection'
behaviour: ['TaskBehFO', '10']
priority: 1
visibility: 1
parameters: ['gripper_r_base', '0', '0', '0.1', 'gripper_l_base', '1', '0', '0', '0.2']"