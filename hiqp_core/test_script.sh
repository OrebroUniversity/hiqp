rosservice call /yumi/hiqp_joint_velocity_controller/set_tasks \
"tasks:  
- name: 'task_jntconfig'
  priority: 1
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefJntConfig', 'yumi_link_5_l', '1.0']
  dyn_params: ['TDynLinear', '1.0', '2.0'] 
- name: 'full_pose'
  priority: 2
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefFullPose', '0.5', '0.5', '0.5', '0.5', '0.5', '0.5', '0.5', '0.5', '0.5', '0.5', '0.5', '0.5', '0.5', '0.5']
  dyn_params: ['TDynLinear', '1.0', '2.0'] "
#- name: 'task_jntconfig2'
#  priority: 1
#  visible: 1
#  active: 1
#  monitored: 1
#  def_params: ['TDefJntConfig', 'yumi_link_4_l', '1.0']
#  dyn_params: ['TDynLinear', '1.0', '2.0'] "

