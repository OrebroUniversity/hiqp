rosservice call /yumi/hiqp_joint_velocity_controller/set_tasks \
"tasks:  
- name: 'task_jntconfig'
  priority: 2
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefJntConfig', 'yumi_link_1_r', '2.5']
  dyn_params: ['TDynLinear', '2.0', '1.0'] 
- name: 'task_jnt_limit'
  priority: 1
  visible: 1
  active: 1
  monitored: 1
  #def_params: ['TDefJntLimits', 'yumi_link_1_r', '-2.94', '2.94', '1.4835', '0.1'] #lower/upper joint limit, max joint vel, influence zone
  def_params: ['TDefJntLimits', 'yumi_link_1_r', '-2.0', '2.0', '1.4835', '0.1']
  dyn_params: ['TDynJntLimits', '2.0', '20.0', '1.0'] 
#- name: 'full_pose'
#  priority: 3
#  visible: 1
#  active: 1
#  monitored: 1
#  def_params: ['TDefFullPose', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0']
#  dyn_params: ['TDynLinear', '1.0', '2.0'] "
#- name: 'task_jntconfig2'
#  priority: 1
#  visible: 1
#  active: 1
#  monitored: 1
#  def_params: ['TDefJntConfig', 'yumi_link_4_l', '1.0']
#  dyn_params: ['TDynLinear', '1.0', '2.0'] "

