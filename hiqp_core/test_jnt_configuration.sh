rosservice call /amici/hiqp_joint_velocity_controller/set_tasks \
"tasks:
- name: 'task_jnt_limit'
  priority: 1
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefJntLimits', 'shoulder_link', '-2.0', '2.0', '1.6'] #lower/upper joint limit, max joint vel
  dyn_params: ['TDynJntLimits', '20', '10.0']   
- name: 'shoulder_link_jntconfig'
  priority: 2
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefJntConfig', 'shoulder_link', '0.5']
  dyn_params: ['TDynLinear', '2.0', '3.0'] 
- name: 'upper_arm_link_jntconfig'
  priority: 2
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefJntConfig', 'upper_arm_link', '0.0']
  dyn_params: ['TDynLinear', '2.0', '3.0'] 
- name: 'forearm_link_jntconfig'
  priority: 2
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefJntConfig', 'forearm_link', '0.0']
  dyn_params: ['TDynLinear', '2.0', '3.0'] 
- name: 'wrist_1_link_jntconfig'
  priority: 2
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefJntConfig', 'wrist_1_link', '0.0']
  dyn_params: ['TDynLinear', '2.0', '3.0'] 
- name: 'wrist_2_link_jntconfig'
  priority: 2
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefJntConfig', 'wrist_2_link', '0.0']
  dyn_params: ['TDynLinear', '2.0', '1.0'] 
- name: 'wrist_3_link_jntconfig'
  priority: 2
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefJntConfig', 'wrist_3_link', '0.0']
  dyn_params: ['TDynLinear', '2.0', '3.0'] 
#- name: 'full_pose'
#  priority: 3
#  visible: 1
#  active: 1
#  monitored: 1
#  def_params: ['TDefFullPose', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0']
#  dyn_params: ['TDynLinear', '1.0', '3.0'] "
#- name: 'task_jntconfig2'
#  priority: 1
#  visible: 1
#  active: 1
#  monitored: 1
#  def_params: ['TDefJntConfig', 'yumi_link_4_l', '1.0']
#  dyn_params: ['TDynLinear', '1.0', '2.0'] "

