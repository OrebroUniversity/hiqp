rosservice call /amici/hiqp_joint_velocity_controller/set_tasks \
"tasks:
- name: 'full_pose'
  priority: 3
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefFullPose', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0']
  dyn_params: ['TDynLinear', '1.0', '3.0'] "

