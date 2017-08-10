rosservice call /amici/hiqp_joint_velocity_controller/set_primitives \
"primitives:
- name: 'ee_point'
  type: 'point'
  frame_id: 'wrist_3_link'
  visible: true
  color: [1.0, 0.0, 0.0, 0.0]   
  parameters: [0.0, 0.4, 0.0]
- name: 'target_point'
  type: 'point'
  frame_id: 'world'
  visible: true
  color: [0.0, 0.0, 1.0, 1.0]   
  parameters: [0.0, 1.0, 1.0]" 

rosservice call /amici/hiqp_joint_velocity_controller/set_tasks \
"tasks:  
- name: 'point_point_projection'
  priority: 1
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefGeomProj', 'point', 'point', 'ee_point = target_point']
  dyn_params: ['TDynLinear', '1.0', '2.0'] "



