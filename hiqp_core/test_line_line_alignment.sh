rosservice call /amici/hiqp_joint_velocity_controller/set_primitives \
"primitives:
- name: 'ee_line'
  type: 'line'
  frame_id: 'wrist_3_link'
  visible: true
  color: [1.0, 0.0, 0.0, 1.0]   
  parameters: [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
- name: 'target_line'
  type: 'line'
  frame_id: 'world'
  visible: true
  color: [1.0, 0.0, 1.0, 1.0]   
  parameters: [0.0, 0.0, 1.0, 0.5, 0.5, 0.0]" 

rosservice call /amici/hiqp_joint_velocity_controller/set_tasks \
"tasks:  
- name: 'line_line_alignment'
  priority: 2
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefGeomAlign', 'line', 'line', 'ee_line = target_line', '0.2']
  dyn_params: ['TDynLinear', '1.0', '3.0']"
