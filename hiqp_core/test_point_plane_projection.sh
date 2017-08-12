rosservice call /amici/hiqp_joint_velocity_controller/set_primitives \
"primitives:
- name: 'ee_point'
  type: 'point'
  frame_id: 'wrist_3_link'
  visible: true
  color: [1.0, 0.0, 0.0, 1.0]   
  parameters: [0.0, 0.4, 0.0]
- name: 'target_plane'
  type: 'plane'
  frame_id: 'world'
  visible: true
  color: [1.0, 0.0, 1.0, 0.5]   
  parameters: [0.0, 0.0, 1.0, 2.5]" 

rosservice call /amici/hiqp_joint_velocity_controller/set_tasks \
"tasks:  
- name: 'point_plane_projection'
  priority: 2
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefGeomProj', 'point', 'plane', 'ee_point = target_plane']
  dyn_params: ['TDynLinear', '1.0', '3.0'] "



