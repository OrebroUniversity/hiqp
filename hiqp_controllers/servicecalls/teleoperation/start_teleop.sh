rosservice call /wintracker_rebase/synchronize 

sleep 1.0

rosservice call /yumi/hiqp_kinematics_controller/set_task \
"name: 'task_teleop'
priority: 2
visible: 1
active: 1
def_params: ['TDefGeomProj', 'point', 'point', 'teleop_point = teleop_point2']
dyn_params: ['TDynFirstOrder', '10.0']"
