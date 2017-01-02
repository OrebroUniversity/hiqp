rosservice call /two_arm_gripper/hiqp_joint_velocity_controller/set_task \
"name: 'task_fullpose'
priority: 3
visible: 1
active: 1
monitored: 1
def_params: ['TDefFullPose', '0.5']
dyn_params: ['TDynLinear', '1']"