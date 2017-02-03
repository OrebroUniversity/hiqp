#include <hiqp_ros/hiqp_client.h>

int main(int argn, char* args[]) {
  ros::init(argn, args, "test_frameonframe");

  hiqp_ros::HiQPClient client("yumi/hiqp_joint_velocity_controller");

  std::vector<hiqp_msgs::Primitive> primitives;
  hiqp_msgs::Primitive p1, p2;
  p1.name = "r_gripper";
  p1.type = "frame";
  p1.frame_id = "gripper_l_base";
  p1.visible = true;
  p1.color = {0.0, 0.0, 1.0, 1.0};
  p1.parameters = {0.0, 0.0, 0.1, 0.0, 0.0, 0.0};

  p2.name = "target";
  p2.type = "frame";
  p2.frame_id = "world";
  p2.visible = true;
  p2.color = {0.0, 0.0, 1.0, 1.0};
  p2.parameters = {0.298, -0.051, 0.231, 2.645, -0.079, 0.144};

  primitives.push_back(p1);
  primitives.push_back(p2);

  client.setPrimitives(primitives);
  client.setTask("frameonframe_alignment", 2, true, true, true,
                 {"TDefGeomAlign", "frame", "frame", "r_gripper = target", "0"},
                 {"TDynLinear", "1.0"});
  client.setTask("frameonframe_projection", 2, true, true, true,
                 {"TDefGeomProj", "frame", "frame", "r_gripper = target"},
                 {"TDynLinear", "1.0"});

  return 0;
}
