#include <hiqp_ros/hiqp_client.h>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  hiqp_ros::HiQPClient client("","hiqp_controller");

  std::vector<hiqp_msgs::msg::Primitive> primitives;
  hiqp_msgs::msg::Primitive p1, p2;
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

  client.run();
  client.setPrimitives(primitives);
  client.setJointAngles(std::vector<double>(2, -0.5));
  client.quit();

  return 0;
}
