#include <hiqp_ros/hiqp_client.h>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  hiqp_ros::HiQPClient client("","hiqp_controller");

  std::vector<hiqp_msgs::msg::Primitive> primitives;
  hiqp_msgs::msg::Primitive p1, p2, p3, p4, p5;
  p1.name = "tool";
  p1.type = "frame";
  p1.frame_id = "tool_link";
  p1.visible = true;
  p1.color = {0.0, 0.0, 1.0, 1.0};
  p1.parameters = {0.0, 0.0, 0.1, 0.0, 0.0, 0.0};

  p2.name = "target";
  p2.type = "frame";
  p2.frame_id = "base_link";
  p2.visible = true;
  p2.color = {0.0, 0.0, 1.0, 1.0};
  p2.parameters = {-1.5, 0.0, 0.5, 0, 3.1415, 0};

  p3.name = "tool_point";
  p3.type = "point";
  p3.frame_id = "tool_link";
  p3.visible = true;
  p3.color = {1.0, 0.0, 0.0, 1.0};
  p3.parameters = {0.0, 0.0, 0.05};

  p4.name = "target_sphere";
  p4.type = "sphere";
  p4.frame_id = "base_link";
  p4.visible = true;
  p4.color = {0.0, 0.0, 1.0, 1.0};
  p4.parameters = {1.5, 0.2, 0.5, 0.35};
  
  //test nullspace
  p5.name = "second_point";
  p5.type = "point";
  p5.frame_id = "link2";
  p5.visible = true;
  p5.color = {1.0, 0.0, 0.0, 1.0};
  p5.parameters = {0.0, 0.0, 0.0};

  primitives.push_back(p1);
  primitives.push_back(p2);
  primitives.push_back(p3);
  primitives.push_back(p4);
  primitives.push_back(p5);

  client.run();
  client.setPrimitives(primitives);
  std::vector<double> config1 {-1.0, -0.75};
  std::vector<double> config2 {1.0, 0.75};
  client.setJointAngles(config1);

  //test frame-frame alignment
  double tol = 0.001; //error tolerance when task is considered complete
  std::string tname = "frame2frame";
  std::vector<std::string> def_params{"TDefGeomAlign", "frame", "frame", "tool = target", "1.5"};
  bool success = client.setTask(tname, 0, true, true, true,
                 def_params, {"TDynLinear", "0.75"});

  if (success) {
    std::cerr<<"Waiting for completion of task"<<std::endl;
    //char c;
    //std::cin>>c;
    client.waitForCompletion({"frame2frame"}, {hiqp_ros::TaskDoneReaction::REMOVE},
        {tol});
  }
  //move to other side 
  client.setJointAngles(config2,false); //set joint angles and do not remove task when done

  //test point in sphere
  tol = 0.02;
  tname = "point_in_sphere";
  def_params = {"TDefGeomProj", "point", "sphere", "tool_point < target_sphere"};
  success = client.setTask(tname, 1, true, true, true,
                 def_params, {"TDynLinear", "0.95"});

  if (success) {
    std::cerr<<"Waiting for completion of task"<<std::endl;
    client.waitForCompletion({"point_in_sphere"}, {hiqp_ros::TaskDoneReaction::NONE},
        {tol});
  }

  //remove the joint task
  //client.removeTask("joint_configuration");
  

  p4.name = "link2_target1";
  p4.type = "sphere";
  p4.frame_id = "base_link";
  p4.visible = true;
  p4.color = {0.0, 0.5, 0.5, 1.0};
  p4.parameters = {0.5, 0.2, 1.1, 0.2};
  
  p1.name = "link2_target2";
  p1.type = "sphere";
  p1.frame_id = "base_link";
  p1.visible = true;
  p1.color = {0.0, 0.5, 0.5, 1.0};
  p1.parameters = {1.0, 0.2, 1.8, 0.2};

  primitives.clear();
  primitives.push_back(p1);
  primitives.push_back(p4);
  client.setPrimitives(primitives);

  tname = "nullspace_sphere";
  def_params = {"TDefGeomProj", "point", "sphere", "second_point < link2_target1"};
  success = client.setTask(tname, 2, true, true, true,
                 def_params, {"TDynLinear", "0.75"});

  if (success) {
    std::cerr<<"Waiting for completion of task "<<tname<<std::endl;
    client.waitForCompletion({"nullspace_sphere","point_in_sphere"}, {hiqp_ros::TaskDoneReaction::REMOVE, hiqp_ros::TaskDoneReaction::NONE},
        {tol, tol});
  }

  tname = "nullspace_sphere2";
  def_params = {"TDefGeomProj", "point", "sphere", "second_point < link2_target2"};
  success = client.setTask(tname, 2, true, true, true,
                 def_params, {"TDynLinear", "0.75"});

  if (success) {
    std::cerr<<"Waiting for completion of task "<<tname<<std::endl;
    client.waitForCompletion({"nullspace_sphere2","point_in_sphere"}, {hiqp_ros::TaskDoneReaction::REMOVE, hiqp_ros::TaskDoneReaction::NONE},
        {tol, tol});
  }

  //cleanup
  client.removeAllTasks();
  client.removeAllPrimitives(); 
  client.quit();

  return 0;
}
