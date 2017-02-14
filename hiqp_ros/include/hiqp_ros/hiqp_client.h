#include <hiqp_msgs/ActivateTask.h>
#include <hiqp_msgs/DeactivateTask.h>
#include <hiqp_msgs/MonitorTask.h>
#include <hiqp_msgs/RemoveAllPrimitives.h>
#include <hiqp_msgs/RemoveAllTasks.h>
#include <hiqp_msgs/RemovePrimitive.h>
#include <hiqp_msgs/RemoveTask.h>
#include <hiqp_msgs/SetPrimitives.h>
#include <hiqp_msgs/SetTasks.h>
#include <hiqp_msgs/TaskMeasures.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <mutex>
#include <numeric>

namespace hiqp_ros {

enum TaskDoneReaction { NONE = 0, PRINT_INFO = 1, DEACTIVATE = 2, REMOVE = 3 };

class HiQPClient {
  std::string robot_namespace_;
  std::string controller_namespace_;
  /**
   * A nodehandle for the controller namespace.
   *
   */
  ros::NodeHandle nh_;

  ros::NodeHandle robot_nh_;

  /**
   * A client to the set_primitives service.
   */
  ros::ServiceClient set_primitives_client_;

  /**
   * A client to the set_tasks service.
   */
  ros::ServiceClient set_tasks_client_;

  /**
   * A client to the activate_task service.
   */
  ros::ServiceClient activate_task_client_;

  /**
   * A client to the deactivate_task service.
   */
  ros::ServiceClient deactivate_task_client_;

  /**
   * A client to the remove_task service.
   */
  ros::ServiceClient remove_task_client_;

  /**
   * A client to the remove_primitive service.
   */
  ros::ServiceClient remove_primitive_client_;

  ros::ServiceClient remove_all_tasks_client_;

  ros::ServiceClient remove_all_primitives_client_;

  ros::Subscriber task_measures_sub_;

  std::mutex resource_mutex_;

  std::map<std::string, double> task_name_sq_error_map_;

  /**
   * A callback function for monitoring the task.
   *
   * @param task_measures A ConstPtr to message containing a vector of task
   * measures.
   */
  void taskMeasuresCallback(
      const hiqp_msgs::TaskMeasuresConstPtr& task_measures);

 public:
  HiQPClient(const std::string& robot_namespace,
             const std::string& controller_namespace =
                 "hiqp_joint_velocity_controller",
             bool auto_connect = true);

  void connectToServer();

  /**
   * Call the set_primitive service on the hiqp controller.
   *
   * @param name The name of the primitive instance.
   * @param type The type of primitive.
   * @param frame_id The frame_id in which to add the primitive.
   * @param visible Should this be visible in rviz?
   * @param color If visible is set to true, color and alpha values.
   * @param parameters Parameters for the primitive.
   */
  void setPrimitive(const std::string& name, const std::string& type,
                    const std::string& frame_id, bool visible,
                    const std::vector<double>& color,
                    const std::vector<double>& parameters);

  void setPrimitives(const std::vector<hiqp_msgs::Primitive>& primitives);

  /**
   * Call the set_task service on the hiqp controller.
   *
   * @param name The name of the task instance.
   * @param priority Priority level.
   * @param visible Visibility.
   * @param active Initial state of the task.
   * @param monitored Is it monitored.
   * @param def_params Task name and list of strings that are passed to the init
   * fn.
   * @param dyn_params Name of the dynamics and the parameters.
   */
  void setTask(const std::string& name, int16_t priority, bool visible,
               bool active, bool monitored,
               const std::vector<std::string>& def_params,
               const std::vector<std::string>& dyn_params);

  void setTasks(const std::vector<hiqp_msgs::Task>& tasks);

  void deactivateTask(const std::string& task_name);

  void removeTask(const std::string& task_name);

  void removePrimitive(const std::string& primitive_name);

  void removeAllTasks();

  void removeAllPrimitives();

  void resetHiQPController();

  void setJointAngles(const std::vector<double>& joint_angles);

  /**
   * Wait for a group of tasks to be completed, and then unload them all.
   *
   * @param task_names
   */
  void waitForCompletion(const std::vector<std::string>& task_names,
                         const std::vector<TaskDoneReaction>& reactions,
                         const std::vector<double>& error_tol);
};

hiqp_msgs::Task createTaskMsg(const std::string& name, int16_t priority,
                              bool visible, bool active, bool monitored,
                              const std::vector<std::string>& def_params,
                              const std::vector<std::string>& dyn_params);

hiqp_msgs::Primitive createPrimitiveMsg(const std::string& name,
                                        const std::string& type,
                                        const std::string& frame_id,
                                        bool visible,
                                        const std::vector<double>& color,
                                        const std::vector<double>& parameters);
}
