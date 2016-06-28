#include <task_manager.h>

#include <iostream>

namespace hiqp {

TaskManager::TaskManager()
{}


TaskManager::~TaskManager() noexcept
{}


bool TaskManager::getKinematicControls
(
     const KDL::Tree& kdl_tree,
	unsigned int n_controls,
	std::vector<double> &controls
)
{
     if (n_controls != controls.size())
     {
          std::cerr << "In TaskManager::getKinematicControls, size of"
               << " controls do not match n_controls. Aborting!\n";
          return false;
     }

     for (auto&& control : controls)
          control = 1;

     return true;
}

} // namespace hiqp


