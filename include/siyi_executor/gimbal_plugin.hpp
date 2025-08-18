#pragma once

#include <mrs_lib/subscribe_handler.h>

#include <siyi_cam_driver/GimbalControl.h>
#include <siyi_cam_driver/GimbalState.h>

#include "iroc_mission_handler/subtask_executor_interface.h"

namespace siyi_executor {

/**
 * \brief Executor for Siyi gimbal control.
 *
 * This executor handles the initialization, execution, and completion of gimbal subtasks.
 *
 * Parameters format: [<yaw>, <pitch>] in degrees.
 * Example: "[-30.0, 15.0]"
 *
 */
class SiyiGimbalExecutor : public iroc_mission_handler::SubtaskExecutor {
public:
  SiyiGimbalExecutor()          = default;
  virtual ~SiyiGimbalExecutor() = default;

  bool stop() override;

protected:
  bool initializeImpl(ros::NodeHandle& nh, const std::string& parameters) override;
  bool startImpl() override;
  bool checkCompletion(double& progress) override;

private:
  double _speed_tolerance_ = 0.01;
  double progress_         = 0.0;
  bool has_started_        = false;
  bool is_stopped_         = false;

  std::vector<double> initial_angles_; // [pitch, yaw]
  std::vector<double> goal_angles_;    // [pitch, yaw]

  ros::ServiceClient sc_set_gimbal_control_;
  mrs_lib::SubscribeHandler<siyi_cam_driver::GimbalState> sh_current_state_;

  /**
   * \brief Callback for current gimbal state updates.
   * \param msg Current gimbal state message.
   */
  void stateCallback(const siyi_cam_driver::GimbalState::ConstPtr msg);
};

} // namespace siyi_executor

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(siyi_executor::SiyiGimbalExecutor, iroc_mission_handler::SubtaskExecutor)