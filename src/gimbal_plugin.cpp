#include "siyi_executor/gimbal_plugin.hpp"

namespace siyi_executor {

bool SiyiGimbalExecutor::initializeImpl(ros::NodeHandle& nh, const std::string& parameters) {
  mrs_lib::ParamLoader param_loader(nh, "SiyiGimbalExecutor");
  param_loader.addYamlFileFromParam("executor_config");

  // Load custom configuration if provided
  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path);
  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.setPrefix("mission_handler/subtask_executors/siyi_camera");
  // param_loader.loadParam("speed_tolerance", _speed_tolerance);

  // Parse parameters
  if (!parseParams(parameters, goal_angles_)) {
    ROS_ERROR_STREAM("[SiyiGimbalExecutor]: Failed to parse parameters: " << parameters);
    return false;
  }

  // Validate gimbal parameters
  if (goal_angles_.size() == 1) {
    goal_angles_.push_back(0.0); // Default pitch if only yaw is provided
  } else if (goal_angles_.size() != 2) {
    ROS_ERROR("[SiyiGimbalExecutor]: Invalid parameters format, expected [<yaw>, <pitch>]");
    return false;
  }

  // Initialize subscriber
  mrs_lib::SubscribeHandlerOptions sh_opts;
  sh_opts.nh = nh;
  sh_opts.node_name = "MissionHandler";
  sh_opts.no_message_timeout = ros::Duration(5.0);
  sh_opts.threadsafe = true;
  sh_opts.autostart = false;
  sh_opts.queue_size = 10;
  sh_opts.transport_hints = ros::TransportHints().tcpNoDelay();

  sh_current_state_ = mrs_lib::SubscribeHandler<siyi_cam_driver::GimbalState>(sh_opts, "/groundstation/gimbal/state", // Remapped
                                                                              &SiyiGimbalExecutor::stateCallback, this);

  // Initialize service client for gimbal control
  sc_set_gimbal_control_ = nh.serviceClient<siyi_cam_driver::GimbalControl>("/groundstation/gimbal/control");

  ROS_DEBUG_STREAM("[SiyiGimbalExecutor]: Initialized with yaw: " << goal_angles_[0] << ", pitch: " << goal_angles_[1]);
  return true;
}

bool SiyiGimbalExecutor::startImpl() {
  // Create and send gimbal control command
  siyi_cam_driver::GimbalControl srv;
  srv.request.yaw = goal_angles_[0];
  srv.request.pitch = goal_angles_[1];

  if (!sc_set_gimbal_control_.call(srv)) {
    ROS_ERROR("[SiyiGimbalExecutor]: Failed to call gimbal control service");
    return false;
  }
  if (!srv.response.success) {
    ROS_ERROR_STREAM("[SiyiGimbalExecutor]: Gimbal control service failed: " << srv.response.message);
    return false;
  }

  ROS_INFO_STREAM("[SiyiGimbalExecutor]: Gimbal control started with yaw: " << goal_angles_[0] << ", pitch: " << goal_angles_[1]);
  return true;
}

bool SiyiGimbalExecutor::checkCompletion(double& progress) {
  progress = progress_;
  return is_stopped_;
}

bool SiyiGimbalExecutor::stop() {
  // It is not possible to stop the gimbal once it has been set, so we just log this
  ROS_INFO("[SiyiGimbalExecutor]: Stopping gimbal executor, but gimbal control cannot be stopped once set.");
  has_started_ = false;
  progress_ = 0.0;
  is_stopped_ = false;
  goal_angles_.clear();
  sc_set_gimbal_control_.shutdown(); // Shutdown the service client
  sh_current_state_.stop();          // Stop receiving updates

  return true;
}

void SiyiGimbalExecutor::stateCallback(const siyi_cam_driver::GimbalState::ConstPtr msg) {
  if (!has_started_) {
    initial_angles_ = {msg->yaw, msg->pitch}; // Store initial angles on first state update
    has_started_ = true;
    return; // Return after storing initial angles to prevent immediate stopping
  }

  // If gimbal is not moving, then it has reached the goal or a limit position
  if (std::abs(msg->yaw_speed) < _speed_tolerance && std::abs(msg->pitch_speed) < _speed_tolerance) {
    ROS_INFO("[SiyiGimbalExecutor]: Gimbal state matches has reached the goal angles.");
    is_stopped_ = true;
    sh_current_state_.stop(); // Stop receiving updates
  } else {
    ROS_DEBUG("[SiyiGimbalExecutor]: Current gimbal state - Yaw: %f, Pitch: %f", msg->yaw_speed, msg->pitch_speed);
  }

  // Progress computation
  double roll_den = std::abs(initial_angles_[0] - goal_angles_[0]) < 1e-6 ? 1.0 : std::abs(initial_angles_[0] - goal_angles_[0]);
  double pitch_den = std::abs(initial_angles_[1] - goal_angles_[1]) < 1e-6 ? 1.0 : std::abs(initial_angles_[1] - goal_angles_[1]);

  progress_ = 1.0 - (std::abs(msg->roll - goal_angles_[0]) / roll_den + std::abs(msg->pitch - goal_angles_[1]) / pitch_den) / 2.0;
}
} // namespace siyi_executor