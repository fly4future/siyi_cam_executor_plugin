#pragma once

#include <mrs_lib/subscribe_handler.h>

#include <siyi_cam_driver/Timestamp.h>

#include <iroc_mission_handler/subtask_executor_interface.h>

namespace iroc_mission_handler {

namespace executors {

namespace siyi_executor {

/**
 * \brief Executor for Siyi camera.
 *
 * This executor handles the initialization, execution, and completion of camera subtask.
 *
 * Parameters format: none
 *
 */
class SiyiCameraExecutor : public iroc_mission_handler::SubtaskExecutor {
 public:
  SiyiCameraExecutor()          = default;
  virtual ~SiyiCameraExecutor() = default;

  bool stop() override;

 protected:
  bool initializeImpl(ros::NodeHandle& nh, const std::string& parameters) override;
  bool startImpl() override;
  bool checkCompletion(double& progress) override;

 private:
  double _capture_wait_time_  = 0.0;

  double progress_  = 0.0;
  bool is_stopped_  = false;

  ros::ServiceClient sc_camera_capture_image_;
};

} // namespace siyi_executor
} // namespace executors
} // namespace iroc_mission_handler

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_mission_handler::executors::siyi_executor::SiyiCameraExecutor, iroc_mission_handler::SubtaskExecutor)
