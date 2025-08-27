#include "siyi_executor/camera_plugin.hpp"

namespace iroc_mission_handler {

namespace executors {

namespace siyi_executor {

bool SiyiCameraExecutor::initializeImpl(ros::NodeHandle& nh, [[maybe_unused]] const std::string& parameters) {

  // Initialize service client for gimbal control
  sc_camera_capture_image_ = nh.serviceClient<siyi_cam_driver::Timestamp>("srv_capture_image_in");

  ROS_DEBUG_STREAM("[SiyiCameraExecutor]: Initialized");
  return true;
}

bool SiyiCameraExecutor::startImpl() {
  // Create and send Timestamp command
  siyi_cam_driver::Timestamp srv;

  if (!sc_camera_capture_image_.call(srv)) {
    ROS_ERROR("[SiyiCameraExecutor]: Failed to call sc_camera_capture_image service");
    return false;
  }
  if (!srv.response.success) {
    ROS_ERROR_STREAM("[SiyiCameraExecutor]: Gimbal control service failed: " << srv.response.message);
    return false;
  }

  ROS_INFO_STREAM("[SiyiCameraExecutor]: " << srv.response.message);
  progress_ = 1;
  is_stopped_ = true;
  return true;
}

bool SiyiCameraExecutor::checkCompletion(double& progress) {
  progress = progress_;
  return is_stopped_;
}

bool SiyiCameraExecutor::stop() {
  // It is not possible to stop the gimbal once it has been set, so we just log this
  ROS_INFO("[SiyiCameraExecutor]: Stopping gimbal executor, but gimbal control cannot be stopped once set.");
  progress_    = 0.0;
  is_stopped_  = false;
  sc_camera_capture_image_.shutdown(); // Shutdown the service client

  return true;
}
} // namespace siyi_executor

} // namespace executors

} // namespace iroc_mission_handler
