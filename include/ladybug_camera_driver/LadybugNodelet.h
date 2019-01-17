#pragma once

#include "ros/ros.h"
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "ladybug_camera_driver/LadybugCamera.h" // The actual standalone library for the Ladybug
#include "ladybug_camera_driver/diagnostics.h"

#include <image_transport/image_transport.h> // ROS library that allows sending compressed images
#include <camera_info_manager/camera_info_manager.h> // ROS library that publishes CameraInfo topics
#include <sensor_msgs/CameraInfo.h> // ROS message header for CameraInfo

#include <sensor_msgs/CompressedImage.h>
#include <image_exposure_msgs/ExposureSequence.h> // Message type for configuring gain and white balance.

#include <diagnostic_updater/diagnostic_updater.h> // Headers for publishing diagnostic messages.
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h> // Needed for the dynamic_reconfigure gui service to run

#include <boost/thread.hpp> // Needed for the nodelet to launch the reading thread.
#include <fstream>

namespace ladybug_camera_driver {
class LadybugCameraNodelet : public nodelet::Nodelet {
 public:
  LadybugCameraNodelet() = default;

  ~LadybugCameraNodelet() override;

 private:

  /**
 * Function called when nodelet is loaded (serves as a psuedo-constructor for the LadybugNodelet class).
 * This function needs to do the MINIMUM amount of work to get the nodelet running.
 * Nodelets should not call blocking functions here.
 * Grabs ROS parameters, initializes dynamic reconfigure, initializes camera info manager,
 */
  void onInit() override;

  /**
   * @brief Method for configuring ladybug parameters via. dynamic reconfig
   * This function serves as a callback for the dynamic reconfigure service. It simply passes the configuration
   * object to the driver to allow the camera to reconfigure.
   * @param config camera_library::CameraConfig object passed by reference. Values will be changed to those the driver is currently using.
   * @param level driver_base reconfiguration level. See driver_base/SensorLevels.h for more information.
   */
  void ConfigCallback(ladybug_camera_driver::LadybugConfig &config, uint32_t level);

  /**
   * Callback for receiving diagnostic messages from the ladybug.
   */
  void DiagnosticsCallback();

  /**
   * Callback for when the subscriber status for one of the relevant topics changes.
   * Case 1: No subscribers, Connected
   * Case 2: No subscribers, Disconnected
   */
  void ConnectCallback();

  /**
   * Function for the boost::thread to grab diagnostics & publish them.
   */
  void DiagnosticsPoll();

  /**
   * Function for the boost::thread to grabImages and publish them.
   * This function continues until the thread is interrupted. Responsible for getting images and publishing them.
   */
  void DevicePoll();

  //! Group camera diagnostics
  std::unique_ptr<DiagnosticsManager> diagnostics_manager_; // Manager for diagnostics
  diagnostic_updater::Updater updater_; // Handles publishing diagnostics messages.
  std::shared_ptr<ros::Publisher> diagnostics_pub_; // Publisher for publishing camera diagnostics

  //! Group threading
  boost::recursive_mutex config_mutex_; // For configuring camera settings
  boost::mutex connect_mutex_; // For connecting to camera
  boost::shared_ptr<boost::thread> publish_thread_; // The thread that reads and publishes the images.
  boost::shared_ptr<boost::thread> diagnostics_thread_; // The thread that reads and publishes the diagnostics.

  //! Group image publishing
  bool publish_raw_images_ = true; // Boolean for whether we should grab raw images (vs. compressed) [Rosparam].
  std::string frame_id_ = "camera"; // Frame id for the camera messages [Rosparam].

  std::shared_ptr<image_transport::ImageTransport>
      transport_; // Needed to initialize and keep the ImageTransport in scope.
  image_transport::CameraPublisher
      transport_pub_[6];   // 6 camera publishers corresponding to each of the ladybug cameras

  // Diagnosed publishers, need to be a pointer because of constructor requirements
  std::shared_ptr<diagnostic_updater::DiagnosedPublisher<ladybug_msgs::LadybugTiles> > tiles_publisher_;
  std::shared_ptr<diagnostic_updater::DiagnosedPublisher<sensor_msgs::CompressedImage> > jpeg_publisher_;

  // Needed to initialize and keep the CameraInfoManager in scope.
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::CameraInfoPtr camera_info_[6]; // Camera Info message.

  // Needed to initialize and keep the dynamic_reconfigure::Server in scope.
  std::shared_ptr<dynamic_reconfigure::Server<ladybug_camera_driver::LadybugConfig> > reconfig_srv_;

  LadybugCamera ladybug_; // Instance of the LadybugCamera wrapper class, used to interface with the camera.

  ros::Subscriber sub_; // Subscriber for gain and white balance changes.

  // Camera settings image parameters
  std::string data_capture_format_ = "raw8";
  std::string color_processing_method_ = "downsample16";
  std::string processed_pixel_format_ = "bgr";

  // Diagnostics publisher parameters
  double diagnostics_desired_freq_ = 10; // Desired frequency for publishing camera diagnostics.
  double
      diagnostics_min_freq_ = diagnostics_desired_freq_; // Minimum publish frequency for publishing camera diagnostics.
  double
      diagnostics_max_freq_ = diagnostics_desired_freq_; // Maximum publish frequency for publishing camera diagnostics.
  double diagnostics_freq_tolerance_ = 0.1; // Tolerance before stating error on publish frequency, fraction of desired.
  int diagnostics_window_size_ = 10; // Number of samples to consider in frequency
  double diagnostics_min_acceptable_ =
      0; // The minimum publishing delay (in seconds) before warning.  Negative values mean future dated messages.
  double diagnostics_max_acceptable_ = 0.01; // The maximum publishing delay (in seconds) before warning.

  ladybug_camera_driver::LadybugConfig config_; // Ladybug configuration file
};

PLUGINLIB_EXPORT_CLASS(ladybug_camera_driver::LadybugCameraNodelet, nodelet::Nodelet)  // Needed for Nodelet declaration
}