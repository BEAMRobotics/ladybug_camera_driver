#include "ladybug_camera_driver/LadybugNodelet.h"

namespace ladybug_camera_driver {

LadybugCameraNodelet::~LadybugCameraNodelet() {
  boost::mutex::scoped_lock scopedLock(connect_mutex_);

  if (publish_thread_) {
    publish_thread_->interrupt();
    publish_thread_->join();

    try {
      NODELET_INFO("Stopping camera capture.");
      ladybug_.stop();
      NODELET_INFO("Disconnecting from camera.");
      ladybug_.disconnect();
    }
    catch (std::runtime_error &e) {
      NODELET_ERROR("%s", e.what());
    }
  }
}

void LadybugCameraNodelet::onInit() {

  // Get nodeHandles
  ros::NodeHandle &nh = getMTNodeHandle();
  ros::NodeHandle &pnh = getMTPrivateNodeHandle();

  // ROS params
  pnh.param<std::string>("data_capture_format", data_capture_format_, "raw8");
  pnh.param<std::string>("color_processing_method", color_processing_method_, "downsample16");
  pnh.param<std::string>("processed_pixel_format", processed_pixel_format_, "bgr");
  pnh.param<bool>("publish_raw_images", publish_raw_images_, true);
  pnh.param<std::string>("frame_id", frame_id_, "camera");
  pnh.param<double>("diagnostics_desired_freq", diagnostics_desired_freq_, 10);
  pnh.param<double>("diagnostics_min_freq", diagnostics_min_freq_, diagnostics_desired_freq_);
  pnh.param<double>("diagnostics_max_freq", diagnostics_max_freq_, diagnostics_desired_freq_);
  pnh.param<double>("diagnostics_freq_tolerance", diagnostics_freq_tolerance_, 0.1);
  pnh.param<int>("diagnostics_window_size", diagnostics_window_size_, 10);
  pnh.param<double>("diagnostics_min_acceptable_delay", diagnostics_min_acceptable_, 0.0);
  pnh.param<double>("diagnostics_max_acceptable_delay", diagnostics_max_acceptable_, 0.01);

  // Set relevant parameters on camera
  ladybug_.SetDataCaptureFormat(data_capture_format_);
  ladybug_.SetColorProcessingMethod(color_processing_method_);
  ladybug_.SetProcessedPixelFormat(processed_pixel_format_);
  ladybug_.collect_raw_ = publish_raw_images_;


  // TODO: Add frames for each image of ladybug
  // TODO: Add param for saving ladybug .conf file
  // TODO: Add implementation for setting ladybug frame rate

  // Do not call the ConnectCallback function until after we are done initializing.
  boost::mutex::scoped_lock scoped_lock(connect_mutex_);

  // Initialize the dynamic reconfigure server / bind the ConfigCallback
  reconfig_srv_ =
      std::make_shared<dynamic_reconfigure::Server<ladybug_camera_driver::LadybugConfig> >(config_mutex_, pnh);
  dynamic_reconfigure::Server<ladybug_camera_driver::LadybugConfig>::CallbackType config_callback =
      boost::bind(&ladybug_camera_driver::LadybugCameraNodelet::ConfigCallback, this, _1, _2);
  reconfig_srv_->setCallback(config_callback);

  // Initialize the camera info manager
  camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(nh));

  // Initialize image transport / image transport SubscriberStatus callback (for monitoring subscriptions)
  transport_.reset(new image_transport::ImageTransport(nh));
  image_transport::SubscriberStatusCallback
      transport_sub_cb = boost::bind(&LadybugCameraNodelet::ConnectCallback, this);

  for (unsigned int i = 0; i < LADYBUG_NUM_CAMERAS; i++) {
    const std::string istr = std::to_string(i);
    const std::string topic = std::string("/ladybug/camera_") + istr + std::string("/image_raw");
    transport_pub_[i] = transport_->advertiseCamera(topic, 5, transport_sub_cb, transport_sub_cb);
  }

  // Set up diagnostics
  updater_.setHardwareID("ladybug_camera");

  ros::SubscriberStatusCallback tiles_sub_cb = boost::bind(&LadybugCameraNodelet::ConnectCallback, this);

  // Set up a diagnostics publisher (clang-format messing up formating)
  tiles_publisher_
      .reset(new diagnostic_updater::DiagnosedPublisher<ladybug_msgs::LadybugTiles>(nh.advertise<ladybug_msgs::LadybugTiles>(
          "image_tiles", 5, tiles_sub_cb, tiles_sub_cb), updater_,
                                                                                    diagnostic_updater::FrequencyStatusParam(
                                                                                        &diagnostics_min_freq_,
                                                                                        &diagnostics_max_freq_,
                                                                                        diagnostics_freq_tolerance_,
                                                                                        diagnostics_window_size_),
                                                                                    diagnostic_updater::TimeStampStatusParam(
                                                                                        diagnostics_min_acceptable_,
                                                                                        diagnostics_max_acceptable_)));

  /*
  jpeg_publisher_.reset(new diagnostic_updater::DiagnosedPublisher<sensor_msgs::CompressedImage>(nh.advertise<sensor_msgs::CompressedImage>("image_test/compressed", 5, tiles_sub_cb, tiles_sub_cb),
             updater_,
             diagnostic_updater::FrequencyStatusParam(&diagnostics_min_freq_, &diagnostics_max_freq_, freq_tolerance, window_size),
             diagnostic_updater::TimeStampStatusParam(min_acceptable, max_acceptable)));
  */

  // Set up diagnostics aggregator publisher and diagnostics manager
  ros::SubscriberStatusCallback diag_cb = boost::bind(&LadybugCameraNodelet::DiagnosticsCallback, this);
  diagnostics_pub_
      .reset(new ros::Publisher(nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1, diag_cb, diag_cb)));

  diagnostics_manager_ = std::unique_ptr<DiagnosticsManager>(new DiagnosticsManager(frame_id_,
                                                                                    std::to_string(ladybug_
                                                                                                       .getSerial()),
                                                                                    diagnostics_pub_));

  // TODO: Add the diagnostics available for the ladybug (pressure, temperature)
  /*diagnostics_manager_->addDiagnostic("DeviceTemperature", true, std::make_pair(0.0f, 90.0f), -10.0f, 95.0f);
    diagnostics_manager_->addDiagnostic("AcquisitionResultingFrameRate", true, std::make_pair(10.0f, 60.0f), 5.0f, 90.0f);
    diagnostics_manager_->addDiagnostic("PowerSupplyVoltage", true, std::make_pair(4.5f, 5.2f), 4.4f, 5.3f);
    diagnostics_manager_->addDiagnostic("PowerSupplyCurrent", true, std::make_pair(0.4f, 0.6f), 0.3f, 1.0f);
    diagnostics_manager_->addDiagnostic<int>("DeviceUptime");
    diagnostics_manager_->addDiagnostic<int>("U3VMessageChannelID"); */
}

void LadybugCameraNodelet::ConfigCallback(ladybug_camera_driver::LadybugConfig &config, uint32_t level) {
  NODELET_DEBUG("Re-configure callback");
  config_ = config;

  try {
    NODELET_INFO("Dynamic reconfigure callback with level: %d", level);
    ladybug_.setNewConfiguration(config);
  }
  catch (std::runtime_error &e) {
    NODELET_ERROR("Reconfigure Callback failed with error: %s", e.what());
  }
  config = ladybug_.config_;
}

void LadybugCameraNodelet::DiagnosticsCallback() {
  if (!diagnostics_thread_)  // Can't publish diagnostics if we aren't connected
  {
    // Start the thread to loop through and publish messages
    diagnostics_thread_.reset(
        new boost::thread(boost::bind(&ladybug_camera_driver::LadybugCameraNodelet::DiagnosticsPoll, this)));
  }
}

void LadybugCameraNodelet::ConnectCallback() {
  NODELET_INFO("Connect callback!");

  // Grab the connect_mutex. Wait until we're done initializing before letting this function through.
  boost::mutex::scoped_lock scoped_lock(connect_mutex_);

  // Check if we should disconnect (there are 0 subscribers to our data)
  if (transport_pub_[0].getNumSubscribers() == 0 && tiles_publisher_->getPublisher().getNumSubscribers() == 0) {
    // Only bother disconnecting if we've already connected
    if (publish_thread_) {
      NODELET_INFO("Disconnecting from ladybug.");
      publish_thread_->interrupt();
      scoped_lock.unlock();
      publish_thread_->join();
      scoped_lock.lock();
      publish_thread_.reset();
      sub_.shutdown();

      try {
        NODELET_INFO("Stopping ladybug camera capture.");
        ladybug_.stop();
      }
      catch (std::runtime_error &e) {
        NODELET_ERROR("%s", e.what());
      }

      try {
        NODELET_INFO("Disconnecting from ladybug camera.");
        ladybug_.disconnect();
      }
      catch (std::runtime_error &e) {
        NODELET_ERROR("%s", e.what());
      }
    }
  } else if (!publish_thread_) // We need to connect
  {
    // Start the thread to loop through and publish messages
    publish_thread_
        .reset(new boost::thread(boost::bind(&ladybug_camera_driver::LadybugCameraNodelet::DevicePoll, this)));
  } else {
    NODELET_INFO("Do nothing in callback.");
  }
}

void LadybugCameraNodelet::DiagnosticsPoll() {
  while (!boost::this_thread::interruption_requested())  // Block until we need to stop this thread.
  {
    diagnostics_manager_->processDiagnostics(&ladybug_);
  }
}

void LadybugCameraNodelet::DevicePoll() {
  enum State {
    NONE, ERROR, STOPPED, DISCONNECTED, CONNECTED, STARTED
  };

  State state = DISCONNECTED;
  State previous_state = NONE;

  while (!boost::this_thread::interruption_requested())   // Block until we need to stop this thread.
  {
    clock_t start1 = clock();
    ros::WallTime start_ = ros::WallTime::now();

    bool state_changed = (state != previous_state);

    previous_state = state;

    switch (state) {
      case ERROR:
        // Generally there's no need to stop before disconnecting after an
        // error. Indeed, stop will usually fail.
#if STOP_ON_ERROR
        // Try stopping the camera
        {
          boost::mutex::scoped_lock scopedLock(connect_mutex_);
          sub_.shutdown();
        }
        try
        {
          NODELET_INFO("Stopping camera.");
          ladybug_.stop();
          NODELET_INFO("Stopped camera.");

          state = STOPPED;
        }
        catch(std::runtime_error& e)
        {
          NODELET_ERROR_COND(state_changed,
              "Failed to stop error: %s", e.what());
          ros::Duration(1.0).sleep(); // sleep for one second each time
        }
        break;
#endif
      case STOPPED:
        // Try disconnecting from the camera
        try {
          NODELET_INFO("Disconnecting from camera.");
          ladybug_.disconnect();
          NODELET_INFO("Disconnected from camera.");
          state = DISCONNECTED;
        }
        catch (std::runtime_error &e) {
          NODELET_ERROR_COND(state_changed,
                             "Failed to disconnect with error: %s", e.what());
          ros::Duration(1.0).sleep(); // sleep for one second each time
        }
        break;
      case DISCONNECTED:
        // Try connecting to the camera
        try {
          NODELET_INFO("Connecting to camera.");
          ladybug_.connect();
          NODELET_INFO("Connected to camera.");

          // Set last configuration, forcing the reconfigure level to stop
          //ladybug_.setNewConfiguration(config_, LadybugCamera::LEVEL_RECONFIGURE_STOP);

          // Set the timeout for grabbing images.
          try {
            double timeout;
            getMTPrivateNodeHandle().param("timeout", timeout, 1.0);
            NODELET_INFO("Setting timeout to: %f.", timeout);
            //ladybug_.setTimeout(timeout);
          }
          catch (std::runtime_error &e) {
            NODELET_ERROR("%s", e.what());
          }
          state = CONNECTED;
        }
        catch (std::runtime_error &e) {
          NODELET_ERROR_COND(state_changed,
                             "Failed to connect with error: %s", e.what());
          ros::Duration(1.0).sleep(); // sleep for one second each time
        }

        break;
      case CONNECTED:
        // Try starting the camera
        try {
          NODELET_INFO("Starting camera.");
          ladybug_.start();
          NODELET_INFO("Started camera.");
          NODELET_INFO("If nothing subscribes to the camera topic, the camera_info is not published .");
          state = STARTED;
        }
        catch (std::runtime_error &e) {
          NODELET_ERROR_COND(state_changed,
                             "Failed to start with error: %s", e.what());
          ros::Duration(1.0).sleep(); // sleep for one second each time
        }
        break;
      case STARTED:
        try {
          if (publish_raw_images_) {
            ladybug_msgs::LadybugTilesPtr tile_images = boost::make_shared<ladybug_msgs::LadybugTiles>();
            ladybug_.grabImage(tile_images, frame_id_);

            // Loop over each of the (6) ladybug cameras
            for (unsigned int i = 0; i < 6; i++) {
              camera_info_[i].reset(new sensor_msgs::CameraInfo(camera_info_manager_->getCameraInfo()));
              camera_info_[i]->header.stamp = tile_images->header.stamp;
              camera_info_[i]->header.frame_id = tile_images->images[i].header.frame_id;
              camera_info_[i]->height = tile_images->images[i].height;
              camera_info_[i]->width = tile_images->images[i].width;
              // The height, width, distortion model, and parameters are all filled in by camera info manager.
              camera_info_[i]->binning_x = 0;
              camera_info_[i]->binning_y = 0;
              camera_info_[i]->roi.x_offset = 0;
              camera_info_[i]->roi.y_offset = 0;
              camera_info_[i]->roi.height = 0;
              camera_info_[i]->roi.width = 0;
              camera_info_[i]->roi.do_rectify = false;

              transport_pub_[i]
                  .publish(boost::make_shared<sensor_msgs::Image>(tile_images->images[i]), camera_info_[i]);
            }
            tiles_publisher_->publish(tile_images);
          } else {
            // Case for collecting jpegs
            std::vector<sensor_msgs::ImagePtr> jpeg_tiles;
            ladybug_.grabImageJpeg(jpeg_tiles, frame_id_);
          }
        }
        catch (CameraTimeoutException &e) {
          NODELET_WARN("%s", e.what());
        }
        catch (CameraImageConsistencyError &e) {
          NODELET_WARN("%s", e.what());
        }
        catch (std::runtime_error &e) {
          NODELET_ERROR("%s", e.what());
          state = ERROR;
        }
        break;
      default:NODELET_ERROR("Unknown camera state %d!", state);
    }
    // Update diagnostics
    //updater_.update();
  }
  NODELET_INFO("Leaving thread.");
}

} /* namespace */
