#include "ladybug_camera_driver/LadybugInterface.h"

LadybugInterface::LadybugInterface()
    : capture_running_(false), camera_connected_(false) {}

void LadybugInterface::Connect() {
  ROS_DEBUG("[Interface] Connecting to camera ");
  // Initialize context.
  error_ = ladybugCreateContext(&context_);
  ROS_DEBUG("ladybugCreateContext: %s", ladybugErrorToString(error_));

  // Initialize the first ladybug on the bus.
  error_ = ladybugInitializeFromIndex(context_, 0);
  ROS_DEBUG("ladybugInitializeFromIndex: %s", ladybugErrorToString(error_));

  // Load config file from the head
  error_ = ladybugLoadConfig(context_, nullptr);
  ROS_DEBUG("ladybugInitializeFromIndex: %s", ladybugErrorToString(error_));

  // Get camera info
  error_ = ladybugGetCameraInfo(context_, &cam_info_);
  ROS_DEBUG("ladybugGetCameraInfo: %s", ladybugErrorToString(error_));

  camera_connected_ = true;
  Initialize();
}

void LadybugInterface::Disconnect() {
  boost::mutex::scoped_lock scopedLock(mutex_);

  capture_running_ = false;

  // Destroy the context
  error_ = ladybugDestroyContext(&context_);
  ROS_WARN("ladybugDestroyContext: %s", ladybugErrorToString(error_));
}

void LadybugInterface::Start() {
  if (!capture_running_) {
    // Start up Camera

    error_ = ladybugStart(context_, LADYBUG_DATA_FORMAT);
    ROS_DEBUG("ladybugStart: %s", ladybugErrorToString(error_));

    // Throw error if startup broken
    if (error_ != LADYBUG_OK)
      throw std::runtime_error(
          "[LadybugInterface:start] Failed to start capture with error: " +
          std::string(ladybugErrorToString(error_)));
    else {
      ROS_INFO("Ladybug camera successfully started");
      OutputCamInfo();
    }

    // Get byte size of pixels (
    GetPixelByteSize();

    // Get image encodings
    GetImageEncodings();

    capture_running_ = true;
  }
}

bool LadybugInterface::Stop() {
  if (capture_running_) {
    // Stop capturing images
    ROS_WARN("Stopping %s (%u)...\n", cam_info_.pszModelName,
             cam_info_.serialHead);
    capture_running_ = false;

    error_ = ::ladybugStop(context_);
    return true;
  }
  return false;
}

void LadybugInterface::OutputCamInfo() {
  ROS_INFO("   Serial Number : %u", cam_info_.serialBase);
  ROS_INFO("     Serial Head : %u", cam_info_.serialHead);
  ROS_INFO("   Color Enabled : %s",
           cam_info_.bIsColourCamera ? "True" : "False");
  ROS_INFO("     Camera Type : %u", cam_info_.deviceType);
  ROS_INFO("      Model Name : %s", cam_info_.pszModelName);
  ROS_INFO("     Sensor Info : %s", cam_info_.pszSensorInfo);
  ROS_INFO("     Vendor Name : %s", cam_info_.pszVendorName);
  ROS_INFO("         USB Bus : %d", cam_info_.iBusNum);
  ROS_INFO("   Max Bus Speed : %d", cam_info_.maxBusSpeed);
  ROS_INFO("  Interface Type : %u", cam_info_.interfaceType);
}

// Gets byte size of raw & processed pixels
void LadybugInterface::GetPixelByteSize() {
  // Get byte size of raw pixel coming off camera
  switch (LADYBUG_DATA_FORMAT) {
    case LADYBUG_DATAFORMAT_RAW8: bytes_per_raw_pixel_ = 1; break;
    case LADYBUG_DATAFORMAT_RAW12: bytes_per_raw_pixel_ = 1.5; break;
    case LADYBUG_DATAFORMAT_RAW16: bytes_per_raw_pixel_ = 2; break;
    case LADYBUG_DATAFORMAT_JPEG8: bytes_per_raw_pixel_ = 1; break;
    case LADYBUG_DATAFORMAT_COLOR_SEP_JPEG8: bytes_per_raw_pixel_ = 1; break;
  }

  // Get byte size of processed pixel
  switch (PROCESSED_PIXEL_FORMAT) {
    case LADYBUG_BGRU: bytes_per_processed_pixel_ = 4; break;
    case LADYBUG_BGRU16: bytes_per_processed_pixel_ = 8; break;
    case LADYBUG_BGR: bytes_per_processed_pixel_ = 3; break;
    case LADYBUG_RGB: bytes_per_processed_pixel_ = 3; break;
  }
}

void LadybugInterface::GetImageEncodings() {
  // Get IMAGE_MSG_ENCODING based on bayer tile format from camera
  error_ = ladybugGetColorTileFormat(context_, &LADYBUG_BAYER_FORMAT);
  ROS_DEBUG("ladybugGetColorTileFormat: %s", ladybugErrorToString(error_));

  switch (LADYBUG_BAYER_FORMAT) {
    case LADYBUG_RGGB:
      IMAGE_MSG_ENCODING = sensor_msgs::image_encodings::BAYER_RGGB8;
      break;
    case LADYBUG_GRBG:
      IMAGE_MSG_ENCODING = sensor_msgs::image_encodings::BAYER_GRBG8;
      break;
    case LADYBUG_GBRG:
      IMAGE_MSG_ENCODING = sensor_msgs::image_encodings::BAYER_GBRG8;
      break;
    case LADYBUG_BGGR:
      IMAGE_MSG_ENCODING = sensor_msgs::image_encodings::BAYER_BGGR8;
      break;
  }

  // Get PROCESSED_MSG_ENCODING based on specified pixel format
  switch (PROCESSED_PIXEL_FORMAT) {
    case LADYBUG_BGRU:
      PROCESSED_MSG_ENCODING = sensor_msgs::image_encodings::BGRA8;
      break;
    case LADYBUG_BGRU16:
      PROCESSED_MSG_ENCODING = sensor_msgs::image_encodings::BGRA16;
      break;
    case LADYBUG_BGR:
      PROCESSED_MSG_ENCODING = sensor_msgs::image_encodings::BGR8;
      break;
    case LADYBUG_RGB:
      PROCESSED_MSG_ENCODING = sensor_msgs::image_encodings::RGB8;
      break;
  }
}

void LadybugInterface::SetNewConfiguration(
    const ladybug_camera_driver::LadybugConfig& config) {
  // Check if camera is connected
  if (!camera_connected_) { LadybugInterface::Connect(); }

  // Activate mutex to prevent us from grabbing images during this time
  // std::lock_guard<std::mutex> scopedLock(mutex_);
  boost::mutex::scoped_lock scopedLock(mutex_);

  // Gain
  if (config.gain_state > 1) // Check if we are in auto mode
    ladybugSetAbsPropertyEx(context_, LADYBUG_GAIN, false, true, true,
                            config_.gain); // Auto mode, so don't update gain
  else if (config.gain_state > 0)
    ladybugSetAbsPropertyEx(context_, LADYBUG_GAIN, false, true, false,
                            config.gain); // Manual mode, update gain
  else
    ladybugSetAbsPropertyEx(context_, LADYBUG_GAIN, false, false, false,
                            config_.gain); // Param is off

  // Exposure
  if (config.exposure_state > 1) // Check if we are in auto mode
    ladybugSetAbsPropertyEx(
        context_, LADYBUG_AUTO_EXPOSURE, false, true, true,
        config_.exposure); // Auto mode, so don't update gain
  else if (config.exposure_state > 0)
    ladybugSetAbsPropertyEx(context_, LADYBUG_AUTO_EXPOSURE, false, true, false,
                            config.exposure); // Manual mode, update gain
  else
    ladybugSetAbsPropertyEx(context_, LADYBUG_AUTO_EXPOSURE, false, false,
                            false, config_.exposure); // Param is off

  // Shutter
  if (config.shutter_state > 1) // Check if we are in auto mode
    ladybugSetAbsPropertyEx(context_, LADYBUG_SHUTTER, false, true, true,
                            config_.shutter); // Auto mode, so don't update gain
  else if (config.shutter_state > 0)
    ladybugSetAbsPropertyEx(context_, LADYBUG_SHUTTER, false, true, false,
                            config.shutter); // Manual mode, update gain
  else
    ladybugSetAbsPropertyEx(context_, LADYBUG_SHUTTER, false, false, false,
                            config_.shutter); // Param is off

  // Frame Rate
  if (config.frame_rate_enable > 1) // Check if we are in auto mode
    ladybugSetAbsPropertyEx(
        context_, LADYBUG_FRAME_RATE, false, true, true,
        config_.frame_rate); // Auto mode, so don't update gain
  else if (config.frame_rate_enable > 0)
    ladybugSetAbsPropertyEx(context_, LADYBUG_FRAME_RATE, false, true, false,
                            config.frame_rate); // Manual mode, update gain
  else
    ladybugSetAbsPropertyEx(context_, LADYBUG_FRAME_RATE, false, false, false,
                            config_.frame_rate); // Param is off

  // Brightness
  ladybugSetAbsPropertyEx(
      context_, LADYBUG_BRIGHTNESS, false, false, false,
      config_.brightness); // Auto mode, so don't update gain

  // Gamma
  if (config.gamma_state > 0) // Check if we are in auto mode
    ladybugSetAbsPropertyEx(context_, LADYBUG_GAMMA, false, true, false,
                            config.gamma); // Config on, update gamma
  else
    ladybugSetAbsPropertyEx(context_, LADYBUG_GAMMA, false, true, false,
                            config_.gamma); // Config off, don't update gamma

  // SetGain(config.gain);
  // SetAutoExposure(config.exposure);
  // SetWhiteBalance(config.)
  // SetShutter(config.shutter);
  // SetFrameRate(config.frame_rate);
  // SetBrightness(config.brightness);
  // SetGamma(config.gamma);

  GetAutoExposure();
  // SetWhiteBalance(config.)
  GetShutter();
  GetFrameRate();
  GetBrightness();
  GetGamma();
  GetGain();
  /*   if (level >= LEVEL_RECONFIGURE_STOP)
    {
      ROS_DEBUG("SpinnakerCamera::setNewConfiguration: Reconfigure Stop.");
      bool capture_was_running = capture_running_;
      start();  // For some reason some params only work after aquisition has be
    started once. stop(); camera_->setNewConfiguration(config, level); if
    (capture_was_running) start();
    }
    else
    {
      camera_->SetNewConfiguration(config, level);
    } */
} // end SetNewConfiguration

void LadybugInterface::SetGain(const float& gain) {
  ladybugSetAbsPropertyEx(context_, LADYBUG_GAIN, false, true, true, gain);
}

void LadybugInterface::SetAutoExposure(const float& exposure) {
  ladybugSetAbsProperty(context_, LADYBUG_AUTO_EXPOSURE, exposure);
}

void LadybugInterface::SetWhiteBalance(const float& white_balance) {
  ladybugSetAbsProperty(context_, LADYBUG_WHITE_BALANCE, white_balance);
}

void LadybugInterface::SetShutter(const float& shutter) {
  ladybugSetAbsProperty(context_, LADYBUG_SHUTTER, shutter);
}

void LadybugInterface::SetFrameRate(const float& frame_rate) {
  ladybugSetAbsProperty(context_, LADYBUG_FRAME_RATE, frame_rate);
}

void LadybugInterface::SetBrightness(const float& brightness) {
  ladybugSetAbsProperty(context_, LADYBUG_BRIGHTNESS, brightness);
}

void LadybugInterface::SetGamma(const float& gamma) {
  ladybugSetAbsProperty(context_, LADYBUG_GAMMA, gamma);
}

void LadybugInterface::GetGain() {
  float read_gain;
  bool in_one_push_mode, in_auto_mode, is_configurable;
  ladybugGetAbsPropertyEx(context_, LADYBUG_GAIN, &in_one_push_mode,
                          &is_configurable, &in_auto_mode, &read_gain);
  /*  std::cout << "\n[Ladybug Gain] Configurable: " << is_configurable
              << ", Auto Mode: " << in_auto_mode
              << ", Current Value: " << read_gain << std::endl;*/
  config_.gain = read_gain;

  if (in_auto_mode)
    config_.gain_state = 2;
  else if (is_configurable)
    config_.gain_state = 1;
  else
    config_.gain_state = 0;
}

void LadybugInterface::GetAutoExposure() {
  float read_exposure;
  ladybugGetAbsProperty(context_, LADYBUG_AUTO_EXPOSURE, &read_exposure);

  bool in_one_push_mode, in_auto_mode, is_configurable;
  ladybugGetAbsPropertyEx(context_, LADYBUG_AUTO_EXPOSURE, &in_one_push_mode,
                          &is_configurable, &in_auto_mode, &read_exposure);
  /*  std::cout << "[Ladybug Exposure] Configurable: " << is_configurable
              << ", Auto Mode: " << in_auto_mode
              << ", Current Value: " << read_exposure << std::endl;*/
  config_.exposure = read_exposure;

  if (in_auto_mode)
    config_.exposure_state = 2;
  else if (is_configurable)
    config_.exposure_state = 1;
  else
    config_.exposure_state = 0;
}

void LadybugInterface::GetWhiteBalance() {
  float read_white_balance;
  ladybugGetAbsProperty(context_, LADYBUG_WHITE_BALANCE, &read_white_balance);

  bool in_one_push_mode, in_auto_mode, is_configurable;
  ladybugGetAbsPropertyEx(context_, LADYBUG_WHITE_BALANCE, &in_one_push_mode,
                          &is_configurable, &in_auto_mode, &read_white_balance);
  /*  std::cout << "[Ladybug White Balance] Configurable: " << is_configurable
              << ", Auto Mode: " << in_auto_mode
              << ", Current Value: " << read_white_balance << std::endl;*/
  // config_.white_balance = read_white_balance;
}

void LadybugInterface::GetShutter() {
  float read_shutter;
  ladybugGetAbsProperty(context_, LADYBUG_SHUTTER, &read_shutter);

  bool in_one_push_mode, in_auto_mode, is_configurable;
  ladybugGetAbsPropertyEx(context_, LADYBUG_SHUTTER, &in_one_push_mode,
                          &is_configurable, &in_auto_mode, &read_shutter);
  /*  std::cout << "[Ladybug Shutter Speed] Configurable: " << is_configurable
              << ", Auto Mode: " << in_auto_mode
              << ", Current Value: " << read_shutter << std::endl;*/
  config_.shutter = read_shutter;
  if (in_auto_mode)
    config_.shutter_state = 2;
  else if (is_configurable)
    config_.shutter_state = 1;
  else
    config_.shutter_state = 0;
}

void LadybugInterface::GetFrameRate() {
  float read_frame_rate;
  ladybugGetAbsProperty(context_, LADYBUG_FRAME_RATE, &read_frame_rate);

  bool in_one_push_mode, in_auto_mode, is_configurable;
  ladybugGetAbsPropertyEx(context_, LADYBUG_FRAME_RATE, &in_one_push_mode,
                          &is_configurable, &in_auto_mode, &read_frame_rate);
  /*  std::cout << "[Ladybug Frame Rate] Configurable: " << is_configurable
              << ", Auto Mode: " << in_auto_mode
              << ", Current Value: " << read_frame_rate << std::endl;*/
  config_.frame_rate = read_frame_rate;
}

void LadybugInterface::GetBrightness() {
  float read_brightness;
  ladybugGetAbsProperty(context_, LADYBUG_BRIGHTNESS, &read_brightness);

  bool in_one_push_mode, in_auto_mode, is_configurable;
  ladybugGetAbsPropertyEx(context_, LADYBUG_BRIGHTNESS, &in_one_push_mode,
                          &is_configurable, &in_auto_mode, &read_brightness);
  /*  std::cout << "[Ladybug Brightness] Configurable: " << is_configurable
              << ", Auto Mode: " << in_auto_mode
              << ", Current Value: " << read_brightness << std::endl;*/
  config_.brightness = read_brightness;
}

void LadybugInterface::GetGamma() {
  float read_gamma;
  ladybugGetAbsProperty(context_, LADYBUG_GAMMA, &read_gamma);
  bool in_one_push_mode, in_auto_mode, is_configurable;
  ladybugGetAbsPropertyEx(context_, LADYBUG_GAMMA, &in_one_push_mode,
                          &is_configurable, &in_auto_mode, &read_gamma);
  config_.gamma = read_gamma;
  /*  std::cout << "[Ladybug Gamma] Configurable: " << is_configurable
              << ", Auto Mode: " << in_auto_mode
              << ", Current Value: " << read_gamma << std::endl;*/
  config_.gamma_state = is_configurable;
}