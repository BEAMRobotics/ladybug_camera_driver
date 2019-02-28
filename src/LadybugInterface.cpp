#include "ladybug_camera_driver/LadybugInterface.h"

#define COLOR_PROCESSING_METHOD LADYBUG_RIGOROUS //LADYBUG_RIGOROUS //LADYBUG_DOWNSAMPLE16  //LADYBUG_RIGOROUS
#define CONVERT_IMAGE_PFORMAT LADYBUG_BGRU //LADYBUG_BGRU

LadybugInterface::LadybugInterface() {
  capture_running_ = false;
  camera_connected_ = false;
}


void LadybugInterface::Connect() {

  ROS_DEBUG("Connecting to camera");
  // Initialize context.
  error_ = ::ladybugCreateContext(&context_);

  // Initialize the first ladybug on the bus.
  error_ = ::ladybugInitializeFromIndex(context_, 0);

  // Load config file from the head
  error_ = ladybugLoadConfig(context_, nullptr);

  // Get camera info
  error_ = ladybugGetCameraInfo(context_, &cam_info_);

  // Set color processing method
  error_ = ::ladybugSetColorProcessingMethod(context_, COLOR_PROCESSING_METHOD);

  // Make the rendering engine use the alpha mask
  error_ = ladybugSetAlphaMasking(context_, false);

  // Set frame rate
  ladybug_frame_rate_ = 60.0f;
  error_ = ladybugSetAbsProperty(context_, LADYBUG_FRAME_RATE, ladybug_frame_rate_);

  //error_ = ::ladybugSetGrabTimeout(context_, 75);
  //std::cout << "[ladybugSetGrabTimeout] Done: " << ladybugErrorToString(error_) << std::endl;
  camera_connected_ = true;
}

void LadybugInterface::Disconnect() {
  boost::mutex::scoped_lock scopedLock(mutex_);

  capture_running_ = false;

  // Destroy the context
  printf("Destroying context...\n");
  error_ = ::ladybugDestroyContext(&context_);
}

void LadybugInterface::Start() {
  if (!capture_running_) {
    // Start up Camera

    ladybug_packet_size_ = 56000;
    ladybug_buffer_size_ = 0;

    error_ = ::ladybugStart(context_, ladybug_data_format_);
    // ::ladybugStartEx(context_, ladybug_data_format_, ladybug_packet_size_, ladybug_buffer_size_);

    // Throw error if startup broken
    if (error_ != LADYBUG_OK)
      throw std::runtime_error(
          "[LadybugInterface:start] Failed to start capture with error: " + std::string(ladybugErrorToString(error_)));
    else {
      ROS_INFO("Ladybug camera successfully started");
      OutputCamInfo();
    }

    //Get byte size of pixels
    GetPixelByteSize();

    //Get image encodings
    GetImageEncodings();

    // Allocate memory for the 6 processed images
    // Currently this isn't used
    for (unsigned int uiCamera = 0; uiCamera < 6; uiCamera++) {
      arp_buffers_[uiCamera] = new unsigned char[image_rows_ * image_cols_ * bytes_per_processed_pixel_];
    }

    capture_running_ = true;
  }
}

bool LadybugInterface::Stop() {
  if (capture_running_) {
    // Stop capturing images
    printf("Stopping %s (%u)...\n", cam_info_.pszModelName, cam_info_.serialHead);
    capture_running_ = false;

    error_ = ::ladybugStop(context_);
    return true;
  }
  return false;
}

void LadybugInterface::GrabImage(ladybug_msgs::LadybugTilesPtr& tiles,
                              const std::string& frame_id) {

  boost::mutex::scoped_lock scopedLock(mutex_);

  if (capture_running_) {
    // Grab image from camera, store in image_
    error_ = ::ladybugGrabImage(context_, &image_);

    // Set msg header information
    tiles->header.stamp.sec = image_.timeStamp.ulSeconds;
    tiles->header.stamp.nsec = image_.timeStamp.ulMicroSeconds;
    tiles->header.frame_id = frame_id;
    tiles->header.seq = image_.imageInfo.ulSequenceId;

    // Populate LadybugTiles message
    for (unsigned int cam = 0; cam < LADYBUG_NUM_CAMERAS; cam++) {
      sensor_msgs::Image tile;
      std::ostringstream s;
      s << frame_id << "_" << cam;
      tile.header.frame_id = s.str();
      fillImage(tile,
                raw_image_encoding_,
                image_rows_,
                image_cols_,
                image_cols_ * bytes_per_raw_pixel_,
                image_.pData + (cam * image_rows_ * image_cols_));
      tiles->images.push_back(tile);
    }
  } else if (camera_connected_) {
    throw CameraNotRunningException(
        "LadybugInterface::GrabImage: Camera is currently not running.  Please start the capture.");
  } else {
    throw std::runtime_error("LadybugInterface not connected!");
  }
}

// GrabImage function for JPEGs
void LadybugInterface::grabImageJpeg(std::vector<sensor_msgs::Image::Ptr> &images, const std::string &frame_id) {
  std::cout << "Trying to grab image--------------------" << std::endl;
  boost::mutex::scoped_lock scopedLock(mutex_);
  images.resize(6);

  float lb_rate;
  LadybugProperty frame_rate = LADYBUG_FRAME_RATE;
  error_ = ladybugGetAbsProperty(context_, frame_rate, &lb_rate);
  std::cout << "[ladybugGetAbsProperty] Done: " << ladybugErrorToString(error_) << std::endl;
  std::cout << " Frame rate = " << lb_rate << std::endl;
  if (capture_running_) {

    // Grab image from camera, store in image_
    ros::WallTime start1_ = ros::WallTime::now();
    error_ = ::ladybugGrabImage(context_, &image_);

    std::cout << "[ladybugGrabImage] Done: " << ladybugErrorToString(error_) << std::endl;
    //std::cout << "Grabbed image" << std::endl;
    ros::WallTime stop1_ = ros::WallTime::now();
    double execution_time1 = (stop1_ - start1_).toNSec() * 1e-6;
    ROS_INFO_STREAM("[Timing] LadybugGrabImage (ms): " << execution_time1);

    ros::WallTime start3_ = ros::WallTime::now();
    // Allocate memory for the 6 processed images
    for (unsigned int uiCamera = 0; uiCamera < LADYBUG_NUM_CAMERAS; uiCamera++) {
      //std::cout << arp_buffers_[uiCamera] << std::endl;
      memset(arp_buffers_[uiCamera], 0xff, image_rows_ * image_cols_ * bytes_per_processed_pixel_);
    }
    // Color-process the image (GPU ENABLED)
    error_ = ::ladybugConvertImageGPU(context_, &image_, arp_buffers_, LADYBUG_RGBU);
    std::cout << "[ladybugConvertImage] Done: " << ladybugErrorToString(error_) << std::endl;
    ros::WallTime stop3_ = ros::WallTime::now();
    double execution_time3 = (stop3_ - start3_).toNSec() * 1e-6;
    ROS_INFO_STREAM("     [Timing] Conversion time (ms): " << execution_time3);

    ros::WallTime start2_ = ros::WallTime::now();
    for (unsigned int i = 0; i < 6; i++) {
      if (!images[i])
        images[i].reset(new sensor_msgs::Image);
      sensor_msgs::Image &img = *(images[i]);
      img.header.stamp.sec = image_.timeStamp.ulSeconds;
      img.header.stamp.nsec = image_.timeStamp.ulMicroSeconds;
      img.header.frame_id = frame_id;
      img.height = image_rows_; //FULL_HEIGHT;
      img.width = image_cols_; //FULL_WIDTH;
      img.is_bigendian = false;
      img.step = img.width * bytes_per_processed_pixel_;
      img.data.resize(img.height * img.width * bytes_per_processed_pixel_);
      std::string imageEncoding = sensor_msgs::image_encodings::BGR8;
      fillImage(img,
                imageEncoding,
                image_rows_,
                image_cols_,
                image_cols_ * bytes_per_processed_pixel_,
                arp_buffers_[i]);
      //fillImage(img, m_bayerEncoding, 2048, 2464, 2464 * bytes_per_raw_pixel_, image_.pData + (i * 2048 * 2464));
      std::cout << "Copied data on image " << i << std::endl;
    }
    ros::WallTime stop2_ = ros::WallTime::now();
    double execution_time2 = (stop2_ - start2_).toNSec() * 1e-6;
    ROS_INFO_STREAM("[Timing] Converting to ROS msg (ms): " << execution_time2);

  } else if (camera_connected_) {
    throw CameraNotRunningException(
        "LadybugInterface::GrabImage: Camera is currently not running.  Please start the capture.");
  } else {
    throw std::runtime_error("LadybugInterface not connected!");
  }
}

void LadybugInterface::SetDataCaptureFormat(const std::string &datatype) {
  if (datatype == "raw8") {
    ladybug_data_format_ = LADYBUG_DATAFORMAT_RAW8;
  } else if (datatype == "jpeg8") {
    ladybug_data_format_ = LADYBUG_DATAFORMAT_COLOR_SEP_JPEG8;
  } else {
    return;
  }
  return;
}

void LadybugInterface::SetColorProcessingMethod(const std::string &color_processing_method) {
  if (color_processing_method == "disable") {
    ladybug_color_processing_method_ = LADYBUG_DISABLE;
  } else if (color_processing_method == "edge_sensing") {
    ladybug_color_processing_method_ = LADYBUG_EDGE_SENSING;
  } else if (color_processing_method == "rigorous") {
    ladybug_color_processing_method_ = LADYBUG_RIGOROUS;
  } else if (color_processing_method == "downsample4") {
    ladybug_color_processing_method_ = LADYBUG_DOWNSAMPLE4;
  } else if (color_processing_method == "downsample16") {
    ladybug_color_processing_method_ = LADYBUG_DOWNSAMPLE16;
  } else if (color_processing_method == "mono") {
    ladybug_color_processing_method_ = LADYBUG_MONO;
  } else if (color_processing_method == "hqlinear") {
    ladybug_color_processing_method_ = LADYBUG_HQLINEAR;
  } else if (color_processing_method == "directional") {
    ladybug_color_processing_method_ = LADYBUG_DIRECTIONAL_FILTER;
  } else if (color_processing_method == "weighted_directional") {
    ladybug_color_processing_method_ = LADYBUG_WEIGHTED_DIRECTIONAL_FILTER;
  } else {
    return;
  }
  return;
}

void LadybugInterface::SetProcessedPixelFormat(const std::string &processed_pixel_format) {
  if (processed_pixel_format == "mono8") {
    ladybug_processed_pixel_format_ = LADYBUG_MONO8;
  } else if (processed_pixel_format == "raw8") {
    ladybug_processed_pixel_format_ = LADYBUG_RAW8;
  } else if (processed_pixel_format == "bgr") {
    ladybug_processed_pixel_format_ = LADYBUG_BGR;
  } else if (processed_pixel_format == "bgru") {
    ladybug_processed_pixel_format_ = LADYBUG_BGRU;
  } else if (processed_pixel_format == "rgb") {
    ladybug_processed_pixel_format_ = LADYBUG_RGB;
  } else if (processed_pixel_format == "rgbu") {
    ladybug_processed_pixel_format_ = LADYBUG_RGBU;
  } else {
    return;
  }
  return;
}

void LadybugInterface::OutputCamInfo() {
  std::cout << "      Serial Number:" << cam_info_.serialBase << std::endl;
  std::cout << "      Serial Head:" << cam_info_.serialHead << std::endl;
  std::cout << "      Color Enabled:" << cam_info_.bIsColourCamera << std::endl;
  std::cout << "      Camera Type:" << cam_info_.deviceType << std::endl;
  std::cout << "      Model Name:" << cam_info_.pszModelName << std::endl;
  std::cout << "      Sensor Info:" << cam_info_.pszSensorInfo << std::endl;
  std::cout << "      Vendor Name:" << cam_info_.pszVendorName << std::endl;
  std::cout << "      USB Bus:" << cam_info_.iBusNum << std::endl;
  std::cout << "      Max Bus Speed:" << cam_info_.maxBusSpeed << std::endl;
  std::cout << "      Interface Type:" << cam_info_.interfaceType << std::endl;
  return;
}

// Gets byte size of raw & processed pixels
void LadybugInterface::GetPixelByteSize() {
  //Get byte size of raw pixel coming off camera
  switch (ladybug_data_format_) {
    case LADYBUG_DATAFORMAT_RAW8:bytes_per_raw_pixel_ = 1;
      break;
    case LADYBUG_DATAFORMAT_RAW12:bytes_per_raw_pixel_ = 1.5;
      break;
    case LADYBUG_DATAFORMAT_RAW16:bytes_per_raw_pixel_ = 2;
      break;
    case LADYBUG_DATAFORMAT_JPEG8:bytes_per_raw_pixel_ = 1;
      break;
    case LADYBUG_DATAFORMAT_COLOR_SEP_JPEG8:bytes_per_raw_pixel_ = 1;
      break;
  }

  // Get byte size of processed pixel
  switch (ladybug_processed_pixel_format_) {
    case LADYBUG_BGRU:bytes_per_processed_pixel_ = 4;
      break;
    case LADYBUG_BGRU16:bytes_per_processed_pixel_ = 8;
      break;
    case LADYBUG_BGR:bytes_per_processed_pixel_ = 3;
      break;
    case LADYBUG_RGB:bytes_per_processed_pixel_ = 3;
      break;
  }

  return;
}

// Gets raw_image_ and processed_image_ encodings based on
// bayer tile format & processed utput pixel format
void LadybugInterface::GetImageEncodings() {

  // Get raw_image_encoding_ based on bayer tile format from camera
  error_ = ladybugGetColorTileFormat(context_, &ladybug_bayer_format_);
  std::cout << "[ladybugGetColorTileFormat] Done: " << ladybugErrorToString(error_) << std::endl;
  switch (ladybug_bayer_format_) {
    case LADYBUG_RGGB:raw_image_encoding_ = sensor_msgs::image_encodings::BAYER_RGGB8;
      break;
    case LADYBUG_GRBG:raw_image_encoding_ = sensor_msgs::image_encodings::BAYER_GRBG8;
      break;
    case LADYBUG_GBRG:raw_image_encoding_ = sensor_msgs::image_encodings::BAYER_GBRG8;
      break;
    case LADYBUG_BGGR:raw_image_encoding_ = sensor_msgs::image_encodings::BAYER_BGGR8;
      break;
  }

  // Get processed_image_encoding_ based on specified pixel format
  switch (ladybug_processed_pixel_format_) {
    case LADYBUG_BGRU:processed_image_encoding_ = sensor_msgs::image_encodings::BGRA8;
      break;
    case LADYBUG_BGRU16:processed_image_encoding_ = sensor_msgs::image_encodings::BGRA16;
      break;
    case LADYBUG_BGR:processed_image_encoding_ = sensor_msgs::image_encodings::BGR8;
      break;
    case LADYBUG_RGB:processed_image_encoding_ = sensor_msgs::image_encodings::RGB8;
      break;
  }

}

void LadybugInterface::GetImageSize() {
  // Get the number of rows & columns for our color processing
  if (COLOR_PROCESSING_METHOD == LADYBUG_DOWNSAMPLE4 ||
      COLOR_PROCESSING_METHOD == LADYBUG_MONO) {
    image_cols_ = image_.uiCols / 2;
    image_rows_ = image_.uiRows / 2;
  } else if (COLOR_PROCESSING_METHOD == LADYBUG_DOWNSAMPLE16) {
    image_cols_ = image_.uiCols / 4;
    image_rows_ = image_.uiRows / 4;
  } else {
    image_cols_ = image_.uiCols;
    image_rows_ = image_.uiRows;
  }
}

void LadybugInterface::SetNewConfiguration(
    const ladybug_camera_driver::LadybugConfig& config) {
  // Check if camera is connected
  if (!camera_connected_) { LadybugInterface::Connect();
  }

  // Activate mutex to prevent us from grabbing images during this time
  //std::lock_guard<std::mutex> scopedLock(mutex_);
  boost::mutex::scoped_lock scopedLock(mutex_);


  // Gain
  if (config.gain_state > 1) // Check if we are in auto mode
    ladybugSetAbsPropertyEx(context_, LADYBUG_GAIN, false, true, true, config_.gain); // Auto mode, so don't update gain
  else if (config.gain_state > 0)
    ladybugSetAbsPropertyEx(context_, LADYBUG_GAIN, false, true, false, config.gain); // Manual mode, update gain
  else
    ladybugSetAbsPropertyEx(context_, LADYBUG_GAIN, false, false, false, config_.gain); // Param is off

  // Exposure
  if (config.exposure_state > 1) // Check if we are in auto mode
    ladybugSetAbsPropertyEx(context_,
                            LADYBUG_AUTO_EXPOSURE,
                            false,
                            true,
                            true,
                            config_.exposure); // Auto mode, so don't update gain
  else if (config.exposure_state > 0)
    ladybugSetAbsPropertyEx(context_,
                            LADYBUG_AUTO_EXPOSURE,
                            false,
                            true,
                            false,
                            config.exposure); // Manual mode, update gain
  else
    ladybugSetAbsPropertyEx(context_, LADYBUG_AUTO_EXPOSURE, false, false, false, config_.exposure); // Param is off

  // Shutter
  if (config.shutter_state > 1) // Check if we are in auto mode
    ladybugSetAbsPropertyEx(context_,
                            LADYBUG_SHUTTER,
                            false,
                            true,
                            true,
                            config_.shutter); // Auto mode, so don't update gain
  else if (config.shutter_state > 0)
    ladybugSetAbsPropertyEx(context_, LADYBUG_SHUTTER, false, true, false, config.shutter); // Manual mode, update gain
  else
    ladybugSetAbsPropertyEx(context_, LADYBUG_SHUTTER, false, false, false, config_.shutter); // Param is off

  // Frame Rate
  if (config.frame_rate_enable > 1) // Check if we are in auto mode
    ladybugSetAbsPropertyEx(context_,
                            LADYBUG_FRAME_RATE,
                            false,
                            true,
                            true,
                            config_.frame_rate); // Auto mode, so don't update gain
  else if (config.frame_rate_enable > 0)
    ladybugSetAbsPropertyEx(context_,
                            LADYBUG_FRAME_RATE,
                            false,
                            true,
                            false,
                            config.frame_rate); // Manual mode, update gain
  else
    ladybugSetAbsPropertyEx(context_, LADYBUG_FRAME_RATE, false, false, false, config_.frame_rate); // Param is off

  // Brightness
  ladybugSetAbsPropertyEx(context_,
                          LADYBUG_BRIGHTNESS,
                          false,
                          false,
                          false,
                          config_.brightness); // Auto mode, so don't update gain

  // Gamma
  if (config.gamma_state > 0) // Check if we are in auto mode
    ladybugSetAbsPropertyEx(context_, LADYBUG_GAMMA, false, true, false, config.gamma); // Config on, update gamma
  else
    ladybugSetAbsPropertyEx(context_,
                            LADYBUG_GAMMA,
                            false,
                            true,
                            false,
                            config_.gamma); // Config off, don't update gamma


  //SetGain(config.gain);
  //SetAutoExposure(config.exposure);
  //SetWhiteBalance(config.)
  //SetShutter(config.shutter);
  //SetFrameRate(config.frame_rate);
  //SetBrightness(config.brightness);
  //SetGamma(config.gamma);


  GetAutoExposure();
  //SetWhiteBalance(config.)
  GetShutter();
  GetFrameRate();
  GetBrightness();
  GetGamma();
  GetGain();
/*   if (level >= LEVEL_RECONFIGURE_STOP)
  {
    ROS_DEBUG("SpinnakerCamera::setNewConfiguration: Reconfigure Stop.");
    bool capture_was_running = capture_running_;
    start();  // For some reason some params only work after aquisition has be started once.
    stop();
    camera_->setNewConfiguration(config, level);
    if (capture_was_running)
      start();
  }
  else
  {
    camera_->SetNewConfiguration(config, level);
  } */
}  // end SetNewConfiguration


void LadybugInterface::SetGain(const float &gain) {
  ladybugSetAbsPropertyEx(context_, LADYBUG_GAIN, false, true, true, gain);
}

void LadybugInterface::SetAutoExposure(const float &exposure) {
  ladybugSetAbsProperty(context_, LADYBUG_AUTO_EXPOSURE, exposure);
}

void LadybugInterface::SetWhiteBalance(const float &white_balance) {
  ladybugSetAbsProperty(context_, LADYBUG_WHITE_BALANCE, white_balance);
}

void LadybugInterface::SetShutter(const float &shutter) {
  ladybugSetAbsProperty(context_, LADYBUG_SHUTTER, shutter);
}

void LadybugInterface::SetFrameRate(const float &frame_rate) {
  ladybugSetAbsProperty(context_, LADYBUG_FRAME_RATE, frame_rate);
}

void LadybugInterface::SetBrightness(const float &brightness) {
  ladybugSetAbsProperty(context_, LADYBUG_BRIGHTNESS, brightness);
}

void LadybugInterface::SetGamma(const float &gamma) {
  ladybugSetAbsProperty(context_, LADYBUG_GAMMA, gamma);
}

void LadybugInterface::GetGain() {
  float read_gain;
  bool in_one_push_mode, in_auto_mode, is_configurable;
  ladybugGetAbsPropertyEx(context_, LADYBUG_GAIN, &in_one_push_mode, &is_configurable, &in_auto_mode, &read_gain);
  std::cout << "\n[Ladybug Gain] Configurable: " << is_configurable <<
            ", Auto Mode: " << in_auto_mode <<
            ", Current Value: " << read_gain << std::endl;
  config_.gain = read_gain;

  if (in_auto_mode)
    config_.gain_state = 2;
  else if (is_configurable)
    config_.gain_state = 1;
  else
    config_.gain_state = 0;

  std::cout << "Gain State = " << config_.gain_state << ",  Value = " << config_.gain << std::endl;
}

void LadybugInterface::GetAutoExposure() {
  float read_exposure;
  ladybugGetAbsProperty(context_, LADYBUG_AUTO_EXPOSURE, &read_exposure);

  bool in_one_push_mode, in_auto_mode, is_configurable;
  ladybugGetAbsPropertyEx(context_,
                          LADYBUG_AUTO_EXPOSURE,
                          &in_one_push_mode,
                          &is_configurable,
                          &in_auto_mode,
                          &read_exposure);
  std::cout << "[Ladybug Exposure] Configurable: " << is_configurable <<
            ", Auto Mode: " << in_auto_mode <<
            ", Current Value: " << read_exposure << std::endl;
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
  ladybugGetAbsPropertyEx(context_,
                          LADYBUG_WHITE_BALANCE,
                          &in_one_push_mode,
                          &is_configurable,
                          &in_auto_mode,
                          &read_white_balance);
  std::cout << "[Ladybug White Balance] Configurable: " << is_configurable <<
            ", Auto Mode: " << in_auto_mode <<
            ", Current Value: " << read_white_balance << std::endl;
  //config_.white_balance = read_white_balance;
}

void LadybugInterface::GetShutter() {
  float read_shutter;
  ladybugGetAbsProperty(context_, LADYBUG_SHUTTER, &read_shutter);

  bool in_one_push_mode, in_auto_mode, is_configurable;
  ladybugGetAbsPropertyEx(context_, LADYBUG_SHUTTER, &in_one_push_mode, &is_configurable, &in_auto_mode, &read_shutter);
  std::cout << "[Ladybug Shutter Speed] Configurable: " << is_configurable <<
            ", Auto Mode: " << in_auto_mode <<
            ", Current Value: " << read_shutter << std::endl;
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
  ladybugGetAbsPropertyEx(context_,
                          LADYBUG_FRAME_RATE,
                          &in_one_push_mode,
                          &is_configurable,
                          &in_auto_mode,
                          &read_frame_rate);
  std::cout << "[Ladybug Frame Rate] Configurable: " << is_configurable <<
            ", Auto Mode: " << in_auto_mode <<
            ", Current Value: " << read_frame_rate << std::endl;
  config_.frame_rate = read_frame_rate;

}

void LadybugInterface::GetBrightness() {
  float read_brightness;
  ladybugGetAbsProperty(context_, LADYBUG_BRIGHTNESS, &read_brightness);

  bool in_one_push_mode, in_auto_mode, is_configurable;
  ladybugGetAbsPropertyEx(context_,
                          LADYBUG_BRIGHTNESS,
                          &in_one_push_mode,
                          &is_configurable,
                          &in_auto_mode,
                          &read_brightness);
  std::cout << "[Ladybug Brightness] Configurable: " << is_configurable <<
            ", Auto Mode: " << in_auto_mode <<
            ", Current Value: " << read_brightness << std::endl;
  config_.brightness = read_brightness;
}

void LadybugInterface::GetGamma() {
  float read_gamma;
  ladybugGetAbsProperty(context_, LADYBUG_GAMMA, &read_gamma);
  bool in_one_push_mode, in_auto_mode, is_configurable;
  ladybugGetAbsPropertyEx(context_, LADYBUG_GAMMA, &in_one_push_mode, &is_configurable, &in_auto_mode, &read_gamma);
  config_.gamma = read_gamma;
  std::cout << "[Ladybug Gamma] Configurable: " << is_configurable <<
            ", Auto Mode: " << in_auto_mode <<
            ", Current Value: " << read_gamma << std::endl;
  config_.gamma_state = is_configurable;
}