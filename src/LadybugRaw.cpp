#include "ladybug_camera_driver/LadybugRaw.h"

void LadybugRaw::Initialize() {
  COLOR_PROCESSING_METHOD = LadybugColorProcessingMethod::LADYBUG_DOWNSAMPLE4;
  PROCESSED_PIXEL_FORMAT = LadybugPixelFormat::LADYBUG_BGRU;
  LADYBUG_DATA_FORMAT = LadybugDataFormat::LADYBUG_DATAFORMAT_RAW8;

  ROS_DEBUG("[LadybugRaw] Initializing camera...");

  // Set color processing method
  error_ = ladybugSetColorProcessingMethod(context_, COLOR_PROCESSING_METHOD);
  ROS_DEBUG("ladybugSetColorProcessingMethod: %s",
            ladybugErrorToString(error_));

  // Make the rendering engine use the alpha mask
  error_ = ladybugSetAlphaMasking(context_, false);
  ROS_DEBUG("ladybugSetAlphaMasking: %s", ladybugErrorToString(error_));

  // Set frame rate
  error_ = ladybugSetAbsProperty(context_, LadybugProperty::LADYBUG_FRAME_RATE,
                                 60.0f);
  ROS_DEBUG("ladybugSetAbsProperty (Frame Rate): %s",
            ladybugErrorToString(error_));
}

void LadybugRaw::GrabImage(ladybug_msgs::LadybugTilesPtr& tiles,
                           const std::string& frame_id) {
  boost::mutex::scoped_lock scopedLock(mutex_);

  if (capture_running_) {
    // Grab image from camera
    error_ = ladybugGrabImage(context_, &image_);
    ROS_DEBUG_THROTTLE(10, "ladybugGrabImage: %s",
                       ladybugErrorToString(error_));

    // Set msg header information
    tiles->header.stamp.sec = image_.timeStamp.ulSeconds;
    tiles->header.stamp.nsec = image_.timeStamp.ulMicroSeconds;
    tiles->header.frame_id = frame_id;
    tiles->header.seq = image_.imageInfo.ulSequenceId;

    // Populate LadybugTiles message
    for (unsigned int cam = 0; cam < LADYBUG_NUM_CAMERAS; ++cam) {
      sensor_msgs::Image tile;
      tile.header = tiles->header;
      std::string frame = frame_id + std::string("_") + std::to_string(cam);
      tile.header.frame_id = frame;

      fillImage(tile, IMAGE_MSG_ENCODING, image_rows_, image_cols_,
                image_cols_ * bytes_per_raw_pixel_,
                image_.pData + (cam * image_rows_ * image_cols_));
      tiles->images.push_back(tile);
    }
  } else if (camera_connected_) {
    throw CameraNotRunningException(
        "LadybugRaw::GrabImage: Camera is currently not running.  Please "
        "start the capture.");
  } else {
    throw std::runtime_error("LadybugRaw not connected!");
  }
  ROS_DEBUG_THROTTLE(10, "Finished grabbing image");
}
