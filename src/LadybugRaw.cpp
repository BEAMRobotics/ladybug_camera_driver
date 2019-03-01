#include "ladybug_camera_driver/LadybugRaw.h"

LadybugRaw::LadybugRaw() {
  capture_running_ = false;
  camera_connected_ = false;
}

LadybugRaw::~LadybugRaw() {}

void LadybugRaw::Initialize() {

  ROS_DEBUG("[ladybugRaw] Initializing camera");
  // Set color processing method
  error_ = ::ladybugSetColorProcessingMethod(
      context_, LadybugColorProcessingMethod::LADYBUG_DOWNSAMPLE4);

  // Make the rendering engine use the alpha mask
  error_ = ladybugSetAlphaMasking(context_, false);

  // Set frame rate
  ladybug_frame_rate_ = 60.0f;
  error_ =
      ladybugSetAbsProperty(context_, LADYBUG_FRAME_RATE, ladybug_frame_rate_);

  // error_ = ::ladybugSetGrabTimeout(context_, 75);
  // std::cout << "[ladybugSetGrabTimeout] Done: " <<
  // ladybugErrorToString(error_) << std::endl;
}

void LadybugRaw::GrabImage(ladybug_msgs::LadybugTilesPtr& tiles,
                              const std::string& frame_id) {
  ROS_DEBUG("Received GrabImage request");
  boost::mutex::scoped_lock scopedLock(mutex_);
  ROS_DEBUG("Received GrabImage request2");
  ROS_DEBUG("capture_running_ = %d", capture_running_);

  if (capture_running_) {

    ROS_DEBUG("grabbing image from camera");
    // Grab image from camera, store in image_
    LadybugImage image;
    error_ = ::ladybugGrabImage(context_, &image);
    ROS_DEBUG("grabbing image from camera");

    // Set msg header information
    tiles->header.stamp.sec = image_.timeStamp.ulSeconds;
    tiles->header.stamp.nsec = image_.timeStamp.ulMicroSeconds;
    tiles->header.frame_id = frame_id;
    tiles->header.seq = image_.imageInfo.ulSequenceId;
    ROS_DEBUG("LADYBUG_NUM_CAMERAS: %d", LADYBUG_NUM_CAMERAS);

    // Populate LadybugTiles message
    for (unsigned int cam = 0; cam < LADYBUG_NUM_CAMERAS; cam++) {
      ROS_DEBUG("Grabbing image: %d", cam);
      sensor_msgs::Image tile;
      std::ostringstream s;
      s << frame_id << "_" << cam;
      tile.header.frame_id = s.str();
      fillImage(tile, raw_image_encoding_, image_rows_, image_cols_,
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
  ROS_DEBUG("Finished grabbing image");

}
