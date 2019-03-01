#include "ladybug_camera_driver/LadybugRectified.h"

void LadybugRectified::Initialize() {
  ROS_DEBUG("[LadybugRectified] Initializing camera...");

  COLOR_PROCESSING_METHOD = LadybugColorProcessingMethod::LADYBUG_EDGE_SENSING;
  PROCESSED_PIXEL_FORMAT = LadybugPixelFormat::LADYBUG_BGR;
  LADYBUG_DATA_FORMAT = LadybugDataFormat::LADYBUG_DATAFORMAT_RAW8;

  // Set color processing method
  error_ = ladybugSetColorProcessingMethod(context_, COLOR_PROCESSING_METHOD);
  ROS_DEBUG("ladybugSetColorProcessingMethod: %s",
            ladybugErrorToString(error_));

  // Configure output images
  error_ = ladybugConfigureOutputImages(
      context_, LadybugOutputImage::LADYBUG_ALL_RECTIFIED_IMAGES);
  ROS_DEBUG("ladybugConfigureOutputImages: %s", ladybugErrorToString(error_));

  // Set offscreen image size (for rendering)
  error_ = ladybugSetOffScreenImageSize(
      context_, LadybugOutputImage::LADYBUG_ALL_RECTIFIED_IMAGES, 2464, 2048);
  ROS_DEBUG("ladybugSetOffScreenImageSize: %s", ladybugErrorToString(error_));

  // Set frame rate
  error_ = ladybugSetAbsProperty(context_, LadybugProperty::LADYBUG_FRAME_RATE,
                                 60.0f);
  ROS_DEBUG("ladybugSetAbsProperty (Frame Rate): %s",
            ladybugErrorToString(error_));
}

void LadybugRectified::GrabImage(ladybug_msgs::LadybugTilesPtr& tiles,
                                 const std::string& frame_id) {
  boost::mutex::scoped_lock scopedLock(mutex_);

  if (capture_running_) {
    // Grab image from camera, store in image_
    error_ = ladybugGrabImage(context_, &image_);
    ROS_DEBUG_THROTTLE(10, "ladybugGrabImage: %s",
                       ladybugErrorToString(error_));

    // Convert the image to 6 RGB buffers
    error_ = ladybugConvertImage(context_, &image_, nullptr);
    ROS_DEBUG_THROTTLE(10, "ladybugConvertImage: %s",
                       ladybugErrorToString(error_));

    // Send the RGB buffers to the graphics card for rendering
    error_ = ladybugUpdateTextures(context_, LADYBUG_NUM_CAMERAS, nullptr);
    ROS_DEBUG_THROTTLE(10, "ladybugUpdateTextures: %s",
                       ladybugErrorToString(error_));

    std::vector<LadybugOutputImage> ladybug_output_images = {
        LADYBUG_RECTIFIED_CAM0, LADYBUG_RECTIFIED_CAM1, LADYBUG_RECTIFIED_CAM2,
        LADYBUG_RECTIFIED_CAM3, LADYBUG_RECTIFIED_CAM4, LADYBUG_RECTIFIED_CAM5};

    // Set msg header information
    tiles->header.stamp.sec = image_.timeStamp.ulSeconds;
    tiles->header.stamp.nsec = image_.timeStamp.ulMicroSeconds;
    tiles->header.frame_id = frame_id;
    tiles->header.seq = image_.imageInfo.ulSequenceId;

    for (auto cam = 0; cam < LADYBUG_NUM_CAMERAS; ++cam) {
      LadybugProcessedImage rectified_image;

      // Perform offscreen rendering
      error_ =
          ladybugRenderOffScreenImage(context_, ladybug_output_images[cam],
                                      PROCESSED_PIXEL_FORMAT, &rectified_image);
      ROS_DEBUG_THROTTLE(10, "ladybugRenderOffScreenImage: %s",
                         ladybugErrorToString(error_));

      sensor_msgs::Image tile;

      fillImage(tile, PROCESSED_MSG_ENCODING, image_rows_, image_cols_,
                image_cols_ * bytes_per_processed_pixel_,
                rectified_image.pData);

      // Rotate image in OpenCV
      cv::Size cv_img_dims;
      cv_img_dims.width = 2464;
      cv_img_dims.height = 2048;
      cv::Mat image(cv_img_dims, CV_8UC3,
                    &(tile.data[0])); // Create image container
      cv::transpose(image, image);    // Transpose image
      cv::flip(image, image, 1);      // Flip image

      sensor_msgs::Image flipped_tile;
      flipped_tile.header = tiles->header;
      std::string frame = frame_id + std::string("_") + std::to_string(cam);
      flipped_tile.header.frame_id = frame;
      flipped_tile.height = tile.width;
      flipped_tile.width = tile.height;
      flipped_tile.encoding =
          PROCESSED_MSG_ENCODING; // sensor_msgs::image_encodings::BGR8;
      flipped_tile.step = flipped_tile.width * bytes_per_processed_pixel_;
      flipped_tile.data.resize(flipped_tile.height * flipped_tile.step);

      // Copy rotated image into sensor_msgs::image container
      memcpy(&(flipped_tile.data[0]), image.data,
             bytes_per_processed_pixel_ * image_rows_ * image_cols_);

      tiles->images.push_back(flipped_tile);
    }

    // Populate LadybugTiles message
    for (unsigned int cam = 0; cam < LADYBUG_NUM_CAMERAS; ++cam) {}
  } else if (camera_connected_) {
    throw CameraNotRunningException(
        "LadybugRectified::grabImage: Camera is currently not running.  Please "
        "start the capture.");
  } else {
    throw std::runtime_error("LadybugRectified not connected!");
  }
  ROS_DEBUG_THROTTLE(10, "Finished grabbing image");
}
