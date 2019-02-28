#include "ladybug_camera_driver/LadybugRectified.h"

LadybugRectified::LadybugRectified() {
  capture_running_ = false;
  camera_connected_ = false;
}

LadybugRectified::~LadybugRectified() {}

void LadybugRectified::Initialize() {
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


void LadybugRectified::GrabImage(ladybug_msgs::LadybugTilesPtr &tiles, const std::string &frame_id) {

  boost::mutex::scoped_lock scopedLock(mutex_);

  if (capture_running_) {
    std::cout << "Test" << std::endl;
//    error_ = ladybugSetOffScreenImageSize(context_, LADYBUG_RECTIFIED_CAM0, 512, 384);
//    std::cout << "[ladybugSetOffScreenImageSize] Done: " << ladybugErrorToString(error_) << std::endl;
//
//    error_ = ladybugSetColorProcessingMethod(context_, LADYBUG_EDGE_SENSING);
//    std::cout << "[ladybugSetColorProcessingMethod] Done: " << ladybugErrorToString(error_) << std::endl;
    unsigned int rows, cols;
    error_ = ladybugGetOffScreenImageSize(context_, LADYBUG_RECTIFIED_CAM0, &rows, &cols);
    std::cout << "Rectified images = " << rows << "Rectified rows = " << cols << std::endl;

    error_ = ladybugConfigureOutputImages(context_, LADYBUG_ALL_RECTIFIED_IMAGES);
    std::cout << "[ladybugConfigureOutputImages] Done: " << ladybugErrorToString(error_) << std::endl;

    // Grab image from camera, store in image_
    error_ = ::ladybugGrabImage(context_, &image_);
    std::cout << "[ladybugGrabImage] Done: " << ladybugErrorToString(error_) << std::endl;
    // Convert the image to 6 RGB buffers
    error_ = ladybugConvertImage(context_, &image_, NULL);
    std::cout << "[ladybugConvertImage] Done: " << ladybugErrorToString(error_) << std::endl;
    // Send the RGB buffers to the graphics card
    error_ = ladybugUpdateTextures(context_, LADYBUG_NUM_CAMERAS, NULL);
    std::cout << "[ladybugUpdateTextures] Done: " << ladybugErrorToString(error_) << std::endl;

    std::vector<LadybugOutputImage> ladybug_output_images = {LADYBUG_RECTIFIED_CAM0, LADYBUG_RECTIFIED_CAM1,
                                                             LADYBUG_RECTIFIED_CAM2, LADYBUG_RECTIFIED_CAM3,
                                                             LADYBUG_RECTIFIED_CAM4, LADYBUG_RECTIFIED_CAM5};


    // Set msg header information
    tiles->header.stamp.sec = image_.timeStamp.ulSeconds;
    tiles->header.stamp.nsec = image_.timeStamp.ulMicroSeconds;
    tiles->header.frame_id = frame_id;
    tiles->header.seq = image_.imageInfo.ulSequenceId;

    raw_image_encoding_ = sensor_msgs::image_encodings::BGR8;
    bytes_per_raw_pixel_ = 3;
    std::cout << "Bytes per raw pixel = " << bytes_per_raw_pixel_ << std::endl;
    std::cout << "Image_rows_ = " << image_rows_ << ", image_cols_ = " << image_cols_ << std::endl;

    for (auto cam = 0; cam < LADYBUG_NUM_CAMERAS; ++cam){
      std::cout << "Test" << std::endl;
      error_ = ladybugGetOffScreenImageSize(context_, ladybug_output_images[cam], &rows, &cols);
      std::cout << "Rectified images = " << rows << "Rectified rows = " << cols << std::endl;

      LadybugProcessedImage rectified_image;
      error_ = ladybugRenderOffScreenImage(context_, ladybug_output_images[cam], LADYBUG_BGR, &rectified_image);
      std::cout << "[ladybugRenderOffScreenImage] Done: " << ladybugErrorToString(error_) << std::endl;

      sensor_msgs::Image tile;
      std::ostringstream s;
      s << frame_id << "_" << cam;
      tile.header.frame_id = s.str();
      std::cout << "Test2" << std::endl;
      fillImage(tile,
                raw_image_encoding_,
                image_rows_,
                image_cols_,
                image_cols_ * bytes_per_raw_pixel_,
                rectified_image.pData);
      // Rotate image in OpenCV
      cv::Size size_;
      size_.width = 2464;
      size_.height = 2048;
      std::cout << "Test3" << std::endl;
      cv::Mat image(size_, CV_8UC3, &(tile.data[0])); //Create image container
      cv::transpose(image, image); //Transpose image
      cv::flip(image, image, 1); //Flip image

      sensor_msgs::Image test_img;
      test_img.height = tile.width;
      test_img.width = tile.height;
      test_img.encoding = sensor_msgs::image_encodings::BGR8;
      test_img.step = test_img.width * 3;
      test_img.data.resize(test_img.height * test_img.step);
      // Copy rotated image into sensor_msgs::image container
      memcpy(&(test_img.data[0]), image.data, bytes_per_raw_pixel_*image_rows_*image_cols_);
      std::cout << "Test4" << std::endl;
      tiles->images.push_back(test_img);

    }

    error_ = ladybugGetOffScreenImageSize(context_, LADYBUG_RECTIFIED_CAM0, &rows, &cols);
//    std::cout << "Rectified images = " << rows << "Rectified rows = " << cols << std::endl;
//    std::cout << "Rectified images = " << rectified_images.uiCols << "Rectified rows = " << rectified_images.uiRows << std::endl;
//    std::string output_path = "/home/steve/test.jpg";
//    error_ = ladybugSaveImage( context_, &rectified_images, output_path.c_str(), LADYBUG_FILEFORMAT_JPG );

    // Populate LadybugTiles message
    for (unsigned int cam = 0; cam < LADYBUG_NUM_CAMERAS; cam++) {

    }
  } else if (camera_connected_) {
    throw CameraNotRunningException(
        "LadybugRectified::grabImage: Camera is currently not running.  Please start the capture.");
  } else {
    throw std::runtime_error("LadybugRectified not connected!");
  }
}
