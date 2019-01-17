#pragma once

#include <sstream>
#include <mutex>
#include <iostream>

// ROS
#include <sensor_msgs/Image.h> // ROS message header for Image
#include <sensor_msgs/image_encodings.h> // ROS header for the different supported image encoding types
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/CompressedImage.h>
#include <ladybug_msgs/LadybugTiles.h>

#include "ladybug_camera_driver/LadybugConfig.h"
#include "ladybug_camera_driver/camera_exceptions.h"

// LadybugSDK from Point Grey / FLIR
#include <ladybug/ladybug.h>
#include <ladybug/ladybugstream.h>
#include <ladybug/ladybuggeom.h>
#include <ladybug/ladybugrenderer.h>

class LadybugCamera {
 public:

  LadybugCamera();

  ~LadybugCamera();

  void connect();
  void disconnect();
  void start();
  bool stop();
  void grabImage(ladybug_msgs::LadybugTilesPtr &tiles, const std::string &frame_id);
  void grabImageJpeg(std::vector<sensor_msgs::Image::Ptr> &images, const std::string &frame_id);

  void setNewConfiguration(const ladybug_camera_driver::LadybugConfig &config);
  void SetDataCaptureFormat(const std::string &datatype);
  void SetProcessedPixelFormat(const std::string &processed_pixel_format);
  void SetColorProcessingMethod(const std::string &color_processing_method);

  static const uint8_t LEVEL_RECONFIGURE_CLOSE = 3;
  static const uint8_t LEVEL_RECONFIGURE_STOP = 1;
  static const uint8_t LEVEL_RECONFIGURE_RUNNING = 0;
  bool collect_raw_;
  bool camera_connected_;
  ladybug_camera_driver::LadybugConfig config_;

  uint32_t getSerial() {
    return serial_;
  }

 private:

  uint32_t serial_;
  void GetPixelByteSize();
  void GetImageEncodings();
  void GetImageSize();
  void OutputCamInfo();

  //Dynamic reconfigure setters
  void SetAutoExposure(const float &exposure);
  void SetWhiteBalance(const float &white_balance);
  void SetShutter(const float &shutter);
  void SetFrameRate(const float &frame_rate);
  void SetGain(const float &gain);
  void SetGamma(const float &gamma);
  void SetBrightness(const float &brightness);

  //Dynamic reconfigure getters
  void GetAutoExposure();
  void GetWhiteBalance();
  void GetShutter();
  void GetFrameRate();
  void GetGain();
  void GetGamma();
  void GetBrightness();

  /* Ladybug member variables */
  LadybugContext context_;
  LadybugError error_;
  LadybugCameraInfo cam_info_;
  LadybugImage image_;

  LadybugDataFormat ladybug_data_format_;
  LadybugPixelFormat ladybug_processed_pixel_format_;
  LadybugColorProcessingMethod ladybug_color_processing_method_;
  LadybugStippledFormat ladybug_bayer_format_;

  unsigned int image_cols_ = 2464;
  unsigned int image_rows_ = 2048;
  unsigned int ladybug_packet_size_; //= 56000;
  unsigned int ladybug_buffer_size_; //= round(8000 * packetSize / frameRate);

  unsigned char *arp_buffers_[6] = {0};
  float ladybug_frame_rate_;

  std::string raw_image_encoding_;
  std::string processed_image_encoding_;

  size_t bytes_per_raw_pixel_;
  size_t bytes_per_processed_pixel_;

  boost::mutex
      mutex_; ///< A mutex to make sure that we don't try to grabImages while reconfiguring or vice versa.  Implemented with boost::mutex::scoped_lock.
  volatile bool
      capture_running_; ///< A status boolean that checks if the camera has been started and is loading images into its buffer.ù

};

