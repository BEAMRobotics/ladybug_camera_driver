#pragma once

#include <iostream>
#include <mutex>
#include <sstream>

// ROS
#include <ladybug_msgs/LadybugTiles.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h> // ROS message header for Image
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h> // ROS header for the different supported image encoding types

#include "ladybug_camera_driver/LadybugConfig.h"
#include "ladybug_camera_driver/camera_exceptions.h"

// LadybugSDK from Point Grey / FLIR
#include <ladybug/ladybug.h>
#include <ladybug/ladybuggeom.h>
#include <ladybug/ladybugrenderer.h>
#include <ladybug/ladybugstream.h>

class LadybugInterface {
public:
  LadybugInterface();

  ~LadybugInterface() = default;

  virtual void Connect();
  virtual void Initialize() = 0;
  virtual void Disconnect();
  virtual void Start();
  virtual bool Stop();
  virtual void GrabImage(ladybug_msgs::LadybugTilesPtr& tiles,
                         const std::string& frame_id);
  virtual void grabImageJpeg(std::vector<sensor_msgs::Image::Ptr>& images,
                             const std::string& frame_id);

  virtual void
      SetNewConfiguration(const ladybug_camera_driver::LadybugConfig& config);
  virtual void SetDataCaptureFormat(const std::string& datatype);
  virtual void
      SetProcessedPixelFormat(const std::string& processed_pixel_format);
  virtual void
      SetColorProcessingMethod(const std::string& color_processing_method);

  static const uint8_t LEVEL_RECONFIGURE_CLOSE = 3;
  static const uint8_t LEVEL_RECONFIGURE_STOP = 1;
  static const uint8_t LEVEL_RECONFIGURE_RUNNING = 0;
  bool collect_raw_;
  bool camera_connected_;
  ladybug_camera_driver::LadybugConfig config_;

  virtual uint32_t getSerial() { return serial_; }

  uint32_t serial_;
  virtual void GetPixelByteSize();
  virtual void GetImageEncodings();
  virtual void GetImageSize();
  virtual void OutputCamInfo();

  // Dynamic reconfigure setters
  virtual void SetAutoExposure(const float& exposure);
  virtual void SetWhiteBalance(const float& white_balance);
  virtual void SetShutter(const float& shutter);
  virtual void SetFrameRate(const float& frame_rate);
  virtual void SetGain(const float& gain);
  virtual void SetGamma(const float& gamma);
  virtual void SetBrightness(const float& brightness);

  // Dynamic reconfigure getters
  virtual void GetAutoExposure();
  virtual void GetWhiteBalance();
  virtual void GetShutter();
  virtual void GetFrameRate();
  virtual void GetGain();
  virtual void GetGamma();
  virtual void GetBrightness();

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

  unsigned char* arp_buffers_[6] = {0};
  float ladybug_frame_rate_;

  std::string raw_image_encoding_;
  std::string processed_image_encoding_;

  size_t bytes_per_raw_pixel_;
  size_t bytes_per_processed_pixel_;

  boost::mutex mutex_; ///< A mutex to make sure that we don't try to grabImages
                       ///< while reconfiguring or vice versa.  Implemented with
                       ///< boost::mutex::scoped_lock.
  volatile bool
      capture_running_; ///< A status boolean that checks if the camera has been
                        ///< started and is loading images into its buffer.ù
};
