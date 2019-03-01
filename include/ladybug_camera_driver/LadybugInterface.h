#pragma once

// ROS
#include <ladybug_msgs/LadybugTiles.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>

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

  /**
   * @brief Connect to the camera (load config, get camera info)
   */
  virtual void Connect();

  /**
   * @brief Method for performing whatever initialization is necessary to run
   * the camera in a specific mode (e.g., raw / rectified)
   */
  virtual void Initialize() = 0;

  /**
   * @brief Disconnect from camera
   */
  virtual void Disconnect();

  /**
   * Start image capture on camera
   */
  virtual void Start();

  /**
   * Stop image capture on camera
   * @return True if stop is successful
   */
  virtual bool Stop();

  /**
   * @brief Grab the images currently available from the camera
   * @param tiles Tiles message
   * @param frame_id Frame to publish tiles on
   */
  virtual void GrabImage(ladybug_msgs::LadybugTilesPtr& tiles,
                         const std::string& frame_id) = 0;

  /**
   * @brief Method for configuring camera (used with dynamic reconfigure)
   * @param config Config message
   */
  void SetNewConfiguration(const ladybug_camera_driver::LadybugConfig& config);

  virtual uint32_t getSerial() { return serial_; }

  void GetPixelByteSize();
  void GetImageEncodings();
  void OutputCamInfo();

  // Dynamic reconfigure setters
  void SetAutoExposure(const float& exposure);
  void SetWhiteBalance(const float& white_balance);
  void SetShutter(const float& shutter);
  void SetFrameRate(const float& frame_rate);
  void SetGain(const float& gain);
  void SetGamma(const float& gamma);
  void SetBrightness(const float& brightness);

  // Dynamic reconfigure getters
  void GetAutoExposure();
  void GetWhiteBalance();
  void GetShutter();
  void GetFrameRate();
  void GetGain();
  void GetGamma();
  void GetBrightness();

  static const uint8_t LEVEL_RECONFIGURE_CLOSE = 3;
  static const uint8_t LEVEL_RECONFIGURE_STOP = 1;
  static const uint8_t LEVEL_RECONFIGURE_RUNNING = 0;

  // Ladybug variables
  LadybugContext context_;
  LadybugError error_;
  LadybugCameraInfo cam_info_;
  LadybugImage image_;
  uint32_t serial_;

  ladybug_camera_driver::LadybugConfig config_;
  bool camera_connected_;
  size_t bytes_per_raw_pixel_;
  size_t bytes_per_processed_pixel_;
  std::string IMAGE_MSG_ENCODING;
  std::string PROCESSED_MSG_ENCODING;
  LadybugDataFormat LADYBUG_DATA_FORMAT;
  LadybugPixelFormat PROCESSED_PIXEL_FORMAT;
  LadybugColorProcessingMethod COLOR_PROCESSING_METHOD;
  LadybugStippledFormat LADYBUG_BAYER_FORMAT;
  unsigned int image_cols_ = 2464;
  unsigned int image_rows_ = 2048;

  boost::mutex mutex_; ///< A mutex to make sure that we don't try to grabImages
                       ///< while reconfiguring or vice versa.  Implemented with
                       ///< boost::mutex::scoped_lock.
  volatile bool
      capture_running_; ///< A status boolean that checks if the camera has been
                        ///< started and is loading images into its buffer.ù
};
