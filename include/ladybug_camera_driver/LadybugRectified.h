#pragma once

#include "ladybug_camera_driver/LadybugInterface.h"

// OpenCV
#include <opencv2/core/core.hpp>

/**
 * @brief Implementation of LadybugInterface for using camera in rectified mode
 * (i.e., publishing colored & rectified images)
 */
class LadybugRectified : public LadybugInterface {
public:
  ~LadybugRectified() = default;

  void Initialize() override;

  void GrabImage(ladybug_msgs::LadybugTilesPtr& tiles,
                 const std::string& frame_id) override;
};