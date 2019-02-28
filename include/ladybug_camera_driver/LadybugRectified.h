#pragma once

#include "ladybug_camera_driver/LadybugInterface.h"

// OpenCV
#include <opencv2/core/core.hpp>

class LadybugRectified : public LadybugInterface{
 public:

  LadybugRectified();

  ~LadybugRectified();

  void Initialize();

  void GrabImage(ladybug_msgs::LadybugTilesPtr& tiles,
                 const std::string& frame_id);

};

