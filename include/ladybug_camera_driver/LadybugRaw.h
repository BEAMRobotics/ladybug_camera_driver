#pragma once

#include "ladybug_camera_driver/LadybugInterface.h"

class LadybugRaw : public LadybugInterface{
 public:

  LadybugRaw();

  ~LadybugRaw();

  void Initialize();

  void GrabImage(ladybug_msgs::LadybugTilesPtr& tiles,
                 const std::string& frame_id);
};

