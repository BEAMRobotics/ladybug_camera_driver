#pragma once

#include "ladybug_camera_driver/LadybugInterface.h"

/**
 * @brief Implementation of LadybugInterface for using camera in raw mode (i.e.,
 * publishing raw bayer tiles)
 */
class LadybugRaw : public LadybugInterface {
public:
  ~LadybugRaw() = default;

  void Initialize() override;

  void GrabImage(ladybug_msgs::LadybugTilesPtr& tiles,
                 const std::string& frame_id) override;
};
