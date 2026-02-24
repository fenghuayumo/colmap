// Copyright (c), ETH Zurich and UNC Chapel Hill.
// Minimal header for RigConfig to break circular include dependencies.
// rig.h -> database.h -> two_view_geometry.h -> rig.h (circular)

#pragma once

#include "colmap/geometry/rigid3.h"
#include "colmap/scene/camera.h"

#include <filesystem>
#include <optional>
#include <vector>

namespace colmap {

struct RigConfig {
  struct RigCamera {
    bool ref_sensor = false;
    std::string image_prefix;
    std::optional<Rigid3d> cam_from_rig;
    std::optional<Camera> camera;
  };
  std::vector<RigCamera> cameras;
};

}  // namespace colmap
