// Copyright (c), ETH Zurich and UNC Chapel Hill.
// ReadRigConfig and ApplyRigConfig implementation in separate file to avoid
// include order issues with rig.cc

#include "colmap/scene/rig_config.h"
#include "colmap/scene/rig.h"

#include "colmap/scene/database.h"
#include "colmap/scene/reconstruction.h"
#include "colmap/sensor/models.h"
#include "colmap/util/logging.h"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace colmap {

std::vector<RigConfig> ReadRigConfig(
    const std::filesystem::path& rig_config_path) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(rig_config_path.string().c_str(), pt);

  std::vector<RigConfig> configs;
  for (const auto& rig_node : pt) {
    RigConfig& config = configs.emplace_back();
    bool has_ref_sensor = false;
    for (const auto& camera : rig_node.second.get_child("cameras")) {
      RigConfig::RigCamera& config_camera = config.cameras.emplace_back();

      config_camera.image_prefix =
          camera.second.get<std::string>("image_prefix");

      auto cam_from_rig_rotation_node =
          camera.second.get_child_optional("cam_from_rig_rotation");
      auto cam_from_rig_translation_node =
          camera.second.get_child_optional("cam_from_rig_translation");
      if (cam_from_rig_rotation_node && cam_from_rig_translation_node) {
        Rigid3d cam_from_rig;

        int index = 0;
        Eigen::Vector4d cam_from_rig_wxyz;
        for (const auto& node : cam_from_rig_rotation_node.get()) {
          cam_from_rig_wxyz[index++] = node.second.get_value<double>();
        }
        cam_from_rig.rotation() = Eigen::Quaterniond(cam_from_rig_wxyz(0),
                                                     cam_from_rig_wxyz(1),
                                                     cam_from_rig_wxyz(2),
                                                     cam_from_rig_wxyz(3));

        THROW_CHECK(cam_from_rig_translation_node);
        index = 0;
        for (const auto& node : cam_from_rig_translation_node.get()) {
          cam_from_rig.translation()(index++) = node.second.get_value<double>();
        }
        config_camera.cam_from_rig = cam_from_rig;
      }

      auto ref_sensor_node = camera.second.get_child_optional("ref_sensor");
      if (ref_sensor_node && ref_sensor_node.get().get_value<bool>()) {
        THROW_CHECK(!cam_from_rig_rotation_node &&
                    !cam_from_rig_translation_node)
            << "Reference sensor must not have cam_from_rig";
        THROW_CHECK(!has_ref_sensor)
            << "Rig must only have one reference sensor";
        config_camera.ref_sensor = true;
        has_ref_sensor = true;
      }

      auto camera_model_name_node =
          camera.second.get_child_optional("camera_model_name");
      auto camera_params_node =
          camera.second.get_child_optional("camera_params");
      if (camera_model_name_node && camera_params_node) {
        config_camera.camera = std::make_optional<Camera>();
        config_camera.camera->model_id = CameraModelNameToId(
            camera.second.get<std::string>("camera_model_name"));
        config_camera.camera->has_prior_focal_length = true;
        for (const auto& node : camera_params_node.get()) {
          config_camera.camera->params.push_back(
              node.second.get_value<double>());
        }
      }
    }

    THROW_CHECK(has_ref_sensor) << "Rig must have one reference sensor";
  }

  return configs;
}

void ApplyRigConfig(const std::vector<RigConfig>& configs,
                    Database& database,
                    Reconstruction* reconstruction) {
  // TODO: Implement rig configuration application
  LOG(ERROR) << "ApplyRigConfig is not yet implemented";
}

}  // namespace colmap
