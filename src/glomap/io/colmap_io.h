#pragma once

#include "glomap/io/colmap_converter.h"
#include "glomap/scene/types_sfm.h"
namespace colmap {
class ReconstructionManager;
}
namespace glomap {

void ConvertGlomapToColmapReconstructionManager(
    std::shared_ptr<colmap::ReconstructionManager>& reconstruction_manager,
    const std::unordered_map<camera_t, Camera>& cameras,
    const std::unordered_map<image_t, Image>& images,
    const std::unordered_map<track_t, Track>& tracks,
    const std::string image_path = "");
void WriteGlomapReconstruction(
    const std::string& reconstruction_path,
    const std::unordered_map<camera_t, Camera>& cameras,
    const std::unordered_map<image_t, Image>& images,
    const std::unordered_map<track_t, Track>& tracks,
    const std::string output_format = "bin",
    const std::string image_path = "");

void WriteColmapReconstruction(const std::string& reconstruction_path,
                               const colmap::Reconstruction& reconstruction,
                               const std::string output_format = "bin");

}  // namespace glomap
