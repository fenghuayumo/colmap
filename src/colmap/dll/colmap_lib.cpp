#include "colmap/dll/colmap_lib.h"

#include "sparse_reconstruct_controller.h"
#include "colmap/util/controller_thread.h"
#include "colmap/controllers/incremental_mapper.h"

// std::unique_ptr<colmap::SparseReconstructionController>  controller_;

auto ColmapSparseReconstruct::run() ->bool
{
   colmap::SparseReconstructionController::Options _option;
    _option.image_path = option.image_path;
    _option.workspace_path = option.workspace_path;
    if(option.quality == Quality::Low){
        _option.quality = colmap::SparseReconstructionController::Quality::LOW;
    }
    else if (option.quality == Quality::Medium) {
        _option.quality = colmap::SparseReconstructionController::Quality::MEDIUM;
    }else
        _option.quality = colmap::SparseReconstructionController::Quality::HIGH;
    _option.use_gpu = option.use_gpu;
    _option.gpu_index = std::to_string(option.gpu_index);
    _option.camera_model = option.camera_model;
    _option.use_hierachy = option.use_hierachy;
    _option.use_glomapper = option.use_glomap;
    _option.output_sparse_points = option.output_sparse_points;
    _option.data_type = option.video ? colmap::SparseReconstructionController::DataType::VIDEO :
        colmap::SparseReconstructionController::DataType::INDIVIDUAL;
    std::shared_ptr<colmap::ReconstructionManager> reconstruction_manager_ =
        std::make_shared<colmap::ReconstructionManager>();

    controller_ = std::make_shared<colmap::SparseReconstructionController>(
        _option, reconstruction_manager_);
    try {
        controller_->Start();
        controller_->Wait();
    }
    catch (...){
        std::cout << "colmap handle throw a exception !!\n";
        return false;
    }
    if(controller_->GetSparseReconstructPhase() != 4) return false;
    return true;
}

ColmapSparseReconstruct::~ColmapSparseReconstruct()
{
}

auto ColmapSparseReconstruct::getPoints3D(int id) const
-> std::vector<colmap::SparsePoint> {
  std::vector<colmap::SparsePoint> points;
  if (controller_) {
      if (option.use_glomap) {
        for (const auto& [_, p] : controller_->tracks) {
          colmap::SparsePoint pt;
          pt.xyz = colmap::vec3<float>{(float)p.xyz(0), (float)p.xyz(1), (float)p.xyz(2)};
          pt.color =
              colmap::vec4<unsigned char>{p.color(0), p.color(1), p.color(2),0};
          points.push_back(pt);
        }
      }
      else if (controller_->NumReconstructions() >= 1) {
        const auto& pts = controller_->Points3D(id);
        for (const auto& [_, p] : pts) {
          colmap::SparsePoint pt;
          pt.xyz = colmap::vec3<float>{(float)p.xyz(0), (float)p.xyz(1), (float)p.xyz(2)};
          pt.color =
              colmap::vec4<unsigned char>{p.color(0), p.color(1), p.color(2),0};
          points.push_back(pt);
        }
      }
  }
  return points;
}


auto ColmapSparseReconstruct::getCameraTracks(int id) const
-> std::vector<colmap::CameraTrack> {
  std::vector<colmap::CameraTrack> cameras;
  if (controller_) {
  
      if (option.use_glomap) {
        for (const auto& [_, c] : controller_->cameras) {
          cameras.push_back(
              {c.camera_id, (int)c.model_id, c.width, c.height, c.params});
        }
      }
      else if (controller_->NumReconstructions() >= 1) {
        const auto& cams = controller_->Cameras(id);
        for (const auto& [_, c] : cams) {
          cameras.push_back(
              {c.camera_id, (int)c.model_id, c.width, c.height, c.params});
        }
      }
  }
  return cameras;
}

auto ColmapSparseReconstruct::getImageTracks(int id) const
    -> std::vector<colmap::ImageTrack> {
    std::vector<colmap::ImageTrack> imgIds;
  if (controller_) {
      if (option.use_glomap) {
            for (const auto& [_, img] : controller_->images) {
            imgIds.push_back({img.image_id, img.file_name, img.camera_id});
            const auto& camfromWorld = img.cam_from_world;
            float rotx = camfromWorld.rotation.x();
            float roty = camfromWorld.rotation.y();
            float rotz = camfromWorld.rotation.z();
            float rotw = camfromWorld.rotation.w();
            float tx = camfromWorld.translation.x();
            float ty = camfromWorld.translation.y();
            float tz = camfromWorld.translation.z();
            imgIds.back().rotation = {rotx, roty,rotz,rotw};
            imgIds.back().translation = {tx,ty,tz};
          }
        }
        else if (controller_->NumReconstructions() >= 1) {
          const auto& imgs = controller_->Images(id);
          for (const auto& [_, img] : imgs) {
            imgIds.push_back({img.ImageId(), img.Name(), img.CameraId()});
            const auto& camfromWorld = img.CamFromWorld();
            float rotx = camfromWorld.rotation.x();
            float roty = camfromWorld.rotation.y();
            float rotz = camfromWorld.rotation.z();
            float rotw = camfromWorld.rotation.w();
            float tx = camfromWorld.translation.x();
            float ty = camfromWorld.translation.y();
            float tz = camfromWorld.translation.z();
            imgIds.back().rotation = {rotx, roty,rotz,rotw};
            imgIds.back().translation = {tx,ty,tz};
          }
        }
    }
    return imgIds;
}

int ColmapSparseReconstruct::GetSparseReconstructPhase()
{
  if( controller_ == nullptr) return 0;
  return controller_->GetSparseReconstructPhase();
}

float ColmapSparseReconstruct::GetProgressOnCurrentPhase()
{
  if (controller_ == nullptr) return 0;
  return controller_->GetProgressOnCurrentPhase();
}