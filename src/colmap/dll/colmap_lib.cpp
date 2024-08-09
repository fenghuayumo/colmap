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
    if( controller_->NumReconstructions() >= 1){
        const auto& pts = controller_->Points3D(0);
        const auto& cams = controller_->Cameras(0);
        for(const auto& [_,p] : pts){
          colmap::SparsePoint pt;
          pt.xyz =  colmap::vec3<double>{p.xyz.x(), p.xyz.y(), p.xyz.z()};
          pt.color = colmap::vec3<unsigned char>{p.color.x(), p.color.y(), p.color.z()};
          points.push_back(pt);
        }
        for(const auto& [_,c] : cams){
          cameras.push_back(
              {c.camera_id, (int)c.model_id, c.width, c.height, c.params});
        }
    }
    return true;
}

ColmapSparseReconstruct::~ColmapSparseReconstruct()
{
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