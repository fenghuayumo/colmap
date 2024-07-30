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