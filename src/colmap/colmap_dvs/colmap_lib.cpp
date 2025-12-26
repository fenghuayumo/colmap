#include "colmap_lib.h"
#include "sparse_reconstruct_controller.h"
#include "colmap/util/controller_thread.h"
#include "colmap/controllers/incremental_pipeline.h"

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
    }else if (option.quality == Quality::High) {
        _option.quality = colmap::SparseReconstructionController::Quality::HIGH;
    }else
       _option.quality = colmap::SparseReconstructionController::Quality::EXTREME;
    _option.use_gpu = option.use_gpu;
    _option.gpu_index = std::to_string(option.gpu_index);
    _option.camera_model = option.camera_model;
    _option.use_hierachy = option.use_hierachy;
    _option.use_glomapper = option.use_glomap;
    _option.output_sparse_points = option.output_sparse_points;
    _option.single_camera = option.single_camera;
    _option.data_type = option.video ? colmap::SparseReconstructionController::DataType::VIDEO :
        colmap::SparseReconstructionController::DataType::INDIVIDUAL;
    _option.mask_path = option.mask_path;
    std::shared_ptr<colmap::ReconstructionManager> reconstruction_manager_ =
        std::make_shared<colmap::ReconstructionManager>();

    controller_ = std::make_shared<colmap::SparseReconstructionController>(
        _option, reconstruction_manager_);
    try {
        controller_->Start();
        controller_->Wait();
    }
    catch (...){
        std::cout << "sfm throw a exception !!\n";
        return false;
    }
    if(controller_->GetSparseReconstructPhase() != 4) return false;
    return true;
}

ColmapSparseReconstruct::~ColmapSparseReconstruct()
{
}

// Optimized: Direct FFI array allocation (zero-copy)
auto ColmapSparseReconstruct::getPoints3DArray(int id, size_t* out_count) const -> colmap::SparsePoint* {
  std::lock_guard<std::mutex> lock(controller_mutex_);
  
  *out_count = 0;
  
  if (!controller_ || controller_->NumReconstructions() < 1) {
    return nullptr;
  }
  
  const auto& pts = controller_->Points3D(id);
  if (pts.empty()) {
    return nullptr;
  }
  
  *out_count = pts.size();
  auto* points = static_cast<colmap::SparsePoint*>(malloc(sizeof(colmap::SparsePoint) * pts.size()));
  
  if (points) {
    size_t i = 0;
    for (const auto& [_, p] : pts) {
      points[i].xyz.x = static_cast<float>(p.xyz(0));
      points[i].xyz.y = static_cast<float>(p.xyz(1));
      points[i].xyz.z = static_cast<float>(p.xyz(2));
      points[i].color.x = p.color(0);
      points[i].color.y = p.color(1);
      points[i].color.z = p.color(2);
      points[i].color.w = 0;
      ++i;
    }
  }
  
  return points;
}

auto ColmapSparseReconstruct::getCameraTracksArray(int id, size_t* out_count) const -> colmap::CameraTrack* {
  std::lock_guard<std::mutex> lock(controller_mutex_);
  
  *out_count = 0;
  
  if (!controller_ || controller_->NumReconstructions() < 1) {
    return nullptr;
  }
  
  const auto& cams = controller_->Cameras(id);
  if (cams.empty()) {
    return nullptr;
  }
  
  *out_count = cams.size();
  // ⚠️ Using malloc with C++ objects (std::vector) is DANGEROUS!
  // But we're immediately copying to FFI layer which expects malloc'd memory
  // The FFI layer MUST free this before Rust uses it
  auto* cameras = static_cast<colmap::CameraTrack*>(malloc(sizeof(colmap::CameraTrack) * cams.size()));
  
  if (cameras) {
    size_t i = 0;
    for (const auto& [_, c] : cams) {
      // Use placement new to properly construct the CameraTrack object
      new (&cameras[i]) colmap::CameraTrack();
      
      cameras[i].camera_id = c.camera_id;
      cameras[i].model_id = static_cast<int>(c.model_id);
      cameras[i].width = c.width;
      cameras[i].height = c.height;
      cameras[i].params = c.params;  // std::vector copy
      
      ++i;
    }
  }
  
  return cameras;
}

auto ColmapSparseReconstruct::getImageTracksArray(int id, size_t* out_count) const -> colmap::ImageTrack* {
  std::lock_guard<std::mutex> lock(controller_mutex_);
  
  *out_count = 0;
  
  if (!controller_ || controller_->NumReconstructions() < 1) {
    return nullptr;
  }
  
  const auto& imgs = controller_->Images(id);
  if (imgs.empty()) {
    return nullptr;
  }
  
  *out_count = imgs.size();
  // ⚠️ Using malloc with C++ objects (std::string) is DANGEROUS!
  auto* image_tracks = static_cast<colmap::ImageTrack*>(malloc(sizeof(colmap::ImageTrack) * imgs.size()));
  
  if (image_tracks) {
    size_t i = 0;
    for (const auto& [_, img] : imgs) {
      const auto& camfromWorld = img.CamFromWorld();
      
      // Use placement new to properly construct the ImageTrack object
      new (&image_tracks[i]) colmap::ImageTrack();
      
      image_tracks[i].image_id = img.ImageId();
      image_tracks[i].name = img.Name();  // std::string copy
      image_tracks[i].camera_id = img.CameraId();
      image_tracks[i].rotation.x = camfromWorld.rotation.x();
      image_tracks[i].rotation.y = camfromWorld.rotation.y();
      image_tracks[i].rotation.z = camfromWorld.rotation.z();
      image_tracks[i].rotation.w = camfromWorld.rotation.w();
      image_tracks[i].translation.x = camfromWorld.translation.x();
      image_tracks[i].translation.y = camfromWorld.translation.y();
      image_tracks[i].translation.z = camfromWorld.translation.z();
      
      ++i;
    }
  }
  
  return image_tracks;
}

// Atomic method to get both cameras and images in a single lock
// This prevents data inconsistency when COLMAP is modifying data between separate calls
auto ColmapSparseReconstruct::getCameraAndImageTracksArray(int id) const -> CameraAndImageArrays {
  std::lock_guard<std::mutex> lock(controller_mutex_);
  
  CameraAndImageArrays result{nullptr, 0, nullptr, 0};
  
  if (!controller_ || controller_->NumReconstructions() < 1) {
    return result;
  }
  
  // Get cameras
  const auto& cams = controller_->Cameras(id);
  if (!cams.empty()) {
    auto* cameras = static_cast<colmap::CameraTrack*>(malloc(sizeof(colmap::CameraTrack) * cams.size()));
    
    if (!cameras) {
      // malloc failed, return empty result
      return result;
    }
    
    try {
      size_t i = 0;
      for (const auto& [_, c] : cams) {
        new (&cameras[i]) colmap::CameraTrack();
        cameras[i].camera_id = c.camera_id;
        cameras[i].model_id = static_cast<int>(c.model_id);
        cameras[i].width = c.width;
        cameras[i].height = c.height;
        cameras[i].params = c.params;  // std::vector copy - can throw
        ++i;
      }
      result.camera_count = cams.size();
      result.cameras = cameras;
    } catch (...) {
      // Exception during copy - cleanup allocated memory
      for (size_t i = 0; i < cams.size(); ++i) {
        cameras[i].~CameraTrack();
      }
      free(cameras);
      return result;
    }
  }
  
  // Get images
  const auto& imgs = controller_->Images(id);
  if (!imgs.empty()) {
    auto* images = static_cast<colmap::ImageTrack*>(malloc(sizeof(colmap::ImageTrack) * imgs.size()));
    
    if (!images) {
      // malloc failed - cleanup cameras if allocated
      if (result.cameras) {
        for (size_t i = 0; i < result.camera_count; ++i) {
          result.cameras[i].~CameraTrack();
        }
        free(result.cameras);
        result.cameras = nullptr;
        result.camera_count = 0;
      }
      return result;
    }
    
    try {
      size_t i = 0;
      for (const auto& [_, img] : imgs) {
        const auto& camfromWorld = img.CamFromWorld();
        
        new (&images[i]) colmap::ImageTrack();
        images[i].image_id = img.ImageId();
        images[i].name = img.Name();  // std::string copy - can throw
        images[i].camera_id = img.CameraId();
        images[i].rotation.x = camfromWorld.rotation.x();
        images[i].rotation.y = camfromWorld.rotation.y();
        images[i].rotation.z = camfromWorld.rotation.z();
        images[i].rotation.w = camfromWorld.rotation.w();
        images[i].translation.x = camfromWorld.translation.x();
        images[i].translation.y = camfromWorld.translation.y();
        images[i].translation.z = camfromWorld.translation.z();
        
        ++i;
      }
      result.image_count = imgs.size();
      result.images = images;
    } catch (...) {
      // Exception during copy - cleanup allocated memory
      for (size_t i = 0; i < imgs.size(); ++i) {
        images[i].~ImageTrack();
      }
      free(images);
      
      // Also cleanup cameras if allocated
      if (result.cameras) {
        for (size_t i = 0; i < result.camera_count; ++i) {
          result.cameras[i].~CameraTrack();
        }
        free(result.cameras);
        result.cameras = nullptr;
        result.camera_count = 0;
      }
      
      result.image_count = 0;
      result.images = nullptr;
    }
  }
  
  return result;
}

// Legacy vector-based method (deprecated)
auto ColmapSparseReconstruct::getPoints3D(int id) const
-> std::vector<colmap::SparsePoint> {
  std::vector<colmap::SparsePoint> points;
  if (controller_) {
     if (controller_->NumReconstructions() >= 1) {
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
      if (controller_->NumReconstructions() >= 1) {
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
        if (controller_->NumReconstructions() >= 1) {
          const auto& imgs = controller_->Images(id);
          for (const auto& [_, img] : imgs) {
            const auto& camfromWorld = img.CamFromWorld();
            float rotx = camfromWorld.rotation.x();
            float roty = camfromWorld.rotation.y();
            float rotz = camfromWorld.rotation.z();
            float rotw = camfromWorld.rotation.w();
            float tx = camfromWorld.translation.x();
            float ty = camfromWorld.translation.y();
            float tz = camfromWorld.translation.z();
            imgIds.push_back({img.ImageId(),
                              img.Name(),
                              img.CameraId(),
                              colmap::vec4<float>{rotx,roty,rotz,rotw},
                              colmap::vec3<float>{tx,ty,tz}});
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

auto ColmapSparseReconstruct::stop()->void {
  if (controller_ == nullptr) return;
  controller_->Stop();
}

auto ColmapSparseReconstruct::pause() -> void {
  if (controller_ == nullptr) return;
  controller_->Pause();
}

auto ColmapSparseReconstruct::resume() -> void {
  if (controller_ == nullptr) return;
  controller_->Resume();
}

auto ColmapSparseReconstruct::wait() -> void {
  if (controller_ == nullptr) return;
  controller_->Wait();
}

bool ColmapSparseReconstruct::isStopped() { 
  if (!controller_) return false;
  return controller_->IsStopped();
}
bool ColmapSparseReconstruct::isPaused() {
  if (!controller_) return false;
  return controller_->IsPaused();
}
bool ColmapSparseReconstruct::isRunning() {
  if (!controller_) return false;
  return controller_->IsRunning();
}
bool ColmapSparseReconstruct::isFinished() {
  if (!controller_) return false;
  return controller_->IsFinished();
}