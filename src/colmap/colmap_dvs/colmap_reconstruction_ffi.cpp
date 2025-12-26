/**
 * COLMAP Reconstruction FFI - C++ Implementation
 */

#include "colmap_reconstruction_ffi.h"
#include "colmap/scene/reconstruction.h"
#include "colmap/scene/reconstruction_manager.h"
#include "colmap/controllers/incremental_pipeline.h"
#include "colmap/util/controller_thread.h"
#include <memory>

using namespace colmap;

// ============== Internal Structures ==============

struct ColmapReconstructionManager {
    std::shared_ptr<ReconstructionManager> impl;
    
    ColmapReconstructionManager() 
        : impl(std::make_shared<ReconstructionManager>()) {}
};

struct ColmapReconstruction {
    std::shared_ptr<Reconstruction> impl;
    
    ColmapReconstruction(std::shared_ptr<Reconstruction> recon)
        : impl(recon) {}
};

struct ColmapIncrementalMapper {
    std::unique_ptr<IncrementalPipeline> pipeline;
    std::unique_ptr<ControllerThread<IncrementalPipeline>> controller_thread;
    std::shared_ptr<IncrementalPipelineOptions> options;
    std::shared_ptr<ReconstructionManager> reconstruction_manager;
    ColmapProgressCallback progress_callback = nullptr;
    void* progress_user_data = nullptr;
    
    ~ColmapIncrementalMapper() {
        if (controller_thread && controller_thread->IsRunning()) {
            controller_thread->Stop();
            controller_thread->Wait();
        }
    }
};

// ============== Helper Functions ==============

static void ConfigureOptionsFromQuality(IncrementalPipelineOptions* options, ColmapQuality quality) {
    // Quality settings - just use defaults for now
    // The IncrementalMapper::Options structure doesn't expose BA settings directly
    // They are configured through BundleAdjustmentOptions internally
    (void)options;
    (void)quality;
}

// ============== Reconstruction Manager ==============

extern "C" {

ColmapReconstructionManagerPtr colmap_reconstruction_manager_create() {
    try {
        return new ColmapReconstructionManager();
    } catch (...) {
        return nullptr;
    }
}

void colmap_reconstruction_manager_destroy(ColmapReconstructionManagerPtr mgr) {
    delete mgr;
}

size_t colmap_reconstruction_manager_size(ColmapReconstructionManagerPtr mgr) {
    if (!mgr || !mgr->impl) return 0;
    return mgr->impl->Size();
}

ColmapReconstructionPtr colmap_reconstruction_manager_get(
    ColmapReconstructionManagerPtr mgr, 
    size_t idx) {
    
    if (!mgr || !mgr->impl || idx >= mgr->impl->Size()) {
        return nullptr;
    }
    
    try {
        auto recon = mgr->impl->Get(idx);
        return new ColmapReconstruction(recon);
    } catch (...) {
        return nullptr;
    }
}

// ============== Incremental Mapper ==============

ColmapIncrementalMapperPtr colmap_incremental_mapper_create(
    const ColmapMapperOptions* opts,
    ColmapReconstructionManagerPtr mgr) {
    
    if (!opts || !mgr || !mgr->impl) {
        return nullptr;
    }
    
    try {
        auto mapper = new ColmapIncrementalMapper();
        mapper->reconstruction_manager = mgr->impl;
        
        // Configure options
        mapper->options = std::make_shared<IncrementalPipelineOptions>();
        
        // Configure quality
        ConfigureOptionsFromQuality(mapper->options.get(), opts->quality);
        
        // Create pipeline
        mapper->pipeline = std::make_unique<IncrementalPipeline>(
            mapper->options,
            opts->image_path ? opts->image_path : "",
            opts->database_path ? opts->database_path : "",
            mapper->reconstruction_manager
        );
        
        // Create controller thread
        mapper->controller_thread = std::make_unique<ControllerThread<IncrementalPipeline>>(
            std::shared_ptr<IncrementalPipeline>(mapper->pipeline.get(), [](IncrementalPipeline*){}));

        
        return mapper;
    } catch (...) {
        return nullptr;
    }
}

void colmap_incremental_mapper_destroy(ColmapIncrementalMapperPtr mapper) {
    delete mapper;
}

void colmap_incremental_mapper_start(ColmapIncrementalMapperPtr mapper) {
    if (!mapper || !mapper->controller_thread) return;
    mapper->controller_thread->Start();
}

void colmap_incremental_mapper_stop(ColmapIncrementalMapperPtr mapper) {
    if (!mapper || !mapper->controller_thread) return;
    mapper->controller_thread->Stop();
}

void colmap_incremental_mapper_pause(ColmapIncrementalMapperPtr mapper) {
    if (!mapper || !mapper->controller_thread) return;
    mapper->controller_thread->Pause();
}

void colmap_incremental_mapper_resume(ColmapIncrementalMapperPtr mapper) {
    if (!mapper || !mapper->controller_thread) return;
    mapper->controller_thread->Resume();
}

void colmap_incremental_mapper_wait(ColmapIncrementalMapperPtr mapper) {
    if (!mapper || !mapper->controller_thread) return;
    mapper->controller_thread->Wait();
}

bool colmap_incremental_mapper_is_running(ColmapIncrementalMapperPtr mapper) {
    if (!mapper || !mapper->controller_thread) return false;
    return mapper->controller_thread->IsRunning();
}

bool colmap_incremental_mapper_is_paused(ColmapIncrementalMapperPtr mapper) {
    if (!mapper || !mapper->controller_thread) return false;
    return mapper->controller_thread->IsPaused();
}

bool colmap_incremental_mapper_is_stopped(ColmapIncrementalMapperPtr mapper) {
    if (!mapper || !mapper->controller_thread) return false;
    return mapper->controller_thread->IsStopped();
}

bool colmap_incremental_mapper_is_finished(ColmapIncrementalMapperPtr mapper) {
    if (!mapper || !mapper->controller_thread) return false;
    return mapper->controller_thread->IsFinished();
}

void colmap_incremental_mapper_set_progress_callback(
    ColmapIncrementalMapperPtr mapper,
    ColmapProgressCallback callback,
    void* user_data) {
    
    if (!mapper) return;
    mapper->progress_callback = callback;
    mapper->progress_user_data = user_data;
}

// ============== Reconstruction Access ==============

size_t colmap_reconstruction_num_points3d(ColmapReconstructionPtr recon) {
    if (!recon || !recon->impl) return 0;
    return recon->impl->NumPoints3D();
}

size_t colmap_reconstruction_num_cameras(ColmapReconstructionPtr recon) {
    if (!recon || !recon->impl) return 0;
    return recon->impl->NumCameras();
}

size_t colmap_reconstruction_num_reg_images(ColmapReconstructionPtr recon) {
    if (!recon || !recon->impl) return 0;
    return recon->impl->NumRegImages();
}

size_t colmap_reconstruction_num_images(ColmapReconstructionPtr recon) {
    if (!recon || !recon->impl) return 0;
    return recon->impl->NumImages();
}

void colmap_reconstruction_foreach_point3d(
    ColmapReconstructionPtr recon,
    ColmapPoint3DCallback callback,
    void* user_data) {
    
    if (!recon || !recon->impl || !callback) return;
    
    // Iterate over all 3D points
    for (const auto& [point_id, point] : recon->impl->Points3D()) {
        float xyz[3] = {
            static_cast<float>(point.xyz(0)),
            static_cast<float>(point.xyz(1)),
            static_cast<float>(point.xyz(2))
        };
        
        uint8_t rgb[3] = {
            point.color(0),
            point.color(1),
            point.color(2)
        };
        
        callback(
            point_id,
            xyz,
            rgb,
            point.error,
            point.track.Length(),
            user_data
        );
    }
}

void colmap_reconstruction_foreach_camera(
    ColmapReconstructionPtr recon,
    ColmapCameraCallback callback,
    void* user_data) {
    
    if (!recon || !recon->impl || !callback) return;
    
    // Iterate over all cameras
    for (const auto& [camera_id, camera] : recon->impl->Cameras()) {
        callback(
            camera_id,
            static_cast<int32_t>(camera.model_id),
            static_cast<uint32_t>(camera.width),
            static_cast<uint32_t>(camera.height),
            camera.params.data(),
            camera.params.size(),
            user_data
        );
    }
}

void colmap_reconstruction_foreach_image(
    ColmapReconstructionPtr recon,
    ColmapImageCallback callback,
    void* user_data) {
    
    if (!recon || !recon->impl || !callback) return;
    
    // Iterate over registered images only
    for (const image_t image_id : recon->impl->RegImageIds()) {
        const auto& image = recon->impl->Image(image_id);
        const auto& cam_from_world = image.CamFromWorld();
        
        double qvec[4] = {
            cam_from_world.rotation.w(),
            cam_from_world.rotation.x(),
            cam_from_world.rotation.y(),
            cam_from_world.rotation.z()
        };
        
        double tvec[3] = {
            cam_from_world.translation.x(),
            cam_from_world.translation.y(),
            cam_from_world.translation.z()
        };
        
        callback(
            image_id,
            image.Name().c_str(),
            image.CameraId(),
            qvec,
            tvec,
            image.NumPoints3D(),
            user_data
        );
    }
}

// ============== Batch Copy Functions ==============

size_t colmap_reconstruction_copy_points3d(
    ColmapReconstructionPtr recon,
    float* xyz,
    uint8_t* rgb,
    size_t max_points) {
    
    if (!recon || !recon->impl || !xyz || !rgb) return 0;
    
    const auto& points = recon->impl->Points3D();
    size_t count = std::min(points.size(), max_points);
    
    size_t idx = 0;
    for (const auto& [point_id, point] : points) {
        if (idx >= count) break;
        
        xyz[idx * 3 + 0] = static_cast<float>(point.xyz(0));
        xyz[idx * 3 + 1] = static_cast<float>(point.xyz(1));
        xyz[idx * 3 + 2] = static_cast<float>(point.xyz(2));
        
        rgb[idx * 3 + 0] = point.color(0);
        rgb[idx * 3 + 1] = point.color(1);
        rgb[idx * 3 + 2] = point.color(2);
        
        ++idx;
    }
    
    return idx;
}

size_t colmap_reconstruction_copy_image_poses(
    ColmapReconstructionPtr recon,
    uint32_t* image_ids,
    double* qvec,
    double* tvec,
    size_t max_images) {
    
    if (!recon || !recon->impl || !image_ids || !qvec || !tvec) return 0;
    
    const auto& reg_image_ids = recon->impl->RegImageIds();
    size_t count = std::min(reg_image_ids.size(), max_images);
    
    size_t idx = 0;
    for (const image_t image_id : reg_image_ids) {
        if (idx >= count) break;
        
        const auto& image = recon->impl->Image(image_id);
        const auto& cam_from_world = image.CamFromWorld();
        
        image_ids[idx] = image_id;
        
        qvec[idx * 4 + 0] = cam_from_world.rotation.w();
        qvec[idx * 4 + 1] = cam_from_world.rotation.x();
        qvec[idx * 4 + 2] = cam_from_world.rotation.y();
        qvec[idx * 4 + 3] = cam_from_world.rotation.z();
        
        tvec[idx * 3 + 0] = cam_from_world.translation.x();
        tvec[idx * 3 + 1] = cam_from_world.translation.y();
        tvec[idx * 3 + 2] = cam_from_world.translation.z();
        
        ++idx;
    }
    
    return idx;
}

} // extern "C"

