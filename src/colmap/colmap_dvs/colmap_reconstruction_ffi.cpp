/**
 * COLMAP Reconstruction FFI - C++ Implementation
 */

#include "colmap_reconstruction_ffi.h"
#include "colmap/scene/reconstruction.h"
#include "colmap/scene/reconstruction_manager.h"
#include "colmap/controllers/incremental_pipeline.h"
#include "colmap/controllers/feature_extraction.h"
#include "colmap/controllers/feature_matching.h"
#include "colmap/controllers/image_reader.h"
#include "colmap/feature/sift.h"
#include "colmap/scene/database.h"
#include "colmap/util/controller_thread.h"
#include "colmap/util/misc.h"
#include "colmap/util/file.h"
#include <memory>
#include <iostream>

#ifdef COLMAP_CUDA_ENABLED
#include <cuda_runtime.h>
#endif

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
    // Apply quality settings (matching COLMAP's OptionManager::ModifyFor*Quality())
    switch (quality) {
        case COLMAP_QUALITY_LOW:
            options->ba_local_max_num_iterations = 
                static_cast<int>(options->ba_local_max_num_iterations / 2);
            options->ba_global_max_num_iterations = 
                static_cast<int>(options->ba_global_max_num_iterations / 2);
            options->ba_global_frames_ratio *= 1.2;
            options->ba_global_points_ratio *= 1.2;
            options->ba_global_max_refinements = 2;
            break;
        case COLMAP_QUALITY_MEDIUM:
            options->ba_local_max_num_iterations = 
                static_cast<int>(options->ba_local_max_num_iterations / 1.5);
            options->ba_global_max_num_iterations = 
                static_cast<int>(options->ba_global_max_num_iterations / 1.5);
            options->ba_global_frames_ratio *= 1.1;
            options->ba_global_points_ratio *= 1.1;
            options->ba_global_max_refinements = 2;
            break;
        case COLMAP_QUALITY_HIGH:
            options->ba_local_max_num_iterations = 20;
            options->ba_local_max_refinements = 2;
            options->ba_global_max_refinements = 3;
            options->ba_global_max_num_iterations = 40;
            break;
        case COLMAP_QUALITY_EXTREME:
            options->ba_local_max_num_iterations = 40;
            options->ba_local_max_refinements = 3;
            options->ba_global_max_num_iterations = 50;
            break;
    }
    options->mapper.ba_global_ignore_redundant_points3D = true;
    options->mapper
        .ba_global_ignore_redundant_points3D_min_coverage_gain = 0.1;
}

// ============== Reconstruction Manager ==============

extern "C" {

// ============== CUDA Detection ==============

int32_t colmap_check_cuda_support() {
#ifdef COLMAP_CUDA_ENABLED
    try {
        int device_count = 0;
        cudaError_t error = cudaGetDeviceCount(&device_count);
        
        if (error != cudaSuccess || device_count == 0) {
            // No CUDA devices found
            return -1;
        }
        
        // Get properties of the first device (usually the primary GPU)
        cudaDeviceProp prop;
        error = cudaGetDeviceProperties(&prop, 0);
        
        if (error != cudaSuccess) {
            return -1;
        }
        
        // Return compute capability as integer (e.g. 60 for SM6.0, 86 for SM8.6)
        int compute_capability = prop.major * 10 + prop.minor;
        
        std::cout << "[COLMAP] CUDA device found: " << prop.name 
                  << " (Compute Capability " << prop.major << "." << prop.minor << ")" << std::endl;
        
        return compute_capability;
    } catch (...) {
        return -1;
    }
#else
    // CUDA not enabled during compilation
    return -1;
#endif
}

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

// ============== Database Query ==============

size_t colmap_database_num_images(const char* database_path) {
    if (!database_path) return 0;
    
    try {
        auto database = Database::Open(database_path);
        return database->NumImages();
    } catch (...) {
        return 0;
    }
}

// ============== Feature Extractor ==============

struct ColmapFeatureExtractor {
    std::unique_ptr<Thread> controller;
};

ColmapFeatureExtractorPtr colmap_feature_extractor_create(
    const ColmapFeatureExtractorOptions* opts) {
    
    if (!opts || !opts->database_path || !opts->image_path) {
        return nullptr;
    }
    
    try {
        auto extractor = new ColmapFeatureExtractor();
        
        // Configure image reader options
        ImageReaderOptions reader_options;
        reader_options.image_path = opts->image_path;
        reader_options.single_camera = opts->single_camera;
        reader_options.camera_model = opts->camera_model ? opts->camera_model : "SIMPLE_PINHOLE";
        
        // Configure feature extraction options
        FeatureExtractionOptions extraction_options;
        extraction_options.use_gpu = opts->use_gpu;
        
        // Configure SIFT options based on quality
        if (!extraction_options.sift) {
            extraction_options.sift = std::make_shared<SiftExtractionOptions>();
        }
        
        // Apply quality settings (matching COLMAP's OptionManager::ModifyFor*Quality())
        switch (opts->quality) {
            case COLMAP_QUALITY_LOW:
                extraction_options.max_image_size = 1000;
                (*extraction_options.sift).max_num_features = 2048;
                break;
            case COLMAP_QUALITY_MEDIUM:
                extraction_options.max_image_size = 1600;
                (*extraction_options.sift).max_num_features = 4096;
                break;
            case COLMAP_QUALITY_HIGH:
                extraction_options.max_image_size = 1600;
                (*extraction_options.sift).max_num_features = 8192;
                break;
            case COLMAP_QUALITY_EXTREME:
                extraction_options.max_image_size = 2400;
                (*extraction_options.sift).estimate_affine_shape = true;
                (*extraction_options.sift).domain_size_pooling = true;
                // max_num_features uses default (8192) for EXTREME
                break;
        }
        
        // Create feature extractor controller
        extractor->controller = CreateFeatureExtractorController(
            opts->database_path,
            reader_options,
            extraction_options
        );
        
        return extractor;
    } catch (...) {
        return nullptr;
    }
}

void colmap_feature_extractor_destroy(ColmapFeatureExtractorPtr extractor) {
    delete extractor;
}

void colmap_feature_extractor_start(ColmapFeatureExtractorPtr extractor) {
    if (!extractor || !extractor->controller) return;
    extractor->controller->Start();
}

void colmap_feature_extractor_stop(ColmapFeatureExtractorPtr extractor) {
    if (!extractor || !extractor->controller) return;
    extractor->controller->Stop();
}

void colmap_feature_extractor_pause(ColmapFeatureExtractorPtr extractor) {
    if (!extractor || !extractor->controller) return;
    extractor->controller->Pause();
}

void colmap_feature_extractor_resume(ColmapFeatureExtractorPtr extractor) {
    if (!extractor || !extractor->controller) return;
    extractor->controller->Resume();
}

void colmap_feature_extractor_wait(ColmapFeatureExtractorPtr extractor) {
    if (!extractor || !extractor->controller) return;
    extractor->controller->Wait();
}

bool colmap_feature_extractor_is_running(ColmapFeatureExtractorPtr extractor) {
    if (!extractor || !extractor->controller) return false;
    return extractor->controller->IsRunning();
}

bool colmap_feature_extractor_is_finished(ColmapFeatureExtractorPtr extractor) {
    if (!extractor || !extractor->controller) return false;
    return extractor->controller->IsFinished();
}

float colmap_feature_extractor_get_progress(ColmapFeatureExtractorPtr extractor) {
    if (!extractor || !extractor->controller) return 0.0f;
    return extractor->controller->GetProgress();
}

// ============== Feature Matcher ==============

struct ColmapFeatureMatcher {
    std::unique_ptr<Thread> controller;
};

ColmapFeatureMatcherPtr colmap_feature_matcher_create(
    const ColmapFeatureMatcherOptions* opts) {
    
    if (!opts || !opts->database_path) {
        return nullptr;
    }
    
    try {
        auto matcher = new ColmapFeatureMatcher();
        
        // Configure matching options
        FeatureMatchingOptions matching_options;
        matching_options.use_gpu = opts->use_gpu;
        
        // Apply quality settings (matching COLMAP's OptionManager::ModifyFor*Quality())
        switch (opts->quality) {
            case COLMAP_QUALITY_LOW:
                // Use defaults for LOW
                break;
            case COLMAP_QUALITY_MEDIUM:
                // Use defaults for MEDIUM
                break;
            case COLMAP_QUALITY_HIGH:
                matching_options.guided_matching = true;
                break;
            case COLMAP_QUALITY_EXTREME:
                matching_options.guided_matching = true;
                break;
        }
        
        // Configure two-view geometry options
        TwoViewGeometryOptions geometry_options;
        
        // Create matcher based on mode
        switch (opts->matching_mode) {
            case COLMAP_MATCHING_SEQUENTIAL: {
                SequentialPairingOptions pairing_options;
                pairing_options.overlap = opts->overlap > 0 ? opts->overlap : 10;
                pairing_options.loop_detection = opts->loop_detection;
                
                // Set vocab tree path if loop detection is enabled
                if (opts->loop_detection && opts->vocab_tree_path) {
                    pairing_options.vocab_tree_path = opts->vocab_tree_path;
                }
                
                // Apply quality-specific adjustments
                switch (opts->quality) {
                    case COLMAP_QUALITY_LOW:
                        pairing_options.loop_detection_num_images /= 2;
                        break;
                    case COLMAP_QUALITY_MEDIUM:
                        pairing_options.loop_detection_num_images /= 1.5;
                        break;
                    case COLMAP_QUALITY_HIGH:
                    case COLMAP_QUALITY_EXTREME:
                        // Use defaults
                        break;
                }
                
                matcher->controller = CreateSequentialFeatureMatcher(
                    pairing_options,
                    matching_options,
                    geometry_options,
                    opts->database_path
                );
                break;
            }
            case COLMAP_MATCHING_EXHAUSTIVE: {
                ExhaustivePairingOptions pairing_options;
                matcher->controller = CreateExhaustiveFeatureMatcher(
                    pairing_options,
                    matching_options,
                    geometry_options,
                    opts->database_path
                );
                break;
            }
            case COLMAP_MATCHING_VOCAB_TREE: {
                if (!opts->vocab_tree_path) {
                    delete matcher;
                    return nullptr;
                }
                VocabTreePairingOptions pairing_options;
                pairing_options.vocab_tree_path = opts->vocab_tree_path;
                
                // Apply quality-specific adjustments
                switch (opts->quality) {
                    case COLMAP_QUALITY_LOW:
                        pairing_options.max_num_features = 256;
                        pairing_options.num_images /= 2;
                        break;
                    case COLMAP_QUALITY_MEDIUM:
                        pairing_options.max_num_features = 1024;
                        pairing_options.num_images /= 1.5;
                        break;
                    case COLMAP_QUALITY_HIGH:
                        pairing_options.max_num_features = 4096;
                        break;
                    case COLMAP_QUALITY_EXTREME:
                        // Use defaults
                        break;
                }
                
                matcher->controller = CreateVocabTreeFeatureMatcher(
                    pairing_options,
                    matching_options,
                    geometry_options,
                    opts->database_path
                );
                break;
            }
            default:
                delete matcher;
                return nullptr;
        }
        
        return matcher;
    } catch (...) {
        return nullptr;
    }
}

void colmap_feature_matcher_destroy(ColmapFeatureMatcherPtr matcher) {
    delete matcher;
}

void colmap_feature_matcher_start(ColmapFeatureMatcherPtr matcher) {
    if (!matcher || !matcher->controller) return;
    matcher->controller->Start();
}

void colmap_feature_matcher_stop(ColmapFeatureMatcherPtr matcher) {
    if (!matcher || !matcher->controller) return;
    matcher->controller->Stop();
}

void colmap_feature_matcher_pause(ColmapFeatureMatcherPtr matcher) {
    if (!matcher || !matcher->controller) return;
    matcher->controller->Pause();
}

void colmap_feature_matcher_resume(ColmapFeatureMatcherPtr matcher) {
    if (!matcher || !matcher->controller) return;
    matcher->controller->Resume();
}

void colmap_feature_matcher_wait(ColmapFeatureMatcherPtr matcher) {
    if (!matcher || !matcher->controller) return;
    matcher->controller->Wait();
}

bool colmap_feature_matcher_is_running(ColmapFeatureMatcherPtr matcher) {
    if (!matcher || !matcher->controller) return false;
    return matcher->controller->IsRunning();
}

bool colmap_feature_matcher_is_finished(ColmapFeatureMatcherPtr matcher) {
    if (!matcher || !matcher->controller) return false;
    return matcher->controller->IsFinished();
}

float colmap_feature_matcher_get_progress(ColmapFeatureMatcherPtr matcher) {
    if (!matcher || !matcher->controller) return 0.0f;
    return matcher->controller->GetProgress();
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

int32_t colmap_reconstruction_write_text(
    ColmapReconstructionPtr recon,
    const char* path) {
    
    if (!recon || !recon->impl || !path) {
        std::cerr << "Invalid parameters for colmap_reconstruction_write_text" << std::endl;
        return 0;
    }
    
    try {
        // Create directory if it doesn't exist
        CreateDirIfNotExists(path, /*recursive=*/true);
        
        // Write reconstruction to text files
        recon->impl->WriteText(path);
        
        std::cout << "Successfully wrote reconstruction to " << path << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Error writing reconstruction to text: " << e.what() << std::endl;
        return 0;
    } catch (...) {
        std::cerr << "Unknown error writing reconstruction to text" << std::endl;
        return 0;
    }
}

int32_t colmap_reconstruction_write_binary(
    ColmapReconstructionPtr recon,
    const char* path) {
    
    if (!recon || !recon->impl || !path) {
        std::cerr << "Invalid parameters for colmap_reconstruction_write_binary" << std::endl;
        return 0;
    }
    
    try {
        // Create directory if it doesn't exist
        CreateDirIfNotExists(path, /*recursive=*/true);
        
        // Write reconstruction to binary files
        recon->impl->WriteBinary(path);
        
        std::cout << "Successfully wrote reconstruction to " << path << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Error writing reconstruction to binary: " << e.what() << std::endl;
        return 0;
    } catch (...) {
        std::cerr << "Unknown error writing reconstruction to binary" << std::endl;
        return 0;
    }
}

} // extern "C"

