/**
 * COLMAP Reconstruction FFI - Pure C Interface
 * 
 * This is a redesigned FFI that directly exposes ReconstructionManager
 * without unnecessary C++ wrapper layers. It provides zero-copy access
 * to reconstruction data through iterators and callbacks.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _WIN32
#if defined(COLMAP_BUILD_SHARED_LIBS)
#define COLMAP_FFI_EXPORT __declspec(dllexport)
#else
#define COLMAP_FFI_EXPORT
#endif
#else
#define COLMAP_FFI_EXPORT __attribute__((__visibility__("default")))
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// ============== CUDA Detection ==============

/**
 * Check if CUDA is available and meets minimum compute capability requirement (SM6.0)
 * @return -1 if no CUDA device found, otherwise returns the compute capability 
 *         (e.g. 60 for SM6.0, 86 for SM8.6, 89 for SM8.9)
 */
COLMAP_FFI_EXPORT int32_t colmap_check_cuda_support();

// ============== Opaque Types ==============

typedef struct ColmapReconstructionManager* ColmapReconstructionManagerPtr;
typedef struct ColmapReconstruction* ColmapReconstructionPtr;
typedef struct ColmapIncrementalMapper* ColmapIncrementalMapperPtr;

// ============== Option Structures ==============

typedef enum {
    COLMAP_QUALITY_LOW = 0,
    COLMAP_QUALITY_MEDIUM = 1,
    COLMAP_QUALITY_HIGH = 2,
    COLMAP_QUALITY_EXTREME = 3
} ColmapQuality;

typedef struct {
    const char* database_path;
    const char* image_path;
    const char* mask_path;           // Optional, can be NULL
    const char* camera_model;        // e.g., "SIMPLE_PINHOLE"
    int32_t gpu_index;               // -1 for auto
    bool is_video;
    bool use_gpu;
    bool single_camera;
    ColmapQuality quality;
} ColmapMapperOptions;

// ============== Callback Types ==============

// Callback for each 3D point
// point_id: unique point ID
// xyz: 3D position (3 floats)
// rgb: color (3 uint8_t)
// error: reprojection error
// track_length: number of observations
typedef void (*ColmapPoint3DCallback)(
    uint64_t point_id,
    const float* xyz,
    const uint8_t* rgb,
    double error,
    size_t track_length,
    void* user_data);

// Callback for each camera
// camera_id: unique camera ID
// model_id: camera model type (0=SIMPLE_PINHOLE, etc.)
// width, height: image dimensions
// params: camera parameters array
// num_params: number of parameters
typedef void (*ColmapCameraCallback)(
    uint32_t camera_id,
    int32_t model_id,
    uint32_t width,
    uint32_t height,
    const double* params,
    size_t num_params,
    void* user_data);

// Callback for each registered image
// image_id: unique image ID
// name: image filename
// camera_id: associated camera ID
// qvec: quaternion rotation (4 doubles: qw, qx, qy, qz)
// tvec: translation (3 doubles: tx, ty, tz)
// num_points3d: number of triangulated points
typedef void (*ColmapImageCallback)(
    uint32_t image_id,
    const char* name,
    uint32_t camera_id,
    const double* qvec,
    const double* tvec,
    size_t num_points3d,
    void* user_data);

// Progress callback
// phase: 0=feature_extract, 1=feature_match, 2=sfm
// progress: 0.0 to 1.0
// num_reg_images: number of registered images so far
// num_points: number of 3D points so far
typedef void (*ColmapProgressCallback)(
    int32_t phase,
    float progress,
    size_t num_reg_images,
    size_t num_points,
    void* user_data);

// ============== Reconstruction Manager ==============

/**
 * Create a reconstruction manager
 */
COLMAP_FFI_EXPORT ColmapReconstructionManagerPtr 
colmap_reconstruction_manager_create();

/**
 * Destroy reconstruction manager
 */
COLMAP_FFI_EXPORT void 
colmap_reconstruction_manager_destroy(ColmapReconstructionManagerPtr mgr);

/**
 * Get number of reconstructions
 */
COLMAP_FFI_EXPORT size_t 
colmap_reconstruction_manager_size(ColmapReconstructionManagerPtr mgr);

/**
 * Get reconstruction by index (returns NULL if invalid)
 */
COLMAP_FFI_EXPORT ColmapReconstructionPtr 
colmap_reconstruction_manager_get(ColmapReconstructionManagerPtr mgr, size_t idx);

// ============== Feature Extractor ==============

/**
 * Feature extraction options
 */
typedef struct {
    const char* database_path;
    const char* image_path;
    const char* camera_model;  // e.g., "SIMPLE_PINHOLE", "PINHOLE", "SIMPLE_RADIAL"
    bool single_camera;
    bool use_gpu;
    ColmapQuality quality;
} ColmapFeatureExtractorOptions;

typedef struct ColmapFeatureExtractor ColmapFeatureExtractor;
typedef ColmapFeatureExtractor* ColmapFeatureExtractorPtr;

/**
 * Create a feature extractor
 */
COLMAP_FFI_EXPORT ColmapFeatureExtractorPtr
colmap_feature_extractor_create(const ColmapFeatureExtractorOptions* opts);

/**
 * Destroy feature extractor
 */
COLMAP_FFI_EXPORT void
colmap_feature_extractor_destroy(ColmapFeatureExtractorPtr extractor);

/**
 * Start feature extraction
 */
COLMAP_FFI_EXPORT void
colmap_feature_extractor_start(ColmapFeatureExtractorPtr extractor);

/**
 * Stop feature extraction
 */
COLMAP_FFI_EXPORT void
colmap_feature_extractor_stop(ColmapFeatureExtractorPtr extractor);

/**
 * Pause feature extraction
 */
COLMAP_FFI_EXPORT void
colmap_feature_extractor_pause(ColmapFeatureExtractorPtr extractor);

/**
 * Resume feature extraction
 */
COLMAP_FFI_EXPORT void
colmap_feature_extractor_resume(ColmapFeatureExtractorPtr extractor);

/**
 * Wait for feature extraction to complete
 */
COLMAP_FFI_EXPORT void
colmap_feature_extractor_wait(ColmapFeatureExtractorPtr extractor);

/**
 * Check if feature extraction is running
 */
COLMAP_FFI_EXPORT bool
colmap_feature_extractor_is_running(ColmapFeatureExtractorPtr extractor);

/**
 * Check if feature extraction is finished
 */
COLMAP_FFI_EXPORT bool
colmap_feature_extractor_is_finished(ColmapFeatureExtractorPtr extractor);

/**
 * Get feature extraction progress (0.0 to 1.0)
 */
COLMAP_FFI_EXPORT float
colmap_feature_extractor_get_progress(ColmapFeatureExtractorPtr extractor);

// ============== Database Query ==============

/**
 * Get number of images in database
 */
COLMAP_FFI_EXPORT size_t
colmap_database_num_images(const char* database_path);

// ============== Feature Matcher ==============

/**
 * Feature matching mode
 */
typedef enum {
    COLMAP_MATCHING_SEQUENTIAL = 0,  // For video/sequential images
    COLMAP_MATCHING_EXHAUSTIVE = 1,  // For small datasets
    COLMAP_MATCHING_VOCAB_TREE = 2   // For large datasets (requires vocab tree)
} ColmapMatchingMode;

/**
 * Feature matching options
 */
typedef struct {
    const char* database_path;
    ColmapMatchingMode matching_mode;
    bool use_gpu;
    ColmapQuality quality;
    // For sequential matching
    int32_t overlap;  // Number of overlapping images (default: 10)
    bool loop_detection;  // Enable loop detection for sequential matching
    // For vocab tree matching
    const char* vocab_tree_path;  // Path to vocabulary tree file
} ColmapFeatureMatcherOptions;

typedef struct ColmapFeatureMatcher ColmapFeatureMatcher;
typedef ColmapFeatureMatcher* ColmapFeatureMatcherPtr;

/**
 * Create a feature matcher
 */
COLMAP_FFI_EXPORT ColmapFeatureMatcherPtr
colmap_feature_matcher_create(const ColmapFeatureMatcherOptions* opts);

/**
 * Destroy feature matcher
 */
COLMAP_FFI_EXPORT void
colmap_feature_matcher_destroy(ColmapFeatureMatcherPtr matcher);

/**
 * Start feature matching
 */
COLMAP_FFI_EXPORT void
colmap_feature_matcher_start(ColmapFeatureMatcherPtr matcher);

/**
 * Stop feature matching
 */
COLMAP_FFI_EXPORT void
colmap_feature_matcher_stop(ColmapFeatureMatcherPtr matcher);

/**
 * Pause feature matching
 */
COLMAP_FFI_EXPORT void
colmap_feature_matcher_pause(ColmapFeatureMatcherPtr matcher);

/**
 * Resume feature matching
 */
COLMAP_FFI_EXPORT void
colmap_feature_matcher_resume(ColmapFeatureMatcherPtr matcher);

/**
 * Wait for feature matching to complete
 */
COLMAP_FFI_EXPORT void
colmap_feature_matcher_wait(ColmapFeatureMatcherPtr matcher);

/**
 * Check if feature matching is running
 */
COLMAP_FFI_EXPORT bool
colmap_feature_matcher_is_running(ColmapFeatureMatcherPtr matcher);

/**
 * Check if feature matching is finished
 */
COLMAP_FFI_EXPORT bool
colmap_feature_matcher_is_finished(ColmapFeatureMatcherPtr matcher);

/**
 * Get feature matching progress (0.0 to 1.0)
 */
COLMAP_FFI_EXPORT float
colmap_feature_matcher_get_progress(ColmapFeatureMatcherPtr matcher);

// ============== Incremental Mapper ==============

/**
 * Create incremental mapper
 */
COLMAP_FFI_EXPORT ColmapIncrementalMapperPtr 
colmap_incremental_mapper_create(
    const ColmapMapperOptions* options,
    ColmapReconstructionManagerPtr mgr);

/**
 * Destroy mapper
 */
COLMAP_FFI_EXPORT void 
colmap_incremental_mapper_destroy(ColmapIncrementalMapperPtr mapper);

/**
 * Start mapping in background thread
 */
COLMAP_FFI_EXPORT void 
colmap_incremental_mapper_start(ColmapIncrementalMapperPtr mapper);

/**
 * Stop mapping
 */
COLMAP_FFI_EXPORT void 
colmap_incremental_mapper_stop(ColmapIncrementalMapperPtr mapper);

/**
 * Pause mapping
 */
COLMAP_FFI_EXPORT void 
colmap_incremental_mapper_pause(ColmapIncrementalMapperPtr mapper);

/**
 * Resume mapping
 */
COLMAP_FFI_EXPORT void 
colmap_incremental_mapper_resume(ColmapIncrementalMapperPtr mapper);

/**
 * Wait for mapping to complete
 */
COLMAP_FFI_EXPORT void 
colmap_incremental_mapper_wait(ColmapIncrementalMapperPtr mapper);

/**
 * Check if mapper is running
 */
COLMAP_FFI_EXPORT bool 
colmap_incremental_mapper_is_running(ColmapIncrementalMapperPtr mapper);

/**
 * Check if mapper is paused
 */
COLMAP_FFI_EXPORT bool 
colmap_incremental_mapper_is_paused(ColmapIncrementalMapperPtr mapper);

/**
 * Check if mapper is stopped
 */
COLMAP_FFI_EXPORT bool 
colmap_incremental_mapper_is_stopped(ColmapIncrementalMapperPtr mapper);

/**
 * Check if mapper is finished
 */
COLMAP_FFI_EXPORT bool 
colmap_incremental_mapper_is_finished(ColmapIncrementalMapperPtr mapper);

/**
 * Set progress callback
 */
COLMAP_FFI_EXPORT void 
colmap_incremental_mapper_set_progress_callback(
    ColmapIncrementalMapperPtr mapper,
    ColmapProgressCallback callback,
    void* user_data);

// ============== Reconstruction Access ==============

/**
 * Get number of 3D points
 */
COLMAP_FFI_EXPORT size_t 
colmap_reconstruction_num_points3d(ColmapReconstructionPtr recon);

/**
 * Get number of cameras
 */
COLMAP_FFI_EXPORT size_t 
colmap_reconstruction_num_cameras(ColmapReconstructionPtr recon);

/**
 * Get number of registered images
 */
COLMAP_FFI_EXPORT size_t 
colmap_reconstruction_num_reg_images(ColmapReconstructionPtr recon);

/**
 * Get number of all images (registered + unregistered)
 */
COLMAP_FFI_EXPORT size_t 
colmap_reconstruction_num_images(ColmapReconstructionPtr recon);

/**
 * Iterate over all 3D points (zero-copy, thread-safe read)
 * The callback is called for each point while holding a read lock
 */
COLMAP_FFI_EXPORT void 
colmap_reconstruction_foreach_point3d(
    ColmapReconstructionPtr recon,
    ColmapPoint3DCallback callback,
    void* user_data);

/**
 * Iterate over all cameras (zero-copy, thread-safe read)
 */
COLMAP_FFI_EXPORT void 
colmap_reconstruction_foreach_camera(
    ColmapReconstructionPtr recon,
    ColmapCameraCallback callback,
    void* user_data);

/**
 * Iterate over all registered images (zero-copy, thread-safe read)
 */
COLMAP_FFI_EXPORT void 
colmap_reconstruction_foreach_image(
    ColmapReconstructionPtr recon,
    ColmapImageCallback callback,
    void* user_data);

// ============== Batch Copy Functions (for performance) ==============

/**
 * Copy all points to pre-allocated arrays
 * xyz: float[num_points * 3]
 * rgb: uint8_t[num_points * 3]
 * Returns actual number of points copied
 */
COLMAP_FFI_EXPORT size_t 
colmap_reconstruction_copy_points3d(
    ColmapReconstructionPtr recon,
    float* xyz,
    uint8_t* rgb,
    size_t max_points);

/**
 * Copy all camera poses to pre-allocated arrays
 * image_ids: uint32_t[num_images]
 * qvec: double[num_images * 4]  (qw, qx, qy, qz)
 * tvec: double[num_images * 3]  (tx, ty, tz)
 * Returns actual number of images copied
 */
COLMAP_FFI_EXPORT size_t 
colmap_reconstruction_copy_image_poses(
    ColmapReconstructionPtr recon,
    uint32_t* image_ids,
    double* qvec,
    double* tvec,
    size_t max_images);

/**
 * Write reconstruction to text files (cameras.txt, images.txt, points3D.txt)
 * Returns 1 on success, 0 on failure
 */
COLMAP_FFI_EXPORT int32_t 
colmap_reconstruction_write_text(
    ColmapReconstructionPtr recon,
    const char* path);

/**
 * Write reconstruction to binary files (cameras.bin, images.bin, points3D.bin)
 * Returns 1 on success, 0 on failure
 */
COLMAP_FFI_EXPORT int32_t 
colmap_reconstruction_write_binary(
    ColmapReconstructionPtr recon,
    const char* path);

#ifdef __cplusplus
}
#endif

