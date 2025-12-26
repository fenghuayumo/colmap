/**
 * C-style FFI interface for ColmapSparseReconstruct
 * This header provides a C ABI compatible interface for Rust FFI bindings
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

// Opaque pointer to ColmapSparseReconstruct
typedef struct ColmapReconstructHandle* ColmapReconstructPtr;

// Quality enum (matches C++ Quality enum)
typedef enum {
    COLMAP_QUALITY_LOW = 0,
    COLMAP_QUALITY_MEDIUM = 1,
    COLMAP_QUALITY_HIGH = 2,
    COLMAP_QUALITY_EXTREME = 3
} ColmapQuality;

// Reconstruction phase enum
typedef enum {
    COLMAP_PHASE_INIT = 0,
    COLMAP_PHASE_FEATURE_EXTRACT = 1,
    COLMAP_PHASE_FEATURE_MATCH = 2,
    COLMAP_PHASE_SFM = 3,
    COLMAP_PHASE_DONE = 4,
    COLMAP_PHASE_FAILED = -1
} ColmapPhase;

// Configuration options for sparse reconstruction
typedef struct {
    const char* image_path;
    const char* workspace_path;
    const char* mask_path;          // Can be NULL
    const char* camera_model;       // e.g., "SIMPLE_PINHOLE", "PINHOLE", etc.
    int32_t gpu_index;              // -1 for auto
    uint8_t is_video;               // 0 = false, 1 = true
    uint8_t use_hierarchy;          // 0 = false, 1 = true
    uint8_t use_gpu;                // 0 = false, 1 = true
    uint8_t use_glomap;             // 0 = false, 1 = true
    uint8_t output_sparse_points;   // 0 = false, 1 = true
    uint8_t single_camera;          // 0 = false, 1 = true
    ColmapQuality quality;
} ColmapOptions;

// 3D point structure (POD type for FFI)
typedef struct {
    float x, y, z;
    uint8_t r, g, b, a;
} ColmapSparsePoint;

// Camera track structure
typedef struct {
    uint32_t camera_id;
    int32_t model_id;
    uint32_t width;
    uint32_t height;
    double* params;        // Array of camera parameters
    size_t params_count;   // Number of parameters
} ColmapCameraTrack;

// Image track structure
typedef struct {
    uint32_t image_id;
    char* name;            // Allocated string, must be freed with colmap_free_string
    uint32_t camera_id;
    float qw, qx, qy, qz;  // Rotation quaternion (wxyz order)
    float tx, ty, tz;      // Translation
} ColmapImageTrack;

// Array wrapper for returning multiple items
typedef struct {
    ColmapSparsePoint* data;
    size_t count;
} ColmapSparsePointArray;

typedef struct {
    ColmapCameraTrack* data;
    size_t count;
} ColmapCameraTrackArray;

typedef struct {
    ColmapImageTrack* data;
    size_t count;
} ColmapImageTrackArray;

// Progress callback function type
typedef void (*ColmapProgressCallback)(int32_t phase, float progress, void* user_data);

// ============== Lifecycle Functions ==============

/**
 * Create a new ColmapSparseReconstruct instance
 * @return Opaque pointer to the instance, or NULL on failure
 */
COLMAP_FFI_EXPORT ColmapReconstructPtr colmap_create(void);

/**
 * Destroy a ColmapSparseReconstruct instance
 * @param handle Pointer to the instance
 */
COLMAP_FFI_EXPORT void colmap_destroy(ColmapReconstructPtr handle);

// ============== Configuration Functions ==============

/**
 * Set reconstruction options
 * @param handle Pointer to the instance
 * @param options Configuration options
 * @return 0 on success, non-zero on failure
 */
COLMAP_FFI_EXPORT int32_t colmap_set_options(ColmapReconstructPtr handle, const ColmapOptions* options);

/**
 * Set progress callback
 * @param handle Pointer to the instance
 * @param callback Progress callback function
 * @param user_data User data passed to callback
 */
COLMAP_FFI_EXPORT void colmap_set_progress_callback(
    ColmapReconstructPtr handle,
    ColmapProgressCallback callback,
    void* user_data
);

// ============== Execution Functions ==============

/**
 * Run sparse reconstruction (blocking)
 * @param handle Pointer to the instance
 * @return 1 on success, 0 on failure
 */
COLMAP_FFI_EXPORT int32_t colmap_run(ColmapReconstructPtr handle);

/**
 * Pause reconstruction
 * @param handle Pointer to the instance
 */
COLMAP_FFI_EXPORT void colmap_pause(ColmapReconstructPtr handle);

/**
 * Resume reconstruction
 * @param handle Pointer to the instance
 */
COLMAP_FFI_EXPORT void colmap_resume(ColmapReconstructPtr handle);

/**
 * Stop reconstruction
 * @param handle Pointer to the instance
 */
COLMAP_FFI_EXPORT void colmap_stop(ColmapReconstructPtr handle);

/**
 * Wait for reconstruction to complete
 * @param handle Pointer to the instance
 */
COLMAP_FFI_EXPORT void colmap_wait(ColmapReconstructPtr handle);

// ============== Status Functions ==============

/**
 * Get current reconstruction phase
 * @param handle Pointer to the instance
 * @return Current phase enum value
 */
COLMAP_FFI_EXPORT ColmapPhase colmap_get_phase(ColmapReconstructPtr handle);

/**
 * Get progress on current phase (0.0 - 1.0)
 * @param handle Pointer to the instance
 * @return Progress value
 */
COLMAP_FFI_EXPORT float colmap_get_progress(ColmapReconstructPtr handle);

/**
 * Check if reconstruction is running
 * @param handle Pointer to the instance
 * @return 1 if running, 0 otherwise
 */
COLMAP_FFI_EXPORT int32_t colmap_is_running(ColmapReconstructPtr handle);

/**
 * Check if reconstruction is paused
 * @param handle Pointer to the instance
 * @return 1 if paused, 0 otherwise
 */
COLMAP_FFI_EXPORT int32_t colmap_is_paused(ColmapReconstructPtr handle);

/**
 * Check if reconstruction is stopped
 * @param handle Pointer to the instance
 * @return 1 if stopped, 0 otherwise
 */
COLMAP_FFI_EXPORT int32_t colmap_is_stopped(ColmapReconstructPtr handle);

/**
 * Check if reconstruction is finished
 * @param handle Pointer to the instance
 * @return 1 if finished, 0 otherwise
 */
COLMAP_FFI_EXPORT int32_t colmap_is_finished(ColmapReconstructPtr handle);

// ============== Result Functions ==============

/**
 * Get 3D points from reconstruction
 * @param handle Pointer to the instance
 * @param model_id Model ID (usually 0)
 * @return Array of sparse points (must be freed with colmap_free_points)
 */
COLMAP_FFI_EXPORT ColmapSparsePointArray colmap_get_points3d(ColmapReconstructPtr handle, int32_t model_id);

/**
 * Get camera tracks from reconstruction
 * @param handle Pointer to the instance
 * @param model_id Model ID (usually 0)
 * @return Array of camera tracks (must be freed with colmap_free_camera_tracks)
 */
COLMAP_FFI_EXPORT ColmapCameraTrackArray colmap_get_camera_tracks(ColmapReconstructPtr handle, int32_t model_id);

/**
 * Get image tracks from reconstruction
 * @param handle Pointer to the instance
 * @param model_id Model ID (usually 0)
 * @return Array of image tracks (must be freed with colmap_free_image_tracks)
 */
COLMAP_FFI_EXPORT ColmapImageTrackArray colmap_get_image_tracks(ColmapReconstructPtr handle, int32_t model_id);

/**
 * Atomically get both camera and image tracks in a single lock
 * This prevents data inconsistency when COLMAP is modifying data
 * @param handle Pointer to the instance
 * @param model_id Model ID (usually 0)
 * @param out_camera_array Output pointer for camera tracks array
 * @param out_image_array Output pointer for image tracks array
 */
COLMAP_FFI_EXPORT void colmap_get_camera_and_image_tracks(
    ColmapReconstructPtr handle,
    int32_t model_id,
    ColmapCameraTrackArray* out_camera_array,
    ColmapImageTrackArray* out_image_array);

// ============== Memory Management Functions ==============

/**
 * Free sparse points array
 * @param arr Array to free
 */
COLMAP_FFI_EXPORT void colmap_free_points(ColmapSparsePointArray* arr);

/**
 * Free camera tracks array
 * @param arr Array to free
 */
COLMAP_FFI_EXPORT void colmap_free_camera_tracks(ColmapCameraTrackArray* arr);

/**
 * Free image tracks array
 * @param arr Array to free
 */
COLMAP_FFI_EXPORT void colmap_free_image_tracks(ColmapImageTrackArray* arr);

/**
 * Free a string allocated by this library
 * @param str String to free
 */
COLMAP_FFI_EXPORT void colmap_free_string(char* str);

#ifdef __cplusplus
}
#endif

