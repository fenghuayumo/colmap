/**
 * C-style FFI implementation for ColmapSparseReconstruct
 */

#include "colmap_ffi.h"
#include "colmap_lib.h"
#include <cstring>
#include <cstdlib>
#include <string>

// Internal structure to hold callback info
struct ColmapReconstructHandle {
    ColmapSparseReconstruct* impl;
    ColmapProgressCallback progress_callback;
    void* user_data;
    
    ColmapReconstructHandle() : impl(nullptr), progress_callback(nullptr), user_data(nullptr) {}
};

extern "C" {

ColmapReconstructPtr colmap_create(void) {
    auto* handle = new ColmapReconstructHandle();
    handle->impl = new ColmapSparseReconstruct();
    return handle;
}

void colmap_destroy(ColmapReconstructPtr handle) {
    if (handle) {
        if (handle->impl) {
            // Stop any running reconstruction first
            if (handle->impl->isRunning()) {
                handle->impl->stop();
                handle->impl->wait();
            }
            delete handle->impl;
        }
        delete handle;
    }
}

int32_t colmap_set_options(ColmapReconstructPtr handle, const ColmapOptions* options) {
    if (!handle || !handle->impl || !options) {
        return -1;
    }
    
    auto& opt = handle->impl->option;
    
    if (options->image_path) {
        opt.image_path = options->image_path;
    }
    if (options->workspace_path) {
        opt.workspace_path = options->workspace_path;
    }
    if (options->mask_path) {
        opt.mask_path = options->mask_path;
    }
    if (options->camera_model) {
        opt.camera_model = options->camera_model;
    }
    
    opt.gpu_index = options->gpu_index;
    opt.video = options->is_video != 0;
    opt.use_hierachy = options->use_hierarchy != 0;
    opt.use_gpu = options->use_gpu != 0;
    opt.use_glomap = options->use_glomap != 0;
    opt.output_sparse_points = options->output_sparse_points != 0;
    opt.single_camera = options->single_camera != 0;
    opt.quality = static_cast<ColmapSparseReconstruct::Quality>(options->quality);
    
    return 0;
}

void colmap_set_progress_callback(
    ColmapReconstructPtr handle,
    ColmapProgressCallback callback,
    void* user_data
) {
    if (handle) {
        handle->progress_callback = callback;
        handle->user_data = user_data;
    }
}

int32_t colmap_run(ColmapReconstructPtr handle) {
    if (!handle || !handle->impl) {
        return 0;
    }
    
    bool result = handle->impl->run();
    return result ? 1 : 0;
}

void colmap_pause(ColmapReconstructPtr handle) {
    if (handle && handle->impl) {
        handle->impl->pause();
    }
}

void colmap_resume(ColmapReconstructPtr handle) {
    if (handle && handle->impl) {
        handle->impl->resume();
    }
}

void colmap_stop(ColmapReconstructPtr handle) {
    if (handle && handle->impl) {
        handle->impl->stop();
    }
}

void colmap_wait(ColmapReconstructPtr handle) {
    if (handle && handle->impl) {
        handle->impl->wait();
    }
}

ColmapPhase colmap_get_phase(ColmapReconstructPtr handle) {
    if (!handle || !handle->impl) {
        return COLMAP_PHASE_FAILED;
    }
    
    int phase = handle->impl->GetSparseReconstructPhase();
    // Map C++ phase values to C enum
    // Phase values: 0=init, 1=feature_extract, 2=feature_match, 3=sfm, 4=done
    if (phase < 0) return COLMAP_PHASE_FAILED;
    if (phase >= 4) return COLMAP_PHASE_DONE;
    return static_cast<ColmapPhase>(phase);
}

float colmap_get_progress(ColmapReconstructPtr handle) {
    if (!handle || !handle->impl) {
        return 0.0f;
    }
    return handle->impl->GetProgressOnCurrentPhase();
}

int32_t colmap_is_running(ColmapReconstructPtr handle) {
    if (!handle || !handle->impl) {
        return 0;
    }
    return handle->impl->isRunning() ? 1 : 0;
}

int32_t colmap_is_paused(ColmapReconstructPtr handle) {
    if (!handle || !handle->impl) {
        return 0;
    }
    return handle->impl->isPaused() ? 1 : 0;
}

int32_t colmap_is_stopped(ColmapReconstructPtr handle) {
    if (!handle || !handle->impl) {
        return 1;
    }
    return handle->impl->isStopped() ? 1 : 0;
}

int32_t colmap_is_finished(ColmapReconstructPtr handle) {
    if (!handle || !handle->impl) {
        return 0;
    }
    return handle->impl->isFinished() ? 1 : 0;
}

ColmapSparsePointArray colmap_get_points3d(ColmapReconstructPtr handle, int32_t model_id) {
    ColmapSparsePointArray result = {nullptr, 0};
    
    if (!handle || !handle->impl) {
        return result;
    }
    
    // Use optimized zero-copy method
    size_t count = 0;
    auto* points = handle->impl->getPoints3DArray(model_id, &count);
    
    if (!points || count == 0) {
        return result;
    }
    
    // Zero-copy optimization: colmap::SparsePoint and ColmapSparsePoint have identical memory layout
    // Both are 16 bytes: {float x,y,z; uint8 r,g,b,a}
    // We can directly reinterpret the pointer instead of copying!
    static_assert(sizeof(colmap::SparsePoint) == sizeof(ColmapSparsePoint), 
                  "SparsePoint size mismatch");
    
    result.data = reinterpret_cast<ColmapSparsePoint*>(points);
    result.count = count;
    
    return result;
}

ColmapCameraTrackArray colmap_get_camera_tracks(ColmapReconstructPtr handle, int32_t model_id) {
    ColmapCameraTrackArray result = {nullptr, 0};
    
    if (!handle || !handle->impl) {
        return result;
    }
    
    // Use optimized method - one pass through data
    size_t count = 0;
    auto* tracks = handle->impl->getCameraTracksArray(model_id, &count);
    
    if (!tracks || count == 0) {
        return result;
    }
    
    result.data = static_cast<ColmapCameraTrack*>(malloc(sizeof(ColmapCameraTrack) * count));
    
    if (!result.data) {
        // malloc failed - cleanup and return empty result
        for (size_t i = 0; i < count; ++i) {
            tracks[i].~CameraTrack();
        }
        free(tracks);
        return result;  // {nullptr, 0}
    }
    
    result.count = count;
    
    for (size_t i = 0; i < count; ++i) {
        result.data[i].camera_id = tracks[i].camera_id;
        result.data[i].model_id = tracks[i].model_id;
        result.data[i].width = static_cast<uint32_t>(tracks[i].width);
        result.data[i].height = static_cast<uint32_t>(tracks[i].height);
        result.data[i].params_count = tracks[i].params.size();
        
        // CRITICAL: Initialize to nullptr first to prevent heap corruption
        result.data[i].params = nullptr;
        
        if (result.data[i].params_count > 0) {
            result.data[i].params = static_cast<double*>(malloc(sizeof(double) * result.data[i].params_count));
            if (result.data[i].params) {
                memcpy(result.data[i].params, tracks[i].params.data(), sizeof(double) * result.data[i].params_count);
            } else {
                // If malloc fails, reset params_count to 0 to be consistent
                result.data[i].params_count = 0;
            }
        }
    }
    
    // ✨ Critical: Must call destructors before freeing malloc'd memory!
    // The tracks array was allocated with malloc but contains C++ objects (std::vector)
    for (size_t i = 0; i < count; ++i) {
        tracks[i].~CameraTrack();  // Explicitly call destructor to free std::vector memory
    }
    free(tracks);
    
    return result;
}

ColmapImageTrackArray colmap_get_image_tracks(ColmapReconstructPtr handle, int32_t model_id) {
    ColmapImageTrackArray result = {nullptr, 0};
    
    if (!handle || !handle->impl) {
        return result;
    }
    
    // Use optimized method - one pass through data
    size_t count = 0;
    auto* tracks = handle->impl->getImageTracksArray(model_id, &count);
    
    if (!tracks || count == 0) {
        return result;
    }
    
    result.data = static_cast<ColmapImageTrack*>(malloc(sizeof(ColmapImageTrack) * count));
    
    if (!result.data) {
        // malloc failed - cleanup and return empty result
        for (size_t i = 0; i < count; ++i) {
            tracks[i].~ImageTrack();
        }
        free(tracks);
        return result;  // {nullptr, 0}
    }
    
    result.count = count;
    
    for (size_t i = 0; i < count; ++i) {
        result.data[i].image_id = tracks[i].image_id;
        result.data[i].camera_id = tracks[i].camera_id;
        
        // Copy name string - CRITICAL: Initialize to nullptr first to prevent heap corruption
        result.data[i].name = nullptr;
        if (!tracks[i].name.empty()) {
            result.data[i].name = static_cast<char*>(malloc(tracks[i].name.size() + 1));
            if (result.data[i].name) {
                strcpy(result.data[i].name, tracks[i].name.c_str());
            }
        }
        
        // Rotation quaternion (xyzw in C++ vec4 -> wxyz for standard quaternion)
        result.data[i].qw = tracks[i].rotation.w;
        result.data[i].qx = tracks[i].rotation.x;
        result.data[i].qy = tracks[i].rotation.y;
        result.data[i].qz = tracks[i].rotation.z;
        
        // Translation
        result.data[i].tx = tracks[i].translation.x;
        result.data[i].ty = tracks[i].translation.y;
        result.data[i].tz = tracks[i].translation.z;
    }
    
    // ✨ Critical: Must call destructors before freeing malloc'd memory!
    // The tracks array was allocated with malloc but contains C++ objects (std::string)
    for (size_t i = 0; i < count; ++i) {
        tracks[i].~ImageTrack();  // Explicitly call destructor to free std::string memory
    }
    free(tracks);
    
    return result;
}

void colmap_get_camera_and_image_tracks(
    ColmapReconstructPtr handle,
    int32_t model_id,
    ColmapCameraTrackArray* out_camera_array,
    ColmapImageTrackArray* out_image_array) {
    
    // Initialize outputs
    if (!out_camera_array || !out_image_array) {
        return;
    }
    
    out_camera_array->data = nullptr;
    out_camera_array->count = 0;
    out_image_array->data = nullptr;
    out_image_array->count = 0;
    
    if (!handle || !handle->impl) {
        return;
    }
    
    // Get both arrays atomically in a single lock
    auto result = handle->impl->getCameraAndImageTracksArray(model_id);
    
    // Early return if no data
    if (!result.cameras && !result.images) {
        return;
    }
    
    // Convert cameras
    if (result.cameras && result.camera_count > 0) {
        out_camera_array->data = static_cast<ColmapCameraTrack*>(malloc(sizeof(ColmapCameraTrack) * result.camera_count));
        
        if (!out_camera_array->data) {
            // Cleanup on malloc failure
            for (size_t i = 0; i < result.camera_count; ++i) {
                result.cameras[i].~CameraTrack();
            }
            free(result.cameras);
            
            if (result.images) {
                for (size_t i = 0; i < result.image_count; ++i) {
                    result.images[i].~ImageTrack();
                }
                free(result.images);
            }
            return;
        }
        
        out_camera_array->count = result.camera_count;
        
        for (size_t i = 0; i < result.camera_count; ++i) {
            out_camera_array->data[i].camera_id = result.cameras[i].camera_id;
            out_camera_array->data[i].model_id = result.cameras[i].model_id;
            out_camera_array->data[i].width = static_cast<uint32_t>(result.cameras[i].width);
            out_camera_array->data[i].height = static_cast<uint32_t>(result.cameras[i].height);
            out_camera_array->data[i].params_count = result.cameras[i].params.size();
            
            out_camera_array->data[i].params = nullptr;
            
            if (out_camera_array->data[i].params_count > 0) {
                out_camera_array->data[i].params = static_cast<double*>(malloc(sizeof(double) * out_camera_array->data[i].params_count));
                if (out_camera_array->data[i].params) {
                    memcpy(out_camera_array->data[i].params, result.cameras[i].params.data(), sizeof(double) * out_camera_array->data[i].params_count);
                } else {
                    out_camera_array->data[i].params_count = 0;
                }
            }
        }
        
        // Cleanup source cameras
        for (size_t i = 0; i < result.camera_count; ++i) {
            result.cameras[i].~CameraTrack();
        }
        free(result.cameras);
    }
    
    // Convert images
    if (result.images && result.image_count > 0) {
        out_image_array->data = static_cast<ColmapImageTrack*>(malloc(sizeof(ColmapImageTrack) * result.image_count));
        
        if (!out_image_array->data) {
            // Cleanup on malloc failure
            for (size_t i = 0; i < result.image_count; ++i) {
                result.images[i].~ImageTrack();
            }
            free(result.images);
            
            // Also cleanup already-allocated cameras
            colmap_free_camera_tracks(out_camera_array);
            return;
        }
        
        out_image_array->count = result.image_count;
        
        for (size_t i = 0; i < result.image_count; ++i) {
            out_image_array->data[i].image_id = result.images[i].image_id;
            out_image_array->data[i].camera_id = result.images[i].camera_id;
            
            out_image_array->data[i].name = nullptr;
            if (!result.images[i].name.empty()) {
                out_image_array->data[i].name = static_cast<char*>(malloc(result.images[i].name.size() + 1));
                if (out_image_array->data[i].name) {
                    strcpy(out_image_array->data[i].name, result.images[i].name.c_str());
                }
            }
            
            out_image_array->data[i].qw = result.images[i].rotation.w;
            out_image_array->data[i].qx = result.images[i].rotation.x;
            out_image_array->data[i].qy = result.images[i].rotation.y;
            out_image_array->data[i].qz = result.images[i].rotation.z;
            
            out_image_array->data[i].tx = result.images[i].translation.x;
            out_image_array->data[i].ty = result.images[i].translation.y;
            out_image_array->data[i].tz = result.images[i].translation.z;
        }
        
        // Cleanup source images
        for (size_t i = 0; i < result.image_count; ++i) {
            result.images[i].~ImageTrack();
        }
        free(result.images);
    }
}

void colmap_free_points(ColmapSparsePointArray* arr) {
    if (arr && arr->data) {
        free(arr->data);
        arr->data = nullptr;
        arr->count = 0;
    }
}

void colmap_free_camera_tracks(ColmapCameraTrackArray* arr) {
    if (arr && arr->data) {
        for (size_t i = 0; i < arr->count; ++i) {
            if (arr->data[i].params) {
                free(arr->data[i].params);
            }
        }
        free(arr->data);
        arr->data = nullptr;
        arr->count = 0;
    }
}

void colmap_free_image_tracks(ColmapImageTrackArray* arr) {
    if (arr && arr->data) {
        for (size_t i = 0; i < arr->count; ++i) {
            if (arr->data[i].name) {
                free(arr->data[i].name);
            }
        }
        free(arr->data);
        arr->data = nullptr;
        arr->count = 0;
    }
}

void colmap_free_string(char* str) {
    if (str) {
        free(str);
    }
}

} // extern "C"

