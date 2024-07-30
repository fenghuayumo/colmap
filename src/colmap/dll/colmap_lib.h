#pragma once
#ifdef _WIN32
#define COLMAP_HIDDEN
#if defined(COLMAP_BUILD_SHARED_LIBS)
#define COLMAP_EXPORT __declspec(dllexport)
#define COLMAP_IMPORT __declspec(dllimport)
#else
#define COLMAP_EXPORT
#define COLMAP_IMPORT
#endif
#else // _WIN32
#if defined(__GNUC__)
#define COLMAP_EXPORT __attribute__((__visibility__("default")))
#define COLMAP_HIDDEN __attribute__((__visibility__("hidden")))
#else // defined(__GNUC__)
#define COLMAP_EXPORT
#define COLMAP_HIDDEN
#endif // defined(__GNUC__)
#define COLMAP_IMPORT COLMAP_EXPORT
#endif // _WIN32

#ifdef NO_EXPORT
#undef COLMAP_EXPORT
#define COLMAP_EXPORT
#endif
#ifdef COLMAP_BUILD_MAIN_LIB
#define COLMAP_API COLMAP_EXPORT
#else
#define COLMAP_API COLMAP_IMPORT
#endif

#include <string>
#include <memory>

namespace colmap {
    class AutomaticReconstructionController;
}

struct COLMAP_API ColmapSparseReconstruct {
    ~ColmapSparseReconstruct();
    enum class Quality {
        Low,Medium,High
    };
    struct Option {
        std::string image_path;
        std::string workspace_path;
        int gpu_index = -1;
        bool video = true;
        bool use_hierachy = true;
        Quality quality = Quality::Low;
        bool use_gpu = true;
        std::string camera_model= "SIMPLE_PINHOLE";
    }option;
    int GetSparseReconstructPhase();
    float GetProgressOnCurrentPhase();
    auto run()->bool;

    std::shared_ptr<colmap::AutomaticReconstructionController>  controller_;
};
