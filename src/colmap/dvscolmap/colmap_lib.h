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
#include <vector>
namespace colmap {
    class SparseReconstructionController;
}
#ifndef COLMAP_SPARSEPOINT
#define COLMAP_SPARSEPOINT

namespace colmap{
    
    template<typename T>
    struct vec3 {
        T x,y,z;
    };

    template <typename T>
    struct vec4 {
      T x, y, z,w;
    };

    struct SparsePoint{
        vec3<float> xyz;
        vec4<unsigned char> color;
    };
    struct CameraTrack {
        uint32_t camera_id;
        int model_id;
        size_t width = 0;
        size_t height = 0;
        std::vector<double> params;
    };

    struct ImageTrack {
      uint32_t image_id;
      std::string name;
      uint32_t camera_id;
      vec4<float> rotation;
      vec3<float>  translation;
    };
}
#endif

struct COLMAP_API ColmapSparseReconstruct {
    ~ColmapSparseReconstruct();
    enum class Quality {
        Low,Medium,High,Extreme
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
        bool use_glomap = false;
        bool output_sparse_points = false;
    }option;
    int GetSparseReconstructPhase();
    float GetProgressOnCurrentPhase();
    auto run()->bool;
    auto getPoints3D(int id) const -> std::vector<colmap::SparsePoint>;
    auto getCameraTracks(int id) const -> std::vector<
        colmap::CameraTrack>;
    auto getImageTracks(int id) const -> std::vector<colmap::ImageTrack>;
    //std::vector<colmap::SparsePoint>   points;
    //std::vector<colmap::CameraTrack>    cameras;  
    std::shared_ptr<colmap::SparseReconstructionController>  controller_;
};
