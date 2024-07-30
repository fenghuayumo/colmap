
#pragma once

#include "colmap/controllers/option_manager.h"
#include "colmap/scene/reconstruction_manager.h"
#include "colmap/util/threading.h"

#include <memory>
#include <string>

namespace colmap {
class SparseReconstructionController : public Thread {
 public:
  enum class DataType { INDIVIDUAL, VIDEO, INTERNET };
  enum class Quality { LOW, MEDIUM, HIGH, EXTREME };
  enum class Mesher { POISSON, DELAUNAY };

  struct Options {
    // The path to the workspace folder in which all results are stored.
    std::string workspace_path;

    // The path to the image folder which are used as input.
    std::string image_path;

    // The path to the mask folder which are used as input.
    std::string mask_path;
    // The path to the vocabulary tree for feature matching.
    std::string vocab_tree_path;
    // The type of input data used to choose optimal mapper settings.
    DataType data_type = DataType::INDIVIDUAL;

    // Whether to perform low- or high-quality reconstruction.
    Quality quality = Quality::HIGH;

    // Whether to use shared intrinsics or not.
    bool single_camera = false;

    // Whether to use shared intrinsics or not for all images in the same
    // sub-folder.
    bool single_camera_per_folder = false;

    // Which camera model to use for images.
    std::string camera_model = "SIMPLE_RADIAL";

    // Initial camera params for all images.
    std::string camera_params;

    // The number of threads to use in all stages.
    int num_threads = -1;

    // Whether to use the GPU in feature extraction and matching.
    bool use_gpu = true;

    // Index of the GPU used for GPU stages. For multi-GPU computation,
    // you should separate multiple GPU indices by comma, e.g., "0,1,2,3".
    // By default, all GPUs will be used in all stages.
    std::string gpu_index = "-1";

    bool use_hierachy = true;
    bool use_glomapper = false;
  };

  SparseReconstructionController(
      const Options& options,
      std::shared_ptr<ReconstructionManager> reconstruction_manager);

  void Stop() override;

  int GetSparseReconstructPhase();
  float GetProgressOnCurrentPhase();
  int status_phase = 0;
 private:
  void Run() override;
  void RunFeatureExtraction();
  void RunFeatureMatching();
  void RunSparseMapper();

  const Options options_;
  OptionManager option_manager_;
  std::shared_ptr<ReconstructionManager> reconstruction_manager_;
  Thread* active_thread_;
  std::unique_ptr<Thread> feature_extractor_;
  std::unique_ptr<Thread> exhaustive_matcher_;
  std::unique_ptr<Thread> sequential_matcher_;
  std::unique_ptr<Thread> vocab_tree_matcher_;

  std::shared_ptr<class IncrementalMapperController> incremental_mapper;
  std::shared_ptr<class HierarchicalMapperController> hierarchical_mapper;
};

}  // namespace colmap
