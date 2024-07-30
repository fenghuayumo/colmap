#include "sparse_reconstruct_controller.h"

#include "colmap/controllers/feature_extraction.h"
#include "colmap/controllers/feature_matching.h"
#include "colmap/controllers/incremental_mapper.h"
#include "colmap/controllers/hierarchical_mapper.h"
#include "colmap/controllers/option_manager.h"
#include "colmap/image/undistortion.h"
#include "colmap/mvs/fusion.h"
#include "colmap/mvs/meshing.h"
#include "colmap/mvs/patch_match.h"
#include "colmap/util/logging.h"
#include "colmap/util/misc.h"
#include "glomap/controllers/global_mapper.h"
#include "glomap/controllers/option_manager.h"
#include "glomap/io/colmap_io.h"
#include "glomap/types.h"

namespace colmap {

SparseReconstructionController::SparseReconstructionController(
    const Options& options,
    std::shared_ptr<ReconstructionManager> reconstruction_manager)
    : options_(options),
      reconstruction_manager_(std::move(reconstruction_manager)),
      active_thread_(nullptr) {
  THROW_CHECK_DIR_EXISTS(options_.workspace_path);
  THROW_CHECK_DIR_EXISTS(options_.image_path);
  THROW_CHECK_NOTNULL(reconstruction_manager_);

  option_manager_.AddAllOptions();

  *option_manager_.image_path = options_.image_path;
  *option_manager_.database_path =
      JoinPaths(options_.workspace_path, "database.db");

  if (options_.data_type == DataType::VIDEO) {
    option_manager_.ModifyForVideoData();
  } else if (options_.data_type == DataType::INDIVIDUAL) {
    option_manager_.ModifyForIndividualData();
  } else if (options_.data_type == DataType::INTERNET) {
    option_manager_.ModifyForInternetData();
  } else {
    LOG(FATAL_THROW) << "Data type not supported";
  }

  THROW_CHECK(ExistsCameraModelWithName(options_.camera_model));

  if (options_.quality == Quality::LOW) {
    option_manager_.ModifyForLowQuality();
  } else if (options_.quality == Quality::MEDIUM) {
    option_manager_.ModifyForMediumQuality();
  } else if (options_.quality == Quality::HIGH) {
    option_manager_.ModifyForHighQuality();
  } else if (options_.quality == Quality::EXTREME) {
    option_manager_.ModifyForExtremeQuality();
  }

  option_manager_.sift_extraction->num_threads = options_.num_threads;
  option_manager_.sift_matching->num_threads = options_.num_threads;
  option_manager_.mapper->num_threads = options_.num_threads;
  option_manager_.poisson_meshing->num_threads = options_.num_threads;

  ImageReaderOptions& reader_options = *option_manager_.image_reader;
  reader_options.database_path = *option_manager_.database_path;
  reader_options.image_path = *option_manager_.image_path;
  if (!options_.mask_path.empty()) {
    reader_options.mask_path = options_.mask_path;
    option_manager_.image_reader->mask_path = options_.mask_path;
    option_manager_.stereo_fusion->mask_path = options_.mask_path;
  }
  reader_options.single_camera = options_.single_camera;
  reader_options.single_camera_per_folder = options_.single_camera_per_folder;
  reader_options.camera_model = options_.camera_model;
  reader_options.camera_params = options_.camera_params;

  option_manager_.sift_extraction->use_gpu = options_.use_gpu;
  option_manager_.sift_matching->use_gpu = options_.use_gpu;

  option_manager_.sift_extraction->gpu_index = options_.gpu_index;
  option_manager_.sift_matching->gpu_index = options_.gpu_index;
  option_manager_.patch_match_stereo->gpu_index = options_.gpu_index;

  feature_extractor_ = CreateFeatureExtractorController(
      reader_options, *option_manager_.sift_extraction);

  exhaustive_matcher_ =
      CreateExhaustiveFeatureMatcher(*option_manager_.exhaustive_matching,
                                     *option_manager_.sift_matching,
                                     *option_manager_.two_view_geometry,
                                     *option_manager_.database_path);

  if (!options_.vocab_tree_path.empty()) {
    option_manager_.sequential_matching->loop_detection = true;
    option_manager_.sequential_matching->vocab_tree_path =
        options_.vocab_tree_path;
  }

  sequential_matcher_ =
      CreateSequentialFeatureMatcher(*option_manager_.sequential_matching,
                                     *option_manager_.sift_matching,
                                     *option_manager_.two_view_geometry,
                                     *option_manager_.database_path);

  if (!options_.vocab_tree_path.empty()) {
    option_manager_.vocab_tree_matching->vocab_tree_path =
        options_.vocab_tree_path;
    vocab_tree_matcher_ =
        CreateVocabTreeFeatureMatcher(*option_manager_.vocab_tree_matching,
                                      *option_manager_.sift_matching,
                                      *option_manager_.two_view_geometry,
                                      *option_manager_.database_path);
  }
}

void SparseReconstructionController::Stop() {
  if (active_thread_ != nullptr) {
    active_thread_->Stop();
  }
  Thread::Stop();
}

int SparseReconstructionController::GetSparseReconstructPhase() {
  return status_phase;
}

float SparseReconstructionController::GetProgressOnCurrentPhase() {
  if (status_phase == 1) {
    return feature_extractor_->GetProgress();
  } else if (status_phase == 2) {
    return exhaustive_matcher_->GetProgress();
  } else if (status_phase == 3) {
    if(!options_.use_hierachy)
        return incremental_mapper->GetProgress();
    return hierarchical_mapper->GetProgress();
  }
  return 1.0f;
}

void SparseReconstructionController::Run() {
  if (IsStopped()) {
    return;
  }
  status_phase = 1;
  RunFeatureExtraction();

  if (IsStopped()) {
    return;
  }
  status_phase = 2;
  RunFeatureMatching();

  if (IsStopped()) {
    return;
  }

  RunSparseMapper();

  if (IsStopped()) {
    return;
  }
  status_phase = 4;
}

void SparseReconstructionController::RunFeatureExtraction() {
  THROW_CHECK_NOTNULL(feature_extractor_);
  active_thread_ = feature_extractor_.get();
  feature_extractor_->Start();
  feature_extractor_->Wait();
  // feature_extractor_.reset();
  active_thread_ = nullptr;
}

void SparseReconstructionController::RunFeatureMatching() {
  Thread* matcher = nullptr;
  if (options_.data_type == DataType::VIDEO) {
    matcher = sequential_matcher_.get();
  } else if (options_.data_type == DataType::INDIVIDUAL ||
             options_.data_type == DataType::INTERNET) {
    Database database(*option_manager_.database_path);
    const size_t num_images = database.NumImages();
    if (options_.vocab_tree_path.empty() || num_images < 200) {
      matcher = exhaustive_matcher_.get();
    } else {
      matcher = vocab_tree_matcher_.get();
    }
  }

  THROW_CHECK_NOTNULL(matcher);
  active_thread_ = matcher;
  matcher->Start();
  matcher->Wait();
  // exhaustive_matcher_.reset();
  // sequential_matcher_.reset();
  // vocab_tree_matcher_.reset();
  active_thread_ = nullptr;
}

void SparseReconstructionController::RunSparseMapper() {
  const auto sparse_path = JoinPaths(options_.workspace_path, "sparse");
  if (ExistsDir(sparse_path)) {
    auto dir_list = GetDirList(sparse_path);
    std::sort(dir_list.begin(), dir_list.end());
    if (dir_list.size() > 0) {
      LOG(WARNING)
          << "Skipping sparse reconstruction because it is already computed";
      for (const auto& dir : dir_list) {
        reconstruction_manager_->Read(dir);
      }
      return;
    }
  }

  status_phase = 3;
  if( options_.use_glomapper)
  {
    glomap::GlobalMapperOptions glomapOptions;

    glomap::ViewGraph view_graph;
    std::unordered_map<glomap::camera_t, glomap::Camera> cameras;
    std::unordered_map<glomap::image_t, glomap::Image> images;
    std::unordered_map<glomap::track_t, glomap::Track> tracks;

    const colmap::Database database(*option_manager_.database_path);
    glomap::ConvertDatabaseToGlomap(database, view_graph, cameras, images);

    glomap::GlobalMapper global_mapper(glomapOptions);
    global_mapper.Solve(database, view_graph, cameras, images, tracks);
    WriteGlomapReconstruction(sparse_path,
                              cameras,
                              images,
                              tracks,
                              "bin",
                              *option_manager_.image_path);
    LOG(INFO) << "Export to COLMAP reconstruction done";
  }
  else if(options_.use_hierachy)
  { 
    HierarchicalMapperController::Options mapper_options;
    mapper_options.database_path = *option_manager_.database_path;
    mapper_options.image_path = *option_manager_.image_path;
    mapper_options.incremental_options = *option_manager_.mapper;
    hierarchical_mapper = std::make_shared<HierarchicalMapperController>(mapper_options, reconstruction_manager_);
    hierarchical_mapper->SetCheckIfStoppedFunc([&]() { return IsStopped(); });
    hierarchical_mapper->Run();
  }
  else
  {
    incremental_mapper = std::make_shared<IncrementalMapperController>(option_manager_.mapper,
                                    *option_manager_.image_path,
                                    *option_manager_.database_path,
                                    reconstruction_manager_);
    incremental_mapper->SetCheckIfStoppedFunc([&]() { return IsStopped(); });
    incremental_mapper->Run();
  }
 
  CreateDirIfNotExists(sparse_path);
  reconstruction_manager_->Write(sparse_path);
  option_manager_.Write(JoinPaths(sparse_path, "project.ini"));
}

}  // namespace colmap
