#include "sparse_reconstruct_controller.h"

#include "colmap/controllers/feature_extraction.h"
#include "colmap/controllers/feature_matching.h"
#include "colmap/controllers/incremental_pipeline.h"
#include "colmap/controllers/hierarchical_pipeline.h"
#include "colmap/controllers/option_manager.h"
#include "colmap/image/undistortion.h"
#include "colmap/mvs/fusion.h"
#include "colmap/mvs/meshing.h"
#include "colmap/mvs/patch_match.h"
#include "colmap/util/logging.h"
#include "colmap/util/misc.h"
// #include "glomap/controllers/global_mapper.h"
// #include "glomap/controllers/option_manager.h"
// #include "glomap/io/colmap_io.h"
// #include "glomap/types.h"

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
  option_manager_.image_reader->image_names = options_.image_names;
  option_manager_.mapper->image_names = {options_.image_names.begin(),
                                         options_.image_names.end()};
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

  option_manager_.feature_extraction->num_threads = options_.num_threads;
  option_manager_.feature_matching->num_threads = options_.num_threads;
  option_manager_.sequential_pairing->num_threads = options_.num_threads;
  option_manager_.vocab_tree_pairing->num_threads = options_.num_threads;
  option_manager_.mapper->num_threads = options_.num_threads;
  option_manager_.poisson_meshing->num_threads = options_.num_threads;

  option_manager_.two_view_geometry->ransac_options.random_seed =
      options_.random_seed;
  option_manager_.mapper->random_seed = options_.random_seed;

  ImageReaderOptions& reader_options = *option_manager_.image_reader;
  reader_options.image_path = *option_manager_.image_path;
  reader_options.as_rgb = option_manager_.feature_extraction->RequiresRGB();
  if (!options_.mask_path.empty()) {
    reader_options.mask_path = options_.mask_path;
    option_manager_.image_reader->mask_path = options_.mask_path;
    option_manager_.stereo_fusion->mask_path = options_.mask_path;
  }
  reader_options.single_camera = options_.single_camera;
  reader_options.single_camera_per_folder = options_.single_camera_per_folder;
  reader_options.camera_model = options_.camera_model;
  reader_options.camera_params = options_.camera_params;

  option_manager_.feature_extraction->use_gpu = options_.use_gpu;
  option_manager_.feature_matching->use_gpu = options_.use_gpu;
  option_manager_.mapper->ba_use_gpu = options_.use_gpu;
  option_manager_.bundle_adjustment->use_gpu = options_.use_gpu;

  option_manager_.feature_extraction->gpu_index = options_.gpu_index;
  option_manager_.feature_matching->gpu_index = options_.gpu_index;
  option_manager_.patch_match_stereo->gpu_index = options_.gpu_index;
  option_manager_.mapper->ba_gpu_index = options_.gpu_index;
  option_manager_.bundle_adjustment->gpu_index = options_.gpu_index;

  if (options_.extraction) {
    feature_extractor_ =
        CreateFeatureExtractorController(*option_manager_.database_path,
                                         reader_options,
                                         *option_manager_.feature_extraction);
  }

  if (options_.matching) {
    exhaustive_matcher_ =
        CreateExhaustiveFeatureMatcher(*option_manager_.exhaustive_pairing,
                                       *option_manager_.feature_matching,
                                       *option_manager_.two_view_geometry,
                                       *option_manager_.database_path);

    if (!options_.vocab_tree_path.empty()) {
      option_manager_.sequential_pairing->loop_detection = true;
      option_manager_.sequential_pairing->vocab_tree_path =
          options_.vocab_tree_path;
    }

    sequential_matcher_ =
        CreateSequentialFeatureMatcher(*option_manager_.sequential_pairing,
                                       *option_manager_.feature_matching,
                                       *option_manager_.two_view_geometry,
                                       *option_manager_.database_path);

    if (!options_.vocab_tree_path.empty()) {
      option_manager_.vocab_tree_pairing->vocab_tree_path =
          options_.vocab_tree_path;
      vocab_tree_matcher_ =
          CreateVocabTreeFeatureMatcher(*option_manager_.vocab_tree_pairing,
                                        *option_manager_.feature_matching,
                                        *option_manager_.two_view_geometry,
                                        *option_manager_.database_path);
    }
  }
}

void SparseReconstructionController::Stop() {
  if (active_thread_ != nullptr) {
    active_thread_->Stop();
  }
  Thread::Stop();
}

void SparseReconstructionController::Pause() {
  if (active_thread_ != nullptr) {
    active_thread_->Pause();
  }
  Thread::Pause();
}

void SparseReconstructionController::Resume() {
  if (active_thread_ != nullptr) {
    active_thread_->Resume();
  }
  Thread::Resume();
}

int SparseReconstructionController::GetSparseReconstructPhase() {
  return status_phase;
}

float SparseReconstructionController::GetProgressOnCurrentPhase() {
  if (status_phase == 1) {
    return feature_extractor_->GetProgress();
  } else if (status_phase == 2) {
    return matcher->GetProgress();
  } else if (status_phase == 3) {
    // if (options_.use_glomapper)
    //   return global_mapper ? global_mapper->GetProgress() : 0.0f;
    if(!options_.use_hierachy)
      return incremental_mapper ? incremental_mapper->GetController()->GetProgress() : 0.0f;
    return hierarchical_mapper ? hierarchical_mapper->GetController()->GetProgress() : 0.0f;
  }
  return 1.0f;
}

void SparseReconstructionController::Run() {
  try{
    if (IsStopped()) {
      return;
    }
    status_phase = 1;
    RunFeatureExtraction();

    if (IsStopped()) {
      return;
    }
    RunFeatureMatching();

    if (IsStopped()) {
      return;
    }
    status_phase = 3;
    RunSparseMapper();
    std::cout << "RunSparseMapper done\n";
    if (IsStopped()) {
      return;
    }
    status_phase = 4;
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
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
  // Thread* matcher = nullptr;
  if (options_.data_type == DataType::VIDEO) {
    matcher = sequential_matcher_.get();
  } else if (options_.data_type == DataType::INDIVIDUAL ||
             options_.data_type == DataType::INTERNET) {
    auto database = Database::Open(*option_manager_.database_path);
    const size_t num_images = database->NumImages();
    if (options_.vocab_tree_path.empty() || num_images < 200) {
      matcher = exhaustive_matcher_.get();
    } else {
      matcher = vocab_tree_matcher_.get();
    }
  }
  status_phase = 2;
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
  // if( options_.use_glomapper)
  // {
  //   glomap::GlobalMapperOptions glomapOptions;

  //   glomap::ViewGraph view_graph;

  //   const colmap::Database database(*option_manager_.database_path);
  //   glomap::ConvertDatabaseToGlomap(database, view_graph, cameras, images);
  //   std::cout << "glomap mapper start\n";
  //   //glomapOptions.opt_gp.constraint_type = GlobalPositionerOptions::ONLY_POINTS;
  //   glomapOptions.opt_track.max_num_tracks = 50000;
  //   global_mapper = std::make_shared<glomap::GlobalMapper>(glomapOptions);
  
  //   global_mapper->Solve(database, view_graph, cameras, images, tracks);
  //   ConvertGlomapToColmapReconstructionManager(
  //                               reconstruction_manager_,
  //                               cameras,
  //                               images,
  //                               tracks,
  //                               *option_manager_.image_path);
  //   LOG(INFO) << "Export to COLMAP reconstruction done";
  // }
  // else if(options_.use_hierachy)
  if(options_.use_hierachy)
  { 
    HierarchicalPipeline::Options mapper_options;
    mapper_options.database_path = *option_manager_.database_path;
    mapper_options.image_path = *option_manager_.image_path;
    mapper_options.incremental_options = *option_manager_.mapper;
    hierarchical_mapper =  std::make_unique<ControllerThread<HierarchicalPipeline>>(
        std::make_shared<HierarchicalPipeline>(
        mapper_options, reconstruction_manager_));
    hierarchical_mapper->GetController()->SetCheckIfStoppedFunc([&]() { return IsStopped(); });
    active_thread_ = hierarchical_mapper.get();
  }
  else
  {
    incremental_mapper = std::make_unique<ControllerThread<IncrementalPipeline>>(
        std::make_shared<IncrementalPipeline>(option_manager_.mapper,
                                    *option_manager_.image_path,
                                    *option_manager_.database_path,
                                    reconstruction_manager_));
    incremental_mapper->GetController()->SetCheckIfStoppedFunc([&]() { return IsStopped(); });
    active_thread_ = incremental_mapper.get();
  }
  active_thread_->Start();
  active_thread_->Wait();
  std::cout << "sparse reconstruction done\n";
  if( options_.output_sparse_points && !IsStopped()){
    std::cout << "output sparse points\n";
    CreateDirIfNotExists(sparse_path);
    reconstruction_manager_->Write(sparse_path);
    option_manager_.Write(JoinPaths(sparse_path, "project.ini"));
  }
}
std::unordered_map<point3D_t, struct Point3D> empty_pts;
const std::unordered_map<point3D_t, struct Point3D>& SparseReconstructionController::Points3D(int id) const
{
  if( id < reconstruction_manager_->Size()){
    return reconstruction_manager_->Get(id)->Points3D();
  }
  return empty_pts;
}

std::unordered_map<camera_t, struct Camera> empty_cams;
const std::unordered_map<camera_t, struct Camera>& SparseReconstructionController::Cameras(int id) const
{
  if( id < reconstruction_manager_->Size()){
    return reconstruction_manager_->Get(id)->Cameras();
  }
  return empty_cams;
}

std::unordered_map<image_t, class Image> empty_imags;

const std::unordered_map<image_t, class Image>&
SparseReconstructionController::Images(int id) const {
  if (id < reconstruction_manager_->Size()) {
    // return reconstruction_manager_->Get(id)->Images();
    empty_imags.clear();
    for(auto reg_id : reconstruction_manager_->Get(id)->RegImageIds()){
      empty_imags[reg_id] = reconstruction_manager_->Get(id)->Image(reg_id);
    }
    return empty_imags;
  }
  return empty_imags;
}

}  // namespace colmap
