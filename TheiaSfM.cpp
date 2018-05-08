//#include <theia/theia.h>
#include <theia/image/descriptor/descriptor_extractor.h>
#include <theia/image/descriptor/create_descriptor_extractor.h>
#include <theia/sfm/reconstruction.h>
#include <theia/sfm/reconstruction_estimator_options.h>
#include <theia/sfm/reconstruction_builder.h>
#include <theia/matching/create_feature_matcher.h>
#include <theia/io/read_calibration.h>
#include <theia/io/write_ply_file.h>
#include <theia/io/reconstruction_writer.h>
#include <theia/util/filesystem.h>
#include <theia/util/stringprintf.h>
#include <theia/sfm/colorize_reconstruction.h>
#include <GSLAM/core/GSLAM.h>
#include <GSLAM/core/HashMap.h>

using namespace theia;


using theia::DescriptorExtractorType;
using theia::FeatureDensity;
using theia::GlobalPositionEstimatorType;
using theia::GlobalRotationEstimatorType;
using theia::LossFunctionType;
using theia::MatchingStrategy;
using theia::OptimizeIntrinsicsType;
using theia::ReconstructionEstimatorType;

inline DescriptorExtractorType StringToDescriptorExtractorType(
    const std::string& descriptor) {
  if (descriptor == "SIFT") {
    return DescriptorExtractorType::SIFT;
  } else if (descriptor == "AKAZE") {
    return DescriptorExtractorType::AKAZE;
  } else {
    LOG(FATAL) << "Invalid DescriptorExtractor specified. Using SIFT instead.";
    return DescriptorExtractorType::SIFT;
  }
}

inline FeatureDensity StringToFeatureDensity(
    const std::string& feature_density) {
  if (feature_density == "SPARSE") {
    return FeatureDensity::SPARSE;
  } else if (feature_density == "NORMAL") {
    return FeatureDensity::NORMAL;
  } else if (feature_density == "DENSE") {
    return FeatureDensity::DENSE;
  } else {
    LOG(FATAL) << "Invalid feature density requested. Please use SPARSE, "
                  "NORMAL, or DENSE.";
    return FeatureDensity::NORMAL;
  }
}

inline MatchingStrategy StringToMatchingStrategyType(
    const std::string& matching_strategy) {
  if (matching_strategy == "BRUTE_FORCE") {
    return MatchingStrategy::BRUTE_FORCE;
  } else if (matching_strategy == "CASCADE_HASHING") {
    return MatchingStrategy::CASCADE_HASHING;
  } else {
    LOG(FATAL)
        << "Invalid matching strategy specified. Using BRUTE_FORCE instead.";
    return MatchingStrategy::BRUTE_FORCE;
  }
}

inline ReconstructionEstimatorType StringToReconstructionEstimatorType(
    const std::string& reconstruction_estimator) {
  if (reconstruction_estimator == "GLOBAL") {
    return ReconstructionEstimatorType::GLOBAL;
  } else if (reconstruction_estimator == "INCREMENTAL") {
    return ReconstructionEstimatorType::INCREMENTAL;
  } else if (reconstruction_estimator == "HYBRID") {
    return ReconstructionEstimatorType::HYBRID;
  } else {
    LOG(FATAL)
        << "Invalid reconstruction estimator type. Using GLOBAL instead.";
    return ReconstructionEstimatorType::GLOBAL;
  }
}

inline GlobalRotationEstimatorType StringToRotationEstimatorType(
    const std::string& rotation_estimator) {
  if (rotation_estimator == "ROBUST_L1L2") {
    return GlobalRotationEstimatorType::ROBUST_L1L2;
  } else if (rotation_estimator == "NONLINEAR") {
    return GlobalRotationEstimatorType::NONLINEAR;
  } else if (rotation_estimator == "LINEAR") {
    return GlobalRotationEstimatorType::LINEAR;
  } else {
    LOG(FATAL)
        << "Invalid rotation estimator type. Using ROBUST_L1L2 instead.";
    return GlobalRotationEstimatorType::ROBUST_L1L2;
  }
}

inline GlobalPositionEstimatorType StringToPositionEstimatorType(
    const std::string& position_estimator) {
  if (position_estimator == "NONLINEAR") {
    return GlobalPositionEstimatorType::NONLINEAR;
  } else if (position_estimator == "LINEAR_TRIPLET") {
    return GlobalPositionEstimatorType::LINEAR_TRIPLET;
  } else if (position_estimator == "LEAST_UNSQUARED_DEVIATION") {
    return GlobalPositionEstimatorType::LEAST_UNSQUARED_DEVIATION;
  } else {
    LOG(FATAL)
        << "Invalid position estimator type. Using NONLINEAR instead.";
    return GlobalPositionEstimatorType::NONLINEAR;
  }
}

inline OptimizeIntrinsicsType StringToOptimizeIntrinsicsType(
    const std::string& intrinsics_to_optimize) {
  CHECK_GT(intrinsics_to_optimize.size(), 0)
      << "You must specify which camera intrinsics parametrs to optimize. "
         "Please specify NONE, ALL, or any bitwise OR combination (without "
         "spaces) of FOCAL_LENGTH, PRINCIPAL_POINTS, RADIAL_DISTORTION, "
         "ASPECT_RATIO, SKEW";

  // Split the string by the '|' token.
  std::stringstream ss(intrinsics_to_optimize);
  std::vector<std::string> intrinsics;
  std::string item;
  const char delimiter = '|';
  while (std::getline(ss, item, delimiter)) {
    intrinsics.emplace_back(item);
  }

  CHECK_GT(intrinsics.size(), 0)
      << "Could not decipher any valid camera intrinsics.";
  if (intrinsics[0] == "NONE") {
    return OptimizeIntrinsicsType::NONE;
  }

  if (intrinsics[0] == "ALL") {
    return OptimizeIntrinsicsType::ALL;
  }

  // Compile all intrinsics we wish to optimize.
  OptimizeIntrinsicsType intrinsic_params = OptimizeIntrinsicsType::NONE;
  for (int i = 0; i < intrinsics.size(); i++) {
    if (intrinsics[i] == "FOCAL_LENGTH") {
      intrinsic_params |= OptimizeIntrinsicsType::FOCAL_LENGTH;
    } else if (intrinsics[i] == "ASPECT_RATIO") {
      intrinsic_params |= OptimizeIntrinsicsType::ASPECT_RATIO;
    } else if (intrinsics[i] == "SKEW") {
      intrinsic_params |= OptimizeIntrinsicsType::SKEW;
    } else if (intrinsics[i] == "PRINCIPAL_POINTS") {
      intrinsic_params |= OptimizeIntrinsicsType::PRINCIPAL_POINTS;
    } else if (intrinsics[i] == "RADIAL_DISTORTION") {
      intrinsic_params |= OptimizeIntrinsicsType::RADIAL_DISTORTION;
    } else if (intrinsics[i] == "TANGENTIAL_DISTORTION") {
      intrinsic_params |= OptimizeIntrinsicsType::TANGENTIAL_DISTORTION;
    } else {
      LOG(FATAL) << "Invalid option for intrinsics_to_optimize: "
                 << intrinsics[i];
    }
  }
  return intrinsic_params;
}

inline LossFunctionType StringToLossFunction(
    const std::string& loss_function_type) {
  if (loss_function_type == "NONE") {
    return LossFunctionType::TRIVIAL;
  } else if (loss_function_type == "HUBER") {
    return LossFunctionType::HUBER;
  } else if (loss_function_type == "SOFTLONE") {
    return LossFunctionType::SOFTLONE;
  } else if (loss_function_type == "CAUCHY") {
    return LossFunctionType::CAUCHY;
  } else if (loss_function_type == "ARCTAN") {
    return LossFunctionType::ARCTAN;
  } else if (loss_function_type == "TUKEY") {
    return LossFunctionType::TUKEY;
  } else {
    LOG(FATAL) << "Invalid option for bundle_adjustment_robust_loss_function";
  }
}

ReconstructionBuilderOptions SetReconstructionBuilderOptions(GSLAM::Svar& var) {
  ReconstructionBuilderOptions options;
  options.num_threads = var.GetInt("num_threads",1);
  options.output_matches_file = var.GetString("output_matches_file","./matches.matches");

  options.descriptor_type = StringToDescriptorExtractorType(var.GetString("descriptor","SIFT"));
  options.feature_density = StringToFeatureDensity(var.GetString("feature_density","NORMAL"));
  options.matching_options.match_out_of_core = var.GetInt("match_out_of_core",1);
  options.matching_options.keypoints_and_descriptors_output_dir =
      var.GetString("keypoints_and_descriptors_output_dir","./features");
  options.matching_options.cache_capacity =
      var.GetInt("matching_max_num_images_in_cache",128);
  options.matching_strategy =
      StringToMatchingStrategyType(var.GetString("matching_strategy","CASCADE_HASHING"));
  options.matching_options.lowes_ratio = var.GetDouble("lowes_ratio",0.8);
  options.matching_options.keep_only_symmetric_matches =
      var.GetInt("keep_only_symmetric_matches",1);
  options.min_num_inlier_matches = var.GetInt("min_num_inliers_for_valid_match",30);
  options.matching_options.perform_geometric_verification = true;
  options.matching_options.geometric_verification_options
      .estimate_twoview_info_options.max_sampson_error_pixels =
      var.GetDouble("max_sampson_error_for_verified_match",4.0);
  options.matching_options.geometric_verification_options.bundle_adjustment =
      var.GetInt("bundle_adjust_two_view_geometry",1);
  options.matching_options.geometric_verification_options
      .triangulation_max_reprojection_error =
      var.GetDouble("triangulation_reprojection_error_pixels",15.0);
  options.matching_options.geometric_verification_options
      .min_triangulation_angle_degrees = var.GetDouble("min_triangulation_angle_degrees",4.);
  options.matching_options.geometric_verification_options
      .final_max_reprojection_error = var.GetDouble("max_reprojection_error_pixels",4.);

  options.min_track_length = var.GetInt("min_track_length",2);
  options.max_track_length = var.GetInt("max_track_length",50);

  // Reconstruction Estimator Options.
  theia::ReconstructionEstimatorOptions& reconstruction_estimator_options =
      options.reconstruction_estimator_options;
  reconstruction_estimator_options.min_num_two_view_inliers =
      var.GetInt("min_num_inliers_for_valid_match",30);
  reconstruction_estimator_options.num_threads = options.num_threads;
  reconstruction_estimator_options.intrinsics_to_optimize =
    StringToOptimizeIntrinsicsType(var.GetString("intrinsics_to_optimize","NONE"));
  options.reconstruct_largest_connected_component =var.GetInt("reconstruct_largest_connected_component",0);
  options.only_calibrated_views = var.GetInt("only_calibrated_views",0);
  reconstruction_estimator_options.max_reprojection_error_in_pixels =
      var.GetDouble("max_reprojection_error_pixels",4.);

  // Which type of SfM pipeline to use (e.g., incremental, global, etc.);
  reconstruction_estimator_options.reconstruction_estimator_type =
      StringToReconstructionEstimatorType(var.GetString("reconstruction_estimator","GLOBAL"));

  // Global SfM Options.
  reconstruction_estimator_options.global_rotation_estimator_type =
      StringToRotationEstimatorType(var.GetString("global_rotation_estimator","ROBUST_L1L2"));
  reconstruction_estimator_options.global_position_estimator_type =
      StringToPositionEstimatorType(var.GetString("global_position_estimator","NONLINEAR"));
  reconstruction_estimator_options.num_retriangulation_iterations =
      var.GetInt("retriangulation_iterations",1);
  reconstruction_estimator_options
      .refine_relative_translations_after_rotation_estimation =
      var.GetInt("refine_relative_translations_after_rotation_estimation",1);
  reconstruction_estimator_options.extract_maximal_rigid_subgraph =
      var.GetInt("extract_maximal_rigid_subgraph",0);
  reconstruction_estimator_options.filter_relative_translations_with_1dsfm =
      var.GetInt("filter_relative_translations_with_1dsfm",1);
  reconstruction_estimator_options
      .rotation_filtering_max_difference_degrees =
      var.GetDouble("post_rotation_filtering_degrees",5.);
  reconstruction_estimator_options.nonlinear_position_estimator_options
      .min_num_points_per_view =
      var.GetInt("position_estimation_min_num_tracks_per_view",0);
  reconstruction_estimator_options
      .refine_camera_positions_and_points_after_position_estimation =
      var.GetInt("refine_camera_positions_and_points_after_position_estimation",1);

  // Incremental SfM Options.
  reconstruction_estimator_options
      .absolute_pose_reprojection_error_threshold =
      var.GetDouble("absolute_pose_reprojection_error_threshold",4.);
  reconstruction_estimator_options.min_num_absolute_pose_inliers =
      var.GetInt("min_num_absolute_pose_inliers",30);
  reconstruction_estimator_options
      .full_bundle_adjustment_growth_percent =
      var.GetDouble("full_bundle_adjustment_growth_percent",5.);
  reconstruction_estimator_options.partial_bundle_adjustment_num_views =
      var.GetInt("partial_bundle_adjustment_num_views",20);

  // Triangulation options (used by all SfM pipelines).
  reconstruction_estimator_options.min_triangulation_angle_degrees =
      var.GetDouble("min_triangulation_angle_degrees",4.);
  reconstruction_estimator_options
      .triangulation_max_reprojection_error_in_pixels =
      var.GetDouble("triangulation_reprojection_error_pixels",15.);
  reconstruction_estimator_options.bundle_adjust_tracks =
      var.GetInt("bundle_adjust_tracks",1);

  // Bundle adjustment options (used by all SfM pipelines).
  reconstruction_estimator_options.bundle_adjustment_loss_function_type =
      StringToLossFunction(var.GetString("bundle_adjustment_robust_loss_function","NONE"));
  reconstruction_estimator_options.bundle_adjustment_robust_loss_width =
      var.GetDouble("bundle_adjustment_robust_loss_width",10.);

  // Track subsampling options.
  reconstruction_estimator_options.subsample_tracks_for_bundle_adjustment =
      var.GetInt("subsample_tracks_for_bundle_adjustment",false);
  reconstruction_estimator_options
      .track_subset_selection_long_track_length_threshold =
      var.GetInt("track_subset_selection_long_track_length_threshold",10);
  reconstruction_estimator_options.track_selection_image_grid_cell_size_pixels =
      var.GetInt("track_selection_image_grid_cell_size_pixels",100);
  reconstruction_estimator_options.min_num_optimized_tracks_per_view =
      var.GetInt("min_num_optimized_tracks_per_view",100);
  return options;
}

class TheiaSfM : public GSLAM::SLAM
{
public:
    TheiaSfM(){
        setMap(GSLAM::MapPtr(new GSLAM::HashMap()));
    }

    virtual ~TheiaSfM(){
    }

    virtual std::string type()const{return "TheiaSfM";}

    virtual void  call(const std::string& command,void* arg=NULL){
        if(command=="finalize") finalize();
    }

    virtual bool valid()const{return true;}
    virtual bool isDrawable()const{return false;}

    virtual bool track(GSLAM::FramePtr& frame){
        if(!frame) finalize();
        std::string imagePath;
        frame->call("GetImagePath",&imagePath);
        if(imagePath.empty()){
            LOG(ERROR)<<"Need frame to implement GetImagePath in function MapFrame::call";
            return false;
        }

        imageFolder=GSLAM::Svar::getFolderPath(imagePath);

        if(!_reconstruction_builder)
        {
            const ReconstructionBuilderOptions options =SetReconstructionBuilderOptions(svar);
            _reconstruction_builder=SPtr<theia::ReconstructionBuilder>(new theia::ReconstructionBuilder(options));

            std::string FLAGS_calibration_file=svar.GetString("calibration_file","");
            if (FLAGS_calibration_file.size() != 0) {
              CHECK(theia::ReadCalibration(FLAGS_calibration_file,
                                           &camera_intrinsics_prior))
                  << "Could not read calibration file.";
            }

            // Add images with possible calibration. When the intrinsics group id is
            // invalid, the reconstruction builder will assume that the view does not
            // share its intrinsics with any other views.
            bool FLAGS_shared_calibration=svar.GetInt("shared_calibration",0);
            if (FLAGS_shared_calibration) {
              intrinsics_group_id = 0;
            }
        }
        std::string image_filename;
        CHECK(theia::GetFilenameFromFilepath(imagePath, true, &image_filename));

        const theia::CameraIntrinsicsPrior* image_camera_intrinsics_prior =
          FindOrNull(camera_intrinsics_prior, image_filename);
        if (image_camera_intrinsics_prior != nullptr) {
          CHECK(_reconstruction_builder->AddImageWithCameraIntrinsicsPrior(
              imagePath, *image_camera_intrinsics_prior, intrinsics_group_id));
        } else {
          CHECK(_reconstruction_builder->AddImage(imagePath, intrinsics_group_id));
        }
        return true;
    }

    class MapPoint : public GSLAM::MapPoint
    {
    public:
        MapPoint(const theia::Track& track)
            : GSLAM::MapPoint(svar.GetInt("PointID")++,hnormalized(track.Point())){
            color=track.Color();
        }

        static GSLAM::Point3d hnormalized(Eigen::Vector4d pt){
            Eigen::Vector3d p3d=pt.hnormalized();
            return GSLAM::Point3d(p3d[0],p3d[1],p3d[2]);
        }

        virtual GSLAM::ColorType  getColor()const{return color;}

        GSLAM::ColorType color;
    };

    class MapFrame : public GSLAM::MapFrame
    {
    public:
        MapFrame(const theia::View& view)
            :GSLAM::MapFrame(svar.GetInt("FrameID")++,0){
            GSLAM::Point3d r=view.Camera().GetOrientationAsAngleAxis();
            GSLAM::Point3d t=view.Camera().GetPosition();

            setPose(GSLAM::SE3(GSLAM::SO3::exp(r),t));
        }

        void setDepth(double depth){_depth=depth;}
        virtual double getMedianDepth(){return _depth;}
        double _depth=-1;
    };

    virtual bool finalize(){
        // Extract and match features.
        CHECK(_reconstruction_builder->ExtractAndMatchFeatures());
        std::vector<Reconstruction*> reconstructions;
        CHECK(_reconstruction_builder->BuildReconstruction(&reconstructions))
            << "Could not create a reconstruction.";

        for (int i = 0; i < reconstructions.size(); i++) {
          const std::string output_file =
              theia::StringPrintf("%s-%d", svar.GetString("output_reconstruction","reconstruction").c_str(), i);
          LOG(INFO) << "Writing reconstruction " << i << " to " << output_file;
          CHECK(theia::WriteReconstruction(*reconstructions[i], output_file))
              << "Could not write reconstruction to file.";
          CHECK(WritePlyFile(output_file+".ply",
                             *reconstructions[i],
                             svar.GetInt("min_num_observations_per_point",3)))
              << "Could not write out PLY file.";
          GSLAM::MapPtr map=getMap();
          Reconstruction& reconstruction=*reconstructions[i];
          theia::ColorizeReconstruction(imageFolder+"/",
                                        svar.GetInt("num_threads",1),
                                        &reconstruction);
          for (const TrackId track_id : reconstruction.TrackIds()) {
            const Track& track = *reconstruction.Track(track_id);
            map->insertMapPoint(GSLAM::PointPtr(new MapPoint(track)));
          }
          for (const ViewId view_id : reconstruction.ViewIds()) {
            const View& view = *reconstruction.View(view_id);
            if (!view.IsEstimated()) {
              continue;
            }
            SPtr<MapFrame> frame(new MapFrame(view));
            map->insertMapFrame(frame);


            std::vector<TrackId> ids=view.TrackIds();
            std::vector<double>  depths;
            for(TrackId id:ids){
                Eigen::Vector3d p3d=reconstruction.Track(id)->Point().hnormalized();
                GSLAM::Point3d  p=frame->getPose().inverse()*p3d;
                depths.push_back(p.z);
            }

            std::sort(depths.begin(),depths.end());

            if(depths.size()) frame->setDepth(depths[depths.size()/2]);

          }
        }

        if(_handle) _handle->handle(getMap());




        return true;
    }

    // Load calibration file if it is provided.
    std::string                imageFolder;
    std::unordered_map<std::string, theia::CameraIntrinsicsPrior>
        camera_intrinsics_prior;
    theia::CameraIntrinsicsGroupId intrinsics_group_id =
        theia::kInvalidCameraIntrinsicsGroupId;
    SPtr<theia::ReconstructionBuilder> _reconstruction_builder;
};

USE_GSLAM_PLUGIN(TheiaSfM);
