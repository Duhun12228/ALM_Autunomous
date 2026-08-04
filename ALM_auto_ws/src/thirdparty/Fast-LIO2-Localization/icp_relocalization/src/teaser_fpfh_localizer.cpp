#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <teaser/registration.h>

#include "icp_relocalization/fpfh_pipeline.hpp"

namespace ir = icp_relocalization;

class TeaserFpfhLocalizer : public rclcpp::Node
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TeaserFpfhLocalizer()
        : Node("teaser_fpfh_localizer")
    {
        declare_parameters();
        read_parameters();
        validate_parameters();
        load_map_and_database();

        result_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/icp_result", 10);
        teaser_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/teaser_pose", 10);
        aligned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/teaser_aligned_cloud", 10);
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_, rclcpp::SensorDataQoS().keep_last(1),
            std::bind(&TeaserFpfhLocalizer::cloud_callback, this, std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "FPFH+TEASER++ ready | DB=%zu features, accumulate=%d frames, "
            "voxel=%.2f normal=%.2f FPFH=%.2f, range=%.1fm, z=[%.2f, %.2f]",
            map_features_->size(), accum_frames_, feature_params_.voxel,
            feature_params_.normal_radius, feature_params_.feature_radius,
            feature_params_.max_range, feature_params_.z_min, feature_params_.z_max);
    }

private:
    struct ValidationScore
    {
        double inlier_ratio = 0.0;
        double inlier_rmse = std::numeric_limits<double>::infinity();
        std::size_t inliers = 0;
    };

    struct RegistrationResult
    {
        bool valid = false;
        Eigen::Matrix4f teaser_pose = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f refined_pose = Eigen::Matrix4f::Identity();
        std::size_t matches = 0;
        std::size_t clique_inliers = 0;
        double teaser_seconds = 0.0;
        double gicp_fitness = std::numeric_limits<double>::infinity();
        ValidationScore teaser_score;
        ValidationScore final_score;
    };

    void declare_parameters()
    {
        declare_parameter<std::string>("map_path", "");
        declare_parameter<std::string>("fpfh_db_prefix", "");
        declare_parameter<std::string>("lidar_topic", "/livox/lidar");
        declare_parameter<std::string>("map_frame_id", "map");
        declare_parameter<int>("accum_frames", 10);
        declare_parameter<int>("num_threads", 4);
        declare_parameter<double>("feature_voxel", 0.5);
        declare_parameter<double>("normal_radius", 1.0);
        declare_parameter<double>("feature_radius", 2.5);
        declare_parameter<double>("scan_min_range", 0.5);
        declare_parameter<double>("scan_max_range", 10.0);
        declare_parameter<double>("z_min", -0.35);
        declare_parameter<double>("z_max", 1.0);
        declare_parameter<double>("min_curvature", 0.0);
        declare_parameter<int>("max_scan_features", 1500);
        declare_parameter<double>("feature_ratio_threshold", 0.95);
        declare_parameter<int>("min_feature_matches", 20);
        declare_parameter<int>("max_feature_matches", 400);
        declare_parameter<double>("teaser_noise_bound", 0.5);
        declare_parameter<double>("teaser_rotation_gnc_factor", 1.4);
        declare_parameter<int>("teaser_rotation_max_iterations", 100);
        declare_parameter<double>("teaser_max_clique_time_limit", 2.0);
        declare_parameter<bool>("teaser_exact_clique", false);
        declare_parameter<int>("min_teaser_inliers", 6);
        declare_parameter<double>("validation_map_voxel", 0.20);
        declare_parameter<double>("validation_scan_voxel", 0.20);
        declare_parameter<double>("validation_inlier_distance", 0.50);
        declare_parameter<double>("validation_min_inlier_ratio", 0.20);
        declare_parameter<double>("validation_max_rmse", 0.35);
        declare_parameter<double>("max_abs_roll_deg", 15.0);
        declare_parameter<double>("max_abs_pitch_deg", 15.0);
        declare_parameter<double>("min_result_z", -1.0);
        declare_parameter<double>("max_result_z", 1.0);
        declare_parameter<double>("local_map_radius", 12.0);
        declare_parameter<double>("gicp_map_voxel", 0.25);
        declare_parameter<double>("gicp_scan_voxel", 0.20);
        declare_parameter<double>("gicp_max_correspondence", 1.0);
        declare_parameter<int>("gicp_max_iterations", 60);
        declare_parameter<double>("gicp_fitness_threshold", 0.30);
        declare_parameter<int>("consistent_result_count", 2);
        declare_parameter<double>("consistency_translation", 0.50);
        declare_parameter<double>("consistency_rotation_deg", 5.0);
        declare_parameter<bool>("verify_map_fingerprint", true);
    }

    void read_parameters()
    {
        map_path_ = get_parameter("map_path").as_string();
        db_prefix_ = get_parameter("fpfh_db_prefix").as_string();
        lidar_topic_ = get_parameter("lidar_topic").as_string();
        map_frame_ = get_parameter("map_frame_id").as_string();
        accum_frames_ = static_cast<int>(get_parameter("accum_frames").as_int());
        feature_params_.num_threads = static_cast<int>(get_parameter("num_threads").as_int());
        feature_params_.voxel = get_parameter("feature_voxel").as_double();
        feature_params_.normal_radius = get_parameter("normal_radius").as_double();
        feature_params_.feature_radius = get_parameter("feature_radius").as_double();
        feature_params_.min_range = get_parameter("scan_min_range").as_double();
        feature_params_.max_range = get_parameter("scan_max_range").as_double();
        feature_params_.z_min = get_parameter("z_min").as_double();
        feature_params_.z_max = get_parameter("z_max").as_double();
        feature_params_.min_curvature = get_parameter("min_curvature").as_double();
        feature_params_.max_features =
            static_cast<int>(get_parameter("max_scan_features").as_int());
        feature_ratio_threshold_ = get_parameter("feature_ratio_threshold").as_double();
        min_feature_matches_ = static_cast<int>(get_parameter("min_feature_matches").as_int());
        max_feature_matches_ = static_cast<int>(get_parameter("max_feature_matches").as_int());
        teaser_noise_bound_ = get_parameter("teaser_noise_bound").as_double();
        teaser_rotation_gnc_factor_ = get_parameter("teaser_rotation_gnc_factor").as_double();
        teaser_rotation_max_iterations_ =
            static_cast<int>(get_parameter("teaser_rotation_max_iterations").as_int());
        teaser_max_clique_time_limit_ =
            get_parameter("teaser_max_clique_time_limit").as_double();
        teaser_exact_clique_ = get_parameter("teaser_exact_clique").as_bool();
        min_teaser_inliers_ = static_cast<int>(get_parameter("min_teaser_inliers").as_int());
        validation_map_voxel_ = get_parameter("validation_map_voxel").as_double();
        validation_scan_voxel_ = get_parameter("validation_scan_voxel").as_double();
        validation_inlier_distance_ = get_parameter("validation_inlier_distance").as_double();
        validation_min_inlier_ratio_ = get_parameter("validation_min_inlier_ratio").as_double();
        validation_max_rmse_ = get_parameter("validation_max_rmse").as_double();
        max_abs_roll_ = get_parameter("max_abs_roll_deg").as_double() * M_PI / 180.0;
        max_abs_pitch_ = get_parameter("max_abs_pitch_deg").as_double() * M_PI / 180.0;
        min_result_z_ = get_parameter("min_result_z").as_double();
        max_result_z_ = get_parameter("max_result_z").as_double();
        local_map_radius_ = get_parameter("local_map_radius").as_double();
        gicp_map_voxel_ = get_parameter("gicp_map_voxel").as_double();
        gicp_scan_voxel_ = get_parameter("gicp_scan_voxel").as_double();
        gicp_max_correspondence_ = get_parameter("gicp_max_correspondence").as_double();
        gicp_max_iterations_ = static_cast<int>(get_parameter("gicp_max_iterations").as_int());
        gicp_fitness_threshold_ = get_parameter("gicp_fitness_threshold").as_double();
        consistent_result_count_ =
            static_cast<int>(get_parameter("consistent_result_count").as_int());
        consistency_translation_ = get_parameter("consistency_translation").as_double();
        consistency_rotation_ =
            get_parameter("consistency_rotation_deg").as_double() * M_PI / 180.0;
        verify_map_fingerprint_ = get_parameter("verify_map_fingerprint").as_bool();
    }

    void validate_parameters() const
    {
        if (map_path_.empty() || db_prefix_.empty()) {
            throw std::invalid_argument("map_path and fpfh_db_prefix are required");
        }
        if (accum_frames_ < 1 || feature_params_.voxel <= 0.0 ||
            feature_params_.normal_radius <= feature_params_.voxel ||
            feature_params_.feature_radius <= feature_params_.normal_radius ||
            feature_ratio_threshold_ <= 0.0 || feature_ratio_threshold_ >= 1.0 ||
            min_feature_matches_ < 3 || max_feature_matches_ < min_feature_matches_ ||
            teaser_noise_bound_ <= 0.0 || consistent_result_count_ < 1) {
            throw std::invalid_argument("invalid FPFH/TEASER parameter combination");
        }
    }

    static double metadata_double(
        const std::map<std::string, std::string> &metadata, const std::string &key)
    {
        const auto item = metadata.find(key);
        if (item == metadata.end()) {
            throw std::runtime_error("FPFH DB metadata missing key: " + key);
        }
        return std::stod(item->second);
    }

    void load_map_and_database()
    {
        const auto metadata = ir::read_metadata(db_prefix_ + ".meta");
        if (metadata.empty()) {
            throw std::runtime_error("cannot read FPFH DB metadata: " + db_prefix_ + ".meta");
        }
        if (!ir::close_parameter(metadata_double(metadata, "voxel"), feature_params_.voxel) ||
            !ir::close_parameter(
                metadata_double(metadata, "normal_radius"), feature_params_.normal_radius) ||
            !ir::close_parameter(
                metadata_double(metadata, "feature_radius"), feature_params_.feature_radius) ||
            !ir::close_parameter(metadata_double(metadata, "z_min"), feature_params_.z_min) ||
            !ir::close_parameter(metadata_double(metadata, "z_max"), feature_params_.z_max) ||
            !ir::close_parameter(
                metadata_double(metadata, "min_curvature"), feature_params_.min_curvature)) {
            throw std::runtime_error(
                "FPFH DB parameters do not match runtime preprocessing parameters");
        }
        const auto fingerprint = metadata.find("map_fingerprint");
        if (verify_map_fingerprint_ && fingerprint != metadata.end()) {
            const std::string actual = ir::file_fingerprint(map_path_);
            if (actual != fingerprint->second) {
                throw std::runtime_error(
                    "map PCD fingerprint differs from the FPFH DB map fingerprint");
            }
        }

        map_points_.reset(new ir::Cloud);
        map_normals_.reset(new ir::NormalCloud);
        map_features_.reset(new ir::FeatureCloud);
        if (pcl::io::loadPCDFile(db_prefix_ + "_points.pcd", *map_points_) < 0 ||
            pcl::io::loadPCDFile(db_prefix_ + "_normals.pcd", *map_normals_) < 0 ||
            pcl::io::loadPCDFile(db_prefix_ + "_fpfh.pcd", *map_features_) < 0) {
            throw std::runtime_error("failed to load one or more FPFH DB files");
        }
        if (map_points_->size() != map_features_->size() ||
            map_points_->size() != map_normals_->size() || map_points_->size() < 30) {
            throw std::runtime_error("FPFH DB point/normal/feature sizes are inconsistent");
        }

        ir::CloudPtr full_map(new ir::Cloud);
        if (pcl::io::loadPCDFile(map_path_, *full_map) < 0) {
            throw std::runtime_error("failed to load map PCD: " + map_path_);
        }
        ir::FeatureParams map_filter = feature_params_;
        map_filter.voxel = validation_map_voxel_;
        map_filter.min_range = 0.0;
        map_filter.max_range = 0.0;
        validation_map_ = ir::filter_cloud(full_map, map_filter);
        validation_tree_.setInputCloud(validation_map_);
        RCLCPP_INFO(
            get_logger(), "Map validation cloud: raw=%zu voxel=%.2f -> %zu points",
            full_map->size(), validation_map_voxel_, validation_map_->size());
    }

    RegistrationResult register_cloud(const ir::CloudConstPtr &raw_cloud)
    {
        RegistrationResult result;
        const auto total_start = std::chrono::steady_clock::now();
        const auto feature_start = std::chrono::steady_clock::now();
        const ir::FeatureSet source = ir::compute_features(raw_cloud, feature_params_);
        const double feature_seconds = elapsed(feature_start);
        RCLCPP_INFO(
            get_logger(),
            "[FPFH] raw=%zu filtered=%zu features=%zu time=%.3fs",
            source.input_size, source.filtered_size, source.features->size(), feature_seconds);
        if (source.features->size() < static_cast<std::size_t>(min_feature_matches_)) {
            RCLCPP_WARN(get_logger(), "[FPFH] valid source features are insufficient");
            return result;
        }

        const auto match_start = std::chrono::steady_clock::now();
        const std::vector<ir::FeatureMatch> matches = ir::match_features(
            source.features, map_features_, feature_ratio_threshold_, max_feature_matches_);
        result.matches = matches.size();
        RCLCPP_INFO(
            get_logger(), "[MATCH] mutual+ratio matches=%zu (required >= %d), time=%.3fs",
            matches.size(), min_feature_matches_, elapsed(match_start));
        if (matches.size() < static_cast<std::size_t>(min_feature_matches_)) {
            RCLCPP_WARN(get_logger(), "[MATCH] rejected: too few reliable correspondences");
            return result;
        }

        Eigen::Matrix<double, 3, Eigen::Dynamic> source_matrix(3, matches.size());
        Eigen::Matrix<double, 3, Eigen::Dynamic> target_matrix(3, matches.size());
        for (std::size_t i = 0; i < matches.size(); ++i) {
            const auto &src = (*source.points)[static_cast<std::size_t>(matches[i].source)];
            const auto &dst = (*map_points_)[static_cast<std::size_t>(matches[i].target)];
            source_matrix.col(static_cast<Eigen::Index>(i)) =
                Eigen::Vector3d(src.x, src.y, src.z);
            target_matrix.col(static_cast<Eigen::Index>(i)) =
                Eigen::Vector3d(dst.x, dst.y, dst.z);
        }

        teaser::RobustRegistrationSolver::Params params;
        params.noise_bound = teaser_noise_bound_;
        params.cbar2 = 1.0;
        params.estimate_scaling = false;
        params.rotation_estimation_algorithm =
            teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
        params.rotation_gnc_factor = teaser_rotation_gnc_factor_;
        params.rotation_max_iterations =
            static_cast<std::size_t>(teaser_rotation_max_iterations_);
        params.rotation_cost_threshold = 1e-6;
        params.rotation_tim_graph =
            teaser::RobustRegistrationSolver::INLIER_GRAPH_FORMULATION::CHAIN;
        params.inlier_selection_mode = teaser_exact_clique_
            ? teaser::RobustRegistrationSolver::INLIER_SELECTION_MODE::PMC_EXACT
            : teaser::RobustRegistrationSolver::INLIER_SELECTION_MODE::PMC_HEU;
        params.max_clique_time_limit = teaser_max_clique_time_limit_;

        const auto teaser_start = std::chrono::steady_clock::now();
        teaser::RobustRegistrationSolver solver(params);
        try {
            solver.solve(source_matrix, target_matrix);
        } catch (const std::exception &error) {
            RCLCPP_ERROR(get_logger(), "[TEASER] solver exception: %s", error.what());
            return result;
        }
        result.teaser_seconds = elapsed(teaser_start);
        const teaser::RegistrationSolution solution = solver.getSolution();
        result.clique_inliers = solver.getInlierMaxClique().size();
        if (!solution.valid || !solution.rotation.allFinite() ||
            !solution.translation.allFinite() ||
            result.clique_inliers < static_cast<std::size_t>(min_teaser_inliers_)) {
            RCLCPP_WARN(
                get_logger(), "[TEASER] invalid solution or too few clique inliers: %zu/%d",
                result.clique_inliers, min_teaser_inliers_);
            return result;
        }

        result.teaser_pose.setIdentity();
        result.teaser_pose.block<3, 3>(0, 0) = solution.rotation.cast<float>();
        result.teaser_pose.block<3, 1>(0, 3) = solution.translation.cast<float>();
        if (!plausible_pose(result.teaser_pose, "TEASER")) {
            return result;
        }
        publish_pose(teaser_pose_pub_, result.teaser_pose);

        const ir::CloudPtr validation_scan = downsample(raw_cloud, validation_scan_voxel_);
        result.teaser_score = validate_pose(validation_scan, result.teaser_pose);
        log_registration("TEASER", result.teaser_pose, result.teaser_score,
                         result.matches, result.clique_inliers, result.teaser_seconds);
        if (!accepted_score(result.teaser_score)) {
            RCLCPP_WARN(get_logger(), "[TEASER] rejected by independent map-overlap validation");
            return result;
        }

        if (!refine_gicp(raw_cloud, result.teaser_pose, result.refined_pose,
                         result.gicp_fitness)) {
            return result;
        }
        if (!plausible_pose(result.refined_pose, "GICP")) {
            return result;
        }
        result.final_score = validate_pose(validation_scan, result.refined_pose);
        log_registration("GICP", result.refined_pose, result.final_score,
                         result.matches, result.clique_inliers, elapsed(total_start));
        if (!accepted_score(result.final_score) ||
            !std::isfinite(result.gicp_fitness) ||
            result.gicp_fitness >= gicp_fitness_threshold_) {
            RCLCPP_WARN(
                get_logger(),
                "[GICP] rejected: fitness=%.6f (required < %.3f), overlap=%.1f%%, rmse=%.3f",
                result.gicp_fitness, gicp_fitness_threshold_,
                100.0 * result.final_score.inlier_ratio, result.final_score.inlier_rmse);
            return result;
        }

        result.valid = true;
        publish_aligned_cloud(validation_scan, result.refined_pose);
        return result;
    }

    ir::CloudPtr downsample(const ir::CloudConstPtr &input, double voxel_size) const
    {
        ir::CloudPtr output(new ir::Cloud);
        pcl::VoxelGrid<ir::PointT> voxel;
        voxel.setInputCloud(input);
        const float leaf = static_cast<float>(voxel_size);
        voxel.setLeafSize(leaf, leaf, leaf);
        voxel.filter(*output);
        return output;
    }

    ValidationScore validate_pose(
        const ir::CloudConstPtr &source, const Eigen::Matrix4f &pose) const
    {
        ValidationScore score;
        if (source->empty()) {
            return score;
        }
        const double threshold_sq =
            validation_inlier_distance_ * validation_inlier_distance_;
        double squared_sum = 0.0;
        std::vector<int> indices(1);
        std::vector<float> distances(1);
        for (const auto &point : *source) {
            const Eigen::Vector4f transformed =
                pose * Eigen::Vector4f(point.x, point.y, point.z, 1.0f);
            ir::PointT query;
            query.x = transformed.x();
            query.y = transformed.y();
            query.z = transformed.z();
            if (validation_tree_.nearestKSearch(query, 1, indices, distances) > 0 &&
                distances[0] <= threshold_sq) {
                ++score.inliers;
                squared_sum += distances[0];
            }
        }
        score.inlier_ratio =
            static_cast<double>(score.inliers) / static_cast<double>(source->size());
        if (score.inliers > 0) {
            score.inlier_rmse = std::sqrt(squared_sum / static_cast<double>(score.inliers));
        }
        return score;
    }

    bool accepted_score(const ValidationScore &score) const
    {
        return score.inlier_ratio >= validation_min_inlier_ratio_ &&
               std::isfinite(score.inlier_rmse) &&
               score.inlier_rmse < validation_max_rmse_;
    }

    bool refine_gicp(
        const ir::CloudConstPtr &raw_cloud, const Eigen::Matrix4f &initial,
        Eigen::Matrix4f &refined, double &fitness) const
    {
        pcl::CropBox<ir::PointT> crop;
        crop.setInputCloud(validation_map_);
        crop.setMin(Eigen::Vector4f(
            initial(0, 3) - local_map_radius_, initial(1, 3) - local_map_radius_,
            initial(2, 3) - 3.0f, 1.0f));
        crop.setMax(Eigen::Vector4f(
            initial(0, 3) + local_map_radius_, initial(1, 3) + local_map_radius_,
            initial(2, 3) + 3.0f, 1.0f));
        ir::CloudPtr cropped(new ir::Cloud);
        crop.filter(*cropped);
        const ir::CloudPtr target = downsample(cropped, gicp_map_voxel_);
        ir::FeatureParams scan_filter = feature_params_;
        scan_filter.voxel = gicp_scan_voxel_;
        const ir::CloudPtr source = ir::filter_cloud(raw_cloud, scan_filter);
        if (source->size() < 30 || target->size() < 100) {
            RCLCPP_WARN(
                get_logger(), "[GICP] insufficient points: source=%zu target=%zu",
                source->size(), target->size());
            return false;
        }

        pcl::GeneralizedIterativeClosestPoint<ir::PointT, ir::PointT> gicp;
        gicp.setInputSource(source);
        gicp.setInputTarget(target);
        gicp.setMaxCorrespondenceDistance(gicp_max_correspondence_);
        gicp.setMaximumIterations(gicp_max_iterations_);
        gicp.setTransformationEpsilon(1e-4);
        gicp.setEuclideanFitnessEpsilon(1e-5);
        ir::Cloud aligned;
        gicp.align(aligned, initial);
        refined = gicp.getFinalTransformation();
        fitness = gicp.getFitnessScore(gicp_max_correspondence_);
        if (!gicp.hasConverged()) {
            RCLCPP_WARN(get_logger(), "[GICP] did not converge");
            return false;
        }
        return true;
    }

    bool plausible_pose(const Eigen::Matrix4f &pose, const char *stage) const
    {
        if (!pose.allFinite()) {
            RCLCPP_WARN(get_logger(), "[%s] non-finite pose", stage);
            return false;
        }
        const Eigen::Matrix3f rotation = pose.block<3, 3>(0, 0);
        if (std::abs(rotation.determinant() - 1.0f) > 0.05f) {
            RCLCPP_WARN(get_logger(), "[%s] invalid rotation determinant", stage);
            return false;
        }
        const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
        const double pitch = std::asin(std::max(-1.0f, std::min(1.0f, -rotation(2, 0))));
        const double z = pose(2, 3);
        if (std::abs(roll) > max_abs_roll_ || std::abs(pitch) > max_abs_pitch_ ||
            z < min_result_z_ || z > max_result_z_) {
            RCLCPP_WARN(
                get_logger(), "[%s] implausible pose: z=%.3f roll=%.1fdeg pitch=%.1fdeg",
                stage, z, roll * 180.0 / M_PI, pitch * 180.0 / M_PI);
            return false;
        }
        return true;
    }

    static double pose_yaw(const Eigen::Matrix4f &pose)
    {
        return std::atan2(pose(1, 0), pose(0, 0));
    }

    bool consistent_with_previous(const Eigen::Matrix4f &pose) const
    {
        const Eigen::Vector3f delta =
            pose.block<3, 1>(0, 3) - previous_pose_.block<3, 1>(0, 3);
        const Eigen::Matrix3f delta_rotation =
            previous_pose_.block<3, 3>(0, 0).transpose() * pose.block<3, 3>(0, 0);
        const double cosine = std::max(
            -1.0, std::min(1.0, (static_cast<double>(delta_rotation.trace()) - 1.0) / 2.0));
        const double angle = std::acos(cosine);
        RCLCPP_INFO(
            get_logger(), "[CONSISTENCY] translation=%.3fm rotation=%.2fdeg",
            delta.norm(), angle * 180.0 / M_PI);
        return delta.norm() <= consistency_translation_ && angle <= consistency_rotation_;
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
    {
        if (finished_ || processing_) {
            return;
        }
        ir::Cloud frame;
        pcl::fromROSMsg(*message, frame);
        accumulated_->reserve(accumulated_->size() + frame.size());
        for (const auto &point : frame) {
            if (pcl::isFinite(point)) {
                accumulated_->push_back(point);
            }
        }
        ++accumulated_frames_;
        RCLCPP_INFO(
            get_logger(), "[ACCUM] frame %d/%d, points=%zu (robot must remain stationary)",
            accumulated_frames_, accum_frames_, accumulated_->size());
        if (accumulated_frames_ < accum_frames_) {
            return;
        }

        processing_ = true;
        ++attempt_;
        RCLCPP_INFO(
            get_logger(), "[ATTEMPT %d] global registration started with %zu accumulated points",
            attempt_, accumulated_->size());
        RegistrationResult result;
        try {
            result = register_cloud(accumulated_);
        } catch (const std::exception &error) {
            RCLCPP_ERROR(get_logger(), "registration exception: %s", error.what());
        }
        accumulated_.reset(new ir::Cloud);
        accumulated_frames_ = 0;
        processing_ = false;

        if (!result.valid) {
            consistent_count_ = 0;
            RCLCPP_WARN(get_logger(), "[ATTEMPT %d] rejected; accumulating a fresh scan", attempt_);
            return;
        }

        if (consistent_count_ == 0 || !consistent_with_previous(result.refined_pose)) {
            previous_pose_ = result.refined_pose;
            consistent_count_ = 1;
        } else {
            previous_pose_ = result.refined_pose;
            ++consistent_count_;
        }
        RCLCPP_INFO(
            get_logger(), "[CONSISTENCY] accepted %d/%d",
            consistent_count_, consistent_result_count_);
        if (consistent_count_ < consistent_result_count_) {
            return;
        }

        publish_pose(result_pub_, result.refined_pose);
        finished_ = true;
        RCLCPP_INFO(
            get_logger(),
            "FPFH+TEASER++ localization succeeded; /icp_result published | "
            "x=%.3f y=%.3f z=%.3f yaw=%.1fdeg",
            result.refined_pose(0, 3), result.refined_pose(1, 3),
            result.refined_pose(2, 3), pose_yaw(result.refined_pose) * 180.0 / M_PI);
    }

    void publish_pose(
        const rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr &publisher,
        const Eigen::Matrix4f &pose) const
    {
        geometry_msgs::msg::PoseWithCovarianceStamped message;
        message.header.stamp = now();
        message.header.frame_id = map_frame_;
        message.pose.pose.position.x = pose(0, 3);
        message.pose.pose.position.y = pose(1, 3);
        message.pose.pose.position.z = pose(2, 3);
        const Eigen::Quaternionf quaternion(pose.block<3, 3>(0, 0));
        message.pose.pose.orientation.x = quaternion.x();
        message.pose.pose.orientation.y = quaternion.y();
        message.pose.pose.orientation.z = quaternion.z();
        message.pose.pose.orientation.w = quaternion.w();
        publisher->publish(message);
    }

    void publish_aligned_cloud(
        const ir::CloudConstPtr &source, const Eigen::Matrix4f &pose) const
    {
        ir::Cloud transformed;
        pcl::transformPointCloud(*source, transformed, pose);
        sensor_msgs::msg::PointCloud2 message;
        pcl::toROSMsg(transformed, message);
        message.header.stamp = now();
        message.header.frame_id = map_frame_;
        aligned_pub_->publish(message);
    }

    void log_registration(
        const char *stage, const Eigen::Matrix4f &pose, const ValidationScore &score,
        std::size_t matches, std::size_t clique, double seconds) const
    {
        RCLCPP_INFO(
            get_logger(),
            "[%s] matches=%zu clique=%zu overlap=%.1f%% rmse=%.3f time=%.3fs | "
            "x=%.3f y=%.3f z=%.3f yaw=%.1fdeg",
            stage, matches, clique, 100.0 * score.inlier_ratio, score.inlier_rmse,
            seconds, pose(0, 3), pose(1, 3), pose(2, 3),
            pose_yaw(pose) * 180.0 / M_PI);
    }

    static double elapsed(const std::chrono::steady_clock::time_point &start)
    {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start).count();
    }

    std::string map_path_;
    std::string db_prefix_;
    std::string lidar_topic_;
    std::string map_frame_;
    ir::FeatureParams feature_params_;
    int accum_frames_ = 10;
    double feature_ratio_threshold_ = 0.95;
    int min_feature_matches_ = 20;
    int max_feature_matches_ = 400;
    double teaser_noise_bound_ = 0.5;
    double teaser_rotation_gnc_factor_ = 1.4;
    int teaser_rotation_max_iterations_ = 100;
    double teaser_max_clique_time_limit_ = 2.0;
    bool teaser_exact_clique_ = false;
    int min_teaser_inliers_ = 6;
    double validation_map_voxel_ = 0.2;
    double validation_scan_voxel_ = 0.2;
    double validation_inlier_distance_ = 0.5;
    double validation_min_inlier_ratio_ = 0.2;
    double validation_max_rmse_ = 0.35;
    double max_abs_roll_ = 0.0;
    double max_abs_pitch_ = 0.0;
    double min_result_z_ = -1.0;
    double max_result_z_ = 1.0;
    double local_map_radius_ = 12.0;
    double gicp_map_voxel_ = 0.25;
    double gicp_scan_voxel_ = 0.2;
    double gicp_max_correspondence_ = 1.0;
    int gicp_max_iterations_ = 60;
    double gicp_fitness_threshold_ = 0.3;
    int consistent_result_count_ = 2;
    double consistency_translation_ = 0.5;
    double consistency_rotation_ = 0.0;
    bool verify_map_fingerprint_ = true;

    ir::CloudPtr map_points_{new ir::Cloud};
    ir::NormalCloudPtr map_normals_{new ir::NormalCloud};
    ir::FeatureCloudPtr map_features_{new ir::FeatureCloud};
    ir::CloudPtr validation_map_{new ir::Cloud};
    pcl::KdTreeFLANN<ir::PointT> validation_tree_;
    ir::CloudPtr accumulated_{new ir::Cloud};
    int accumulated_frames_ = 0;
    int attempt_ = 0;
    bool processing_ = false;
    bool finished_ = false;
    int consistent_count_ = 0;
    Eigen::Matrix4f previous_pose_ = Eigen::Matrix4f::Identity();

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr result_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr teaser_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<TeaserFpfhLocalizer>());
    } catch (const std::exception &error) {
        std::cerr << "teaser_fpfh_localizer fatal error: " << error.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
