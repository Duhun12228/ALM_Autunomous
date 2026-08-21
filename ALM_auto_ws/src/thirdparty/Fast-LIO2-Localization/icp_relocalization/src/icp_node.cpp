#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#ifdef USE_LIVOX
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

class ICPNode : public rclcpp::Node
{
public:
    using PointT = pcl::PointXYZ;
    using Cloud = pcl::PointCloud<PointT>;
    using CloudPtr = Cloud::Ptr;
    using CloudConstPtr = Cloud::ConstPtr;
    static constexpr double RAD_TO_DEG = 57.29577951308232;

    ICPNode()
        : Node("icp_node")
    {
        declare_parameters();
        read_parameters();

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "icp_result", 10);
#ifdef USE_LIVOX
        if (pcl_type_ == "livox") {
            lvx_cloud_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                "/livox/lidar", rclcpp::SensorDataQoS().keep_last(1),
                std::bind(&ICPNode::lvx_cloud_callback, this, std::placeholders::_1));
        } else {
            cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/pointcloud2", rclcpp::SensorDataQoS().keep_last(1),
                std::bind(&ICPNode::cloud_callback, this, std::placeholders::_1));
        }
#else
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud2", rclcpp::SensorDataQoS().keep_last(1),
            std::bind(&ICPNode::cloud_callback, this, std::placeholders::_1));
#endif
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10,
            std::bind(&ICPNode::pose_callback, this, std::placeholders::_1));
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("prior_map", 10);
        transformed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "transformed_cloud", 10);

        init_guess_ = make_pose(initial_x_, initial_y_, initial_z_, initial_a_);
        pose_received_ = !wait_for_initial_pose_;
        log_pose("초기 파라미터", init_guess_);

        CloudPtr full_map(new Cloud);
        if (pcl::io::loadPCDFile<PointT>(map_path_, *full_map) == -1) {
            RCLCPP_FATAL(this->get_logger(), "PCD 맵을 읽을 수 없음: %s", map_path_.c_str());
            throw std::runtime_error("failed to load prior map");
        }
        RCLCPP_INFO(this->get_logger(), "Prior map 로드: %zu점", full_map->size());

        build_stages(full_map);
        publish_map_cloud_ = stages_.back().target;
        pcl::toROSMsg(*publish_map_cloud_, target_cloud_msg_);
        target_cloud_msg_.header.stamp = this->now();
        target_cloud_msg_.header.frame_id = map_frame_;
        map_pub_->publish(target_cloud_msg_);

        if (wait_for_initial_pose_) {
            RCLCPP_INFO(this->get_logger(),
                        "SC /initialpose 대기 중 — 후보 수신 전에는 ICP를 실행하지 않음");
        }
    }

private:
    struct Stage
    {
        std::string name;
        double map_voxel;
        double cloud_voxel;
        double max_correspondence;
        double fitness_threshold;
        int max_iterations;
        CloudPtr target;
    };

    struct AlignmentResult
    {
        bool converged = false;
        double fitness = std::numeric_limits<double>::infinity();
        std::size_t source_size = 0;
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    };

    struct PoseScore
    {
        double objective = std::numeric_limits<double>::infinity();
        double capped_mean = std::numeric_limits<double>::infinity();
        double inlier_mean = std::numeric_limits<double>::infinity();
        double inlier_ratio = 0.0;
    };

    void declare_parameters()
    {
        this->declare_parameter("initial_x", 0.0);
        this->declare_parameter("initial_y", 0.0);
        this->declare_parameter("initial_z", 0.0);
        this->declare_parameter("initial_a", 0.0);
        this->declare_parameter("map_path", "");
        this->declare_parameter("map_frame_id", "map");
        this->declare_parameter("pcl_type", "livox");
        this->declare_parameter("wait_for_initial_pose", false);
        this->declare_parameter("enable_multistage", false);
        this->declare_parameter("RANSAC_outlier_rejection_threshold", 1.0);
        this->declare_parameter("converged_count_thre", 20);

        // 단일 단계 호환 파라미터. enable_multistage=false 일 때 사용한다.
        this->declare_parameter("solver_max_iter", 75);
        this->declare_parameter("max_correspondence_distance", 0.1);
        this->declare_parameter("fitness_score_thre", 0.2);
        this->declare_parameter("map_voxel_leaf_size", 0.5);
        this->declare_parameter("cloud_voxel_leaf_size", 0.3);

        this->declare_parameter("coarse_map_voxel", 0.8);
        this->declare_parameter("coarse_cloud_voxel", 0.5);
        this->declare_parameter("coarse_max_correspondence", 2.5);
        this->declare_parameter("coarse_fitness_threshold", 2.5);
        this->declare_parameter("coarse_max_iterations", 40);

        this->declare_parameter("medium_map_voxel", 0.4);
        this->declare_parameter("medium_cloud_voxel", 0.25);
        this->declare_parameter("medium_max_correspondence", 1.0);
        this->declare_parameter("medium_fitness_threshold", 1.0);
        this->declare_parameter("medium_max_iterations", 50);

        this->declare_parameter("fine_map_voxel", 0.2);
        this->declare_parameter("fine_cloud_voxel", 0.15);
        this->declare_parameter("fine_max_correspondence", 0.35);
        this->declare_parameter("fine_fitness_threshold", 0.5);
        this->declare_parameter("fine_max_iterations", 50);

        // Medium 결과 주변을 반복 탐색하고 탐색 간격을 줄인 뒤 Fine ICP를 수행한다.
        this->declare_parameter("enable_fine_search", false);
        this->declare_parameter("fine_search_xy_steps",
                                std::vector<double>{0.5, 0.2, 0.05});
        this->declare_parameter("fine_search_yaw_steps_deg",
                                std::vector<double>{5.0, 2.0, 0.5});
        this->declare_parameter("fine_search_rounds_per_level", 2);
        this->declare_parameter("fine_search_max_attempts", 3);
        this->declare_parameter("fine_search_score_voxel", 0.5);
        this->declare_parameter("fine_search_score_max_distance", 1.5);
        this->declare_parameter("fine_search_inlier_distance", 0.35);
        this->declare_parameter("fine_search_min_inlier_ratio", 0.15);
        this->declare_parameter("fine_search_inlier_weight", 0.5);
    }

    void read_parameters()
    {
        initial_x_ = this->get_parameter("initial_x").as_double();
        initial_y_ = this->get_parameter("initial_y").as_double();
        initial_z_ = this->get_parameter("initial_z").as_double();
        initial_a_ = this->get_parameter("initial_a").as_double();
        map_path_ = this->get_parameter("map_path").as_string();
        map_frame_ = this->get_parameter("map_frame_id").as_string();
        pcl_type_ = this->get_parameter("pcl_type").as_string();
        wait_for_initial_pose_ = this->get_parameter("wait_for_initial_pose").as_bool();
        enable_multistage_ = this->get_parameter("enable_multistage").as_bool();
        ransac_threshold_ =
            this->get_parameter("RANSAC_outlier_rejection_threshold").as_double();
        converged_count_threshold_ = this->get_parameter("converged_count_thre").as_int();

        solver_max_iterations_ = this->get_parameter("solver_max_iter").as_int();
        max_correspondence_distance_ =
            this->get_parameter("max_correspondence_distance").as_double();
        fitness_threshold_ = this->get_parameter("fitness_score_thre").as_double();
        map_voxel_ = this->get_parameter("map_voxel_leaf_size").as_double();
        cloud_voxel_ = this->get_parameter("cloud_voxel_leaf_size").as_double();

        coarse_ = read_stage("Coarse", "coarse");
        medium_ = read_stage("Medium", "medium");
        fine_ = read_stage("Fine", "fine");

        enable_fine_search_ = this->get_parameter("enable_fine_search").as_bool();
        fine_search_xy_steps_ =
            this->get_parameter("fine_search_xy_steps").as_double_array();
        fine_search_yaw_steps_deg_ =
            this->get_parameter("fine_search_yaw_steps_deg").as_double_array();
        fine_search_rounds_per_level_ =
            this->get_parameter("fine_search_rounds_per_level").as_int();
        fine_search_max_attempts_ =
            this->get_parameter("fine_search_max_attempts").as_int();
        fine_search_score_voxel_ =
            this->get_parameter("fine_search_score_voxel").as_double();
        fine_search_score_max_distance_ =
            this->get_parameter("fine_search_score_max_distance").as_double();
        fine_search_inlier_distance_ =
            this->get_parameter("fine_search_inlier_distance").as_double();
        fine_search_min_inlier_ratio_ =
            this->get_parameter("fine_search_min_inlier_ratio").as_double();
        fine_search_inlier_weight_ =
            this->get_parameter("fine_search_inlier_weight").as_double();
    }

    Stage read_stage(const std::string &name, const std::string &prefix)
    {
        Stage stage;
        stage.name = name;
        stage.map_voxel = this->get_parameter(prefix + "_map_voxel").as_double();
        stage.cloud_voxel = this->get_parameter(prefix + "_cloud_voxel").as_double();
        stage.max_correspondence =
            this->get_parameter(prefix + "_max_correspondence").as_double();
        stage.fitness_threshold =
            this->get_parameter(prefix + "_fitness_threshold").as_double();
        stage.max_iterations = this->get_parameter(prefix + "_max_iterations").as_int();
        return stage;
    }

    static Eigen::Matrix4f make_pose(double x, double y, double z, double yaw)
    {
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose(0, 3) = static_cast<float>(x);
        pose(1, 3) = static_cast<float>(y);
        pose(2, 3) = static_cast<float>(z);
        pose.block<3, 3>(0, 0) =
            Eigen::AngleAxisf(static_cast<float>(yaw), Eigen::Vector3f::UnitZ())
                .toRotationMatrix();
        return pose;
    }

    CloudPtr downsample(const CloudConstPtr &cloud, double leaf_size) const
    {
        CloudPtr filtered(new Cloud);
        pcl::VoxelGrid<PointT> voxel;
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(
            static_cast<float>(leaf_size),
            static_cast<float>(leaf_size),
            static_cast<float>(leaf_size));
        voxel.filter(*filtered);
        return filtered;
    }

    void validate_stage(const Stage &stage) const
    {
        if (stage.map_voxel <= 0.0 || stage.cloud_voxel <= 0.0 ||
            stage.max_correspondence <= 0.0 || stage.fitness_threshold <= 0.0 ||
            stage.max_iterations <= 0) {
            throw std::invalid_argument(stage.name + " ICP 파라미터는 모두 양수여야 함");
        }
    }

    void build_stages(const CloudConstPtr &full_map)
    {
        if (enable_multistage_) {
            stages_ = {coarse_, medium_, fine_};
        } else {
            Stage legacy;
            legacy.name = "Single";
            legacy.map_voxel = map_voxel_;
            legacy.cloud_voxel = cloud_voxel_;
            legacy.max_correspondence = max_correspondence_distance_;
            legacy.fitness_threshold = fitness_threshold_;
            legacy.max_iterations = solver_max_iterations_;
            stages_ = {legacy};
        }

        for (auto &stage : stages_) {
            validate_stage(stage);
            stage.target = downsample(full_map, stage.map_voxel);
            RCLCPP_INFO(
                this->get_logger(),
                "[%s] map=%zu점 voxel=%.2fm, scan voxel=%.2fm, corr=%.2fm, "
                "fitness<%.3f, iter=%d",
                stage.name.c_str(), stage.target->size(), stage.map_voxel,
                stage.cloud_voxel, stage.max_correspondence,
                stage.fitness_threshold, stage.max_iterations);
        }

        if (enable_fine_search_) {
            if (!enable_multistage_ || stages_.size() != 3) {
                throw std::invalid_argument(
                    "enable_fine_search는 3단계 ICP와 함께 사용해야 함");
            }
            if (fine_search_xy_steps_.empty() ||
                fine_search_xy_steps_.size() != fine_search_yaw_steps_deg_.size() ||
                fine_search_rounds_per_level_ <= 0 || fine_search_max_attempts_ <= 0 ||
                fine_search_score_voxel_ <= 0.0 ||
                fine_search_score_max_distance_ <= 0.0 ||
                fine_search_inlier_distance_ <= 0.0 ||
                fine_search_min_inlier_ratio_ < 0.0 ||
                fine_search_min_inlier_ratio_ > 1.0 || fine_search_inlier_weight_ < 0.0) {
                throw std::invalid_argument("Fine 탐색 파라미터가 올바르지 않음");
            }
            for (std::size_t i = 0; i < fine_search_xy_steps_.size(); ++i) {
                if (fine_search_xy_steps_[i] <= 0.0 ||
                    fine_search_yaw_steps_deg_[i] <= 0.0) {
                    throw std::invalid_argument("Fine 탐색 간격은 모두 양수여야 함");
                }
            }

            fine_score_tree_.reset(new pcl::KdTreeFLANN<PointT>);
            fine_score_tree_->setInputCloud(stages_.back().target);
            RCLCPP_INFO(
                this->get_logger(),
                "[FineSearch] %zu단계 x/y/yaw 3x3x3 탐색, 단계당 최대 %d회, "
                "스캔 최대 %d회 재시도 | "
                "score voxel=%.2fm, cap=%.2fm, inlier<=%.2fm, min ratio=%.2f",
                fine_search_xy_steps_.size(), fine_search_rounds_per_level_,
                fine_search_max_attempts_,
                fine_search_score_voxel_, fine_search_score_max_distance_,
                fine_search_inlier_distance_, fine_search_min_inlier_ratio_);
        }
    }

    AlignmentResult align_stage(
        const Stage &stage,
        const CloudConstPtr &raw_cloud,
        const Eigen::Matrix4f &initial_guess)
    {
        AlignmentResult alignment;
        const CloudPtr source = downsample(raw_cloud, stage.cloud_voxel);
        alignment.source_size = source->size();
        if (source->size() < 20 || stage.target->size() < 20) {
            RCLCPP_WARN(this->get_logger(),
                        "[%s] 점 부족: source=%zu target=%zu — 후보 거절",
                        stage.name.c_str(), source->size(), stage.target->size());
            return alignment;
        }

        pcl::IterativeClosestPoint<PointT, PointT> icp;
        icp.setInputSource(source);
        icp.setInputTarget(stage.target);
        icp.setMaximumIterations(stage.max_iterations);
        icp.setMaxCorrespondenceDistance(stage.max_correspondence);
        icp.setRANSACOutlierRejectionThreshold(ransac_threshold_);

        Cloud aligned;
        icp.align(aligned, initial_guess);
        alignment.converged = icp.hasConverged();
        alignment.fitness = icp.getFitnessScore();
        alignment.transform = icp.getFinalTransformation();
        return alignment;
    }

    bool run_stage(
        const Stage &stage,
        const CloudConstPtr &raw_cloud,
        const Eigen::Matrix4f &initial_guess,
        Eigen::Matrix4f &result)
    {
        const AlignmentResult alignment = align_stage(stage, raw_cloud, initial_guess);
        result = alignment.transform;
        const bool accepted = alignment.converged && std::isfinite(alignment.fitness) &&
                              alignment.fitness < stage.fitness_threshold;

        RCLCPP_INFO(
            this->get_logger(),
            "[%s] %s | source=%zu fitness=%.6f (기준 < %.3f) | "
            "x=%.3f y=%.3f z=%.3f yaw=%.1fdeg",
            stage.name.c_str(), accepted ? "PASS" : "FAIL", alignment.source_size,
            alignment.fitness, stage.fitness_threshold,
            result(0, 3), result(1, 3), result(2, 3),
            std::atan2(result(1, 0), result(0, 0)) * RAD_TO_DEG);
        return accepted;
    }

    Eigen::Matrix4f offset_pose(
        const Eigen::Matrix4f &pose, double dx, double dy, double dyaw) const
    {
        Eigen::Matrix4f candidate = pose;
        candidate(0, 3) += static_cast<float>(dx);
        candidate(1, 3) += static_cast<float>(dy);
        candidate.block<3, 3>(0, 0) =
            Eigen::AngleAxisf(static_cast<float>(dyaw), Eigen::Vector3f::UnitZ()) *
            pose.block<3, 3>(0, 0);
        return candidate;
    }

    PoseScore score_pose(
        const CloudConstPtr &source, const Eigen::Matrix4f &pose) const
    {
        PoseScore score;
        if (!fine_score_tree_ || source->empty()) {
            return score;
        }

        const double cap_squared = fine_search_score_max_distance_ *
                                   fine_search_score_max_distance_;
        const double inlier_squared = fine_search_inlier_distance_ *
                                      fine_search_inlier_distance_;
        double capped_sum = 0.0;
        double inlier_sum = 0.0;
        std::size_t inlier_count = 0;
        std::vector<int> indices(1);
        std::vector<float> distances(1);

        for (const auto &point : *source) {
            const Eigen::Vector4f local(point.x, point.y, point.z, 1.0f);
            const Eigen::Vector4f map_point = pose * local;
            PointT query;
            query.x = map_point.x();
            query.y = map_point.y();
            query.z = map_point.z();

            double distance_squared = cap_squared;
            if (fine_score_tree_->nearestKSearch(query, 1, indices, distances) > 0) {
                distance_squared = static_cast<double>(distances[0]);
            }
            capped_sum += std::min(distance_squared, cap_squared);
            if (distance_squared <= inlier_squared) {
                inlier_sum += distance_squared;
                ++inlier_count;
            }
        }

        const double count = static_cast<double>(source->size());
        score.capped_mean = capped_sum / count;
        score.inlier_ratio = static_cast<double>(inlier_count) / count;
        if (inlier_count > 0) {
            score.inlier_mean = inlier_sum / static_cast<double>(inlier_count);
        }
        score.objective = score.capped_mean +
                          fine_search_inlier_weight_ * (1.0 - score.inlier_ratio);
        return score;
    }

    bool run_fine_search(
        const Stage &stage,
        const CloudConstPtr &raw_cloud,
        const Eigen::Matrix4f &medium_result,
        Eigen::Matrix4f &result)
    {
        const CloudPtr score_source = downsample(raw_cloud, fine_search_score_voxel_);
        if (score_source->size() < 20) {
            RCLCPP_WARN(this->get_logger(),
                        "[FineSearch] 점 부족: source=%zu — 후보 거절",
                        score_source->size());
            return false;
        }

        Eigen::Matrix4f center = medium_result;
        PoseScore center_score = score_pose(score_source, center);
        int evaluated = 1;
        RCLCPP_INFO(
            this->get_logger(),
            "[FineSearch] 시작 | source=%zu score=%.6f inlier=%.1f%% | "
            "x=%.3f y=%.3f yaw=%.1fdeg",
            score_source->size(), center_score.objective,
            100.0 * center_score.inlier_ratio, center(0, 3), center(1, 3),
            std::atan2(center(1, 0), center(0, 0)) * RAD_TO_DEG);

        for (std::size_t level = 0; level < fine_search_xy_steps_.size(); ++level) {
            const double xy_step = fine_search_xy_steps_[level];
            const double yaw_step = fine_search_yaw_steps_deg_[level] / RAD_TO_DEG;
            for (int round = 0; round < fine_search_rounds_per_level_; ++round) {
                Eigen::Matrix4f round_best_pose = center;
                PoseScore round_best_score = center_score;

                for (int ix = -1; ix <= 1; ++ix) {
                    for (int iy = -1; iy <= 1; ++iy) {
                        for (int ia = -1; ia <= 1; ++ia) {
                            if (ix == 0 && iy == 0 && ia == 0) {
                                continue;
                            }
                            const Eigen::Matrix4f candidate = offset_pose(
                                center, ix * xy_step, iy * xy_step, ia * yaw_step);
                            const PoseScore candidate_score =
                                score_pose(score_source, candidate);
                            ++evaluated;
                            if (candidate_score.objective < round_best_score.objective) {
                                round_best_pose = candidate;
                                round_best_score = candidate_score;
                            }
                        }
                    }
                }

                const bool improved =
                    round_best_score.objective + 1e-9 < center_score.objective;
                center = round_best_pose;
                center_score = round_best_score;
                if (!improved) {
                    break;
                }
            }

            RCLCPP_INFO(
                this->get_logger(),
                "[FineSearch L%zu] step=%.2fm/%.1fdeg | score=%.6f "
                "inlier=%.1f%% | x=%.3f y=%.3f yaw=%.1fdeg",
                level + 1, xy_step, fine_search_yaw_steps_deg_[level],
                center_score.objective, 100.0 * center_score.inlier_ratio,
                center(0, 3), center(1, 3),
                std::atan2(center(1, 0), center(0, 0)) * RAD_TO_DEG);
        }

        const AlignmentResult alignment = align_stage(stage, raw_cloud, center);
        result = alignment.transform;
        const PoseScore final_score = score_pose(score_source, result);
        const bool accepted = alignment.converged && std::isfinite(alignment.fitness) &&
                              alignment.fitness < stage.fitness_threshold &&
                              final_score.inlier_ratio >= fine_search_min_inlier_ratio_;

        RCLCPP_INFO(
            this->get_logger(),
            "[Fine] %s | hypotheses=%d source=%zu fitness=%.6f (기준 < %.3f) "
            "search_score=%.6f inlier=%.1f%% (기준 >= %.1f%%) | "
            "x=%.3f y=%.3f z=%.3f yaw=%.1fdeg",
            accepted ? "PASS" : "FAIL", evaluated, alignment.source_size,
            alignment.fitness, stage.fitness_threshold, final_score.objective,
            100.0 * final_score.inlier_ratio,
            100.0 * fine_search_min_inlier_ratio_, result(0, 3), result(1, 3),
            result(2, 3), std::atan2(result(1, 0), result(0, 0)) * RAD_TO_DEG);
        return accepted;
    }

    bool continue_fine_search(const CloudConstPtr &cloud, Eigen::Matrix4f &result)
    {
        ++fine_search_attempt_;
        RCLCPP_INFO(this->get_logger(),
                    "[FineSearch] 최신 스캔 재탐색 %d/%d",
                    fine_search_attempt_, fine_search_max_attempts_);

        const bool accepted =
            run_fine_search(stages_.back(), cloud, fine_search_guess_, result);
        if (accepted) {
            fine_search_active_ = false;
            fine_search_attempt_ = 0;
            return true;
        }

        // 실패해도 이번 탐색의 최적 결과를 버리지 않고 다음 최신 스캔의 중심으로 쓴다.
        fine_search_guess_ = result;
        if (fine_search_attempt_ >= fine_search_max_attempts_) {
            fine_search_active_ = false;
            candidate_exhausted_ = true;
            RCLCPP_WARN(
                this->get_logger(),
                "[FineSearch] %d회 재탐색에도 최종 판정 실패 — 다음 SC 후보 대기",
                fine_search_max_attempts_);
        } else {
            RCLCPP_WARN(
                this->get_logger(),
                "[FineSearch] 최적 자세 저장 — 다음 최신 스캔에서 Fine 재탐색 %d/%d 예정",
                fine_search_attempt_ + 1, fine_search_max_attempts_);
        }
        return false;
    }

    bool align_multistage(const CloudConstPtr &cloud, Eigen::Matrix4f &result)
    {
        if (enable_fine_search_ && fine_search_active_) {
            return continue_fine_search(cloud, result);
        }

        Eigen::Matrix4f guess = init_guess_;
        for (std::size_t i = 0; i < stages_.size(); ++i) {
            const Stage &stage = stages_[i];
            Eigen::Matrix4f stage_result = guess;
            const bool is_fine_search = enable_fine_search_ && i + 1 == stages_.size();
            if (is_fine_search) {
                fine_search_active_ = true;
                fine_search_guess_ = guess;
                fine_search_attempt_ = 0;
                return continue_fine_search(cloud, result);
            }

            const bool accepted = run_stage(stage, cloud, guess, stage_result);
            if (!accepted) {
                RCLCPP_WARN(this->get_logger(), "[%s] 단계에서 후보 거절", stage.name.c_str());
                return false;
            }
            guess = stage_result;
        }
        result = guess;
        return true;
    }

    void process_cloud(const CloudPtr &input_cloud)
    {
        if (!pose_received_) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 3000,
                "SC /initialpose 대기 중...");
            return;
        }
        if (input_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "빈 입력 점군 — ICP 건너뜀");
            return;
        }
        if (candidate_exhausted_) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 3000,
                "현재 SC 후보의 Fine 탐색 소진 — 다음 /initialpose 대기 중...");
            return;
        }

        Eigen::Matrix4f refined = init_guess_;
        const bool accepted = align_multistage(input_cloud, refined);
        if (!accepted) {
            converged_count_ = 0;
            publish_transformed_cloud(input_cloud, init_guess_);
            publish_prior_map();
            return;
        }

        // 완전한 Coarse→Medium→Fine 통과 결과만 다음 프레임의 초기값으로 사용한다.
        init_guess_ = refined;
        ++converged_count_;
        RCLCPP_INFO(this->get_logger(),
                    "다단계 ICP 연속 통과 %d/%d",
                    converged_count_, converged_count_threshold_);
        publish_transformed_cloud(input_cloud, refined);
        publish_prior_map();

        if (converged_count_ < converged_count_threshold_) {
            return;
        }

        publish_result(refined);
        RCLCPP_INFO(this->get_logger(), "다단계 ICP 수렴 — /icp_result 발행 후 종료");
        rclcpp::shutdown();
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        CloudPtr input_cloud(new Cloud);
        pcl::fromROSMsg(*msg, *input_cloud);
        process_cloud(input_cloud);
    }

#ifdef USE_LIVOX
    void lvx_cloud_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        CloudPtr input_cloud(new Cloud);
        input_cloud->reserve(msg->point_num);
        for (std::size_t i = 0; i < msg->point_num; ++i) {
            PointT point;
            point.x = msg->points[i].x;
            point.y = msg->points[i].y;
            point.z = msg->points[i].z;
            input_cloud->push_back(point);
        }
        input_cloud->width = input_cloud->size();
        input_cloud->height = 1;
        process_cloud(input_cloud);
    }
#endif

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        init_guess_ = Eigen::Matrix4f::Identity();
        init_guess_(0, 3) = static_cast<float>(msg->pose.pose.position.x);
        init_guess_(1, 3) = static_cast<float>(msg->pose.pose.position.y);
        init_guess_(2, 3) = static_cast<float>(msg->pose.pose.position.z);

        tf2::Quaternion quaternion;
        tf2::fromMsg(msg->pose.pose.orientation, quaternion);
        tf2::Matrix3x3 rotation(quaternion);
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                init_guess_(row, col) = static_cast<float>(rotation[row][col]);
            }
        }

        pose_received_ = true;
        converged_count_ = 0;
        fine_search_active_ = false;
        fine_search_attempt_ = 0;
        candidate_exhausted_ = false;
        log_pose("새 SC 후보 수신", init_guess_);
    }

    void log_pose(const char *label, const Eigen::Matrix4f &pose) const
    {
        const double yaw = std::atan2(pose(1, 0), pose(0, 0));
        RCLCPP_INFO(this->get_logger(),
                    "%s: x=%.3f y=%.3f z=%.3f yaw=%.1fdeg",
                    label, pose(0, 3), pose(1, 3), pose(2, 3), yaw * RAD_TO_DEG);
    }

    void publish_result(const Eigen::Matrix4f &transform)
    {
        geometry_msgs::msg::PoseWithCovarianceStamped message;
        message.header.stamp = this->now();
        message.header.frame_id = map_frame_;
        message.pose.pose.position.x = transform(0, 3);
        message.pose.pose.position.y = transform(1, 3);
        message.pose.pose.position.z = transform(2, 3);

        const Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
        const Eigen::Quaternionf quaternion(rotation);
        message.pose.pose.orientation.x = quaternion.x();
        message.pose.pose.orientation.y = quaternion.y();
        message.pose.pose.orientation.z = quaternion.z();
        message.pose.pose.orientation.w = quaternion.w();
        publisher_->publish(message);
    }

    void publish_transformed_cloud(
        const CloudConstPtr &cloud, const Eigen::Matrix4f &transform)
    {
        Cloud transformed;
        pcl::transformPointCloud(*cloud, transformed, transform);
        sensor_msgs::msg::PointCloud2 message;
        pcl::toROSMsg(transformed, message);
        message.header.stamp = this->now();
        message.header.frame_id = map_frame_;
        transformed_cloud_pub_->publish(message);
    }

    void publish_prior_map()
    {
        target_cloud_msg_.header.stamp = this->now();
        map_pub_->publish(target_cloud_msg_);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
#ifdef USE_LIVOX
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lvx_cloud_sub_;
#endif
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_cloud_pub_;

    std::vector<Stage> stages_;
    Stage coarse_;
    Stage medium_;
    Stage fine_;
    pcl::KdTreeFLANN<PointT>::Ptr fine_score_tree_;
    CloudPtr publish_map_cloud_;
    sensor_msgs::msg::PointCloud2 target_cloud_msg_;
    Eigen::Matrix4f init_guess_ = Eigen::Matrix4f::Identity();

    double initial_x_ = 0.0;
    double initial_y_ = 0.0;
    double initial_z_ = 0.0;
    double initial_a_ = 0.0;
    std::string map_path_;
    std::string map_frame_;
    std::string pcl_type_;
    bool wait_for_initial_pose_ = false;
    bool enable_multistage_ = false;
    bool pose_received_ = false;
    double ransac_threshold_ = 1.0;
    int converged_count_ = 0;
    int converged_count_threshold_ = 20;

    int solver_max_iterations_ = 75;
    double max_correspondence_distance_ = 0.1;
    double fitness_threshold_ = 0.2;
    double map_voxel_ = 0.5;
    double cloud_voxel_ = 0.3;

    bool enable_fine_search_ = false;
    std::vector<double> fine_search_xy_steps_;
    std::vector<double> fine_search_yaw_steps_deg_;
    int fine_search_rounds_per_level_ = 2;
    int fine_search_max_attempts_ = 3;
    double fine_search_score_voxel_ = 0.5;
    double fine_search_score_max_distance_ = 1.5;
    double fine_search_inlier_distance_ = 0.35;
    double fine_search_min_inlier_ratio_ = 0.15;
    double fine_search_inlier_weight_ = 0.5;
    bool fine_search_active_ = false;
    bool candidate_exhausted_ = false;
    int fine_search_attempt_ = 0;
    Eigen::Matrix4f fine_search_guess_ = Eigen::Matrix4f::Identity();
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICPNode>());
    rclcpp::shutdown();
    return 0;
}
