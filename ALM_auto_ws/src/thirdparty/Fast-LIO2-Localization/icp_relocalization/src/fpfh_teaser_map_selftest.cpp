#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <teaser/registration.h>

#include "icp_relocalization/fpfh_pipeline.hpp"

namespace ir = icp_relocalization;

int main(int argc, char **argv)
{
    if (argc < 3 || argc > 6) {
        std::cerr << "Usage: " << argv[0]
                  << " MAP.pcd DB_PREFIX [x y yaw_deg]" << std::endl;
        return 2;
    }
    const std::string map_path(argv[1]);
    const std::string db_prefix(argv[2]);
    const double x = argc >= 4 ? std::stod(argv[3]) : -0.35;
    const double y = argc >= 5 ? std::stod(argv[4]) : 1.60;
    const double yaw = (argc >= 6 ? std::stod(argv[5]) : -30.0) * M_PI / 180.0;

    ir::CloudPtr map(new ir::Cloud);
    ir::CloudPtr map_feature_points(new ir::Cloud);
    ir::FeatureCloudPtr map_features(new ir::FeatureCloud);
    if (pcl::io::loadPCDFile(map_path, *map) < 0 ||
        pcl::io::loadPCDFile(db_prefix + "_points.pcd", *map_feature_points) < 0 ||
        pcl::io::loadPCDFile(db_prefix + "_fpfh.pcd", *map_features) < 0) {
        std::cerr << "Failed to load map or FPFH DB" << std::endl;
        return 1;
    }

    const Eigen::Matrix3d expected_rotation =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d expected_translation(x, y, 0.0);
    ir::CloudPtr synthetic_scan(new ir::Cloud);
    synthetic_scan->reserve(map->size() / 10);
    for (const auto &point : *map) {
        if (!pcl::isFinite(point) || point.z < -0.35 || point.z > 1.0) {
            continue;
        }
        const double dx = point.x - x;
        const double dy = point.y - y;
        const double radius = std::hypot(dx, dy);
        if (radius < 0.5 || radius > 10.0) {
            continue;
        }
        const Eigen::Vector3d map_point(point.x, point.y, point.z);
        const Eigen::Vector3d local =
            expected_rotation.transpose() * (map_point - expected_translation);
        synthetic_scan->push_back(ir::PointT(
            static_cast<float>(local.x()), static_cast<float>(local.y()),
            static_cast<float>(local.z())));
    }
    synthetic_scan->width = static_cast<std::uint32_t>(synthetic_scan->size());
    synthetic_scan->height = 1;

    ir::FeatureParams params;
    params.voxel = 0.5;
    params.normal_radius = 1.0;
    params.feature_radius = 2.5;
    params.min_range = 0.5;
    params.max_range = 10.0;
    params.z_min = -0.35;
    params.z_max = 1.0;
    params.max_features = 1500;
    params.num_threads = 4;
    const ir::FeatureSet source = ir::compute_features(synthetic_scan, params);
    const std::vector<ir::FeatureMatch> matches =
        ir::match_features(source.features, map_features, 0.95, 400);
    std::cout << "Map FPFH self-test: scan_raw=" << synthetic_scan->size()
              << " scan_features=" << source.features->size()
              << " mutual_matches=" << matches.size() << std::endl;
    if (matches.size() < 20) {
        std::cerr << "Map FPFH self-test FAILED: too few feature matches" << std::endl;
        return 1;
    }

    Eigen::Matrix<double, 3, Eigen::Dynamic> source_matrix(3, matches.size());
    Eigen::Matrix<double, 3, Eigen::Dynamic> target_matrix(3, matches.size());
    for (std::size_t i = 0; i < matches.size(); ++i) {
        const auto &src = (*source.points)[static_cast<std::size_t>(matches[i].source)];
        const auto &dst = (*map_feature_points)[static_cast<std::size_t>(matches[i].target)];
        source_matrix.col(static_cast<Eigen::Index>(i)) = Eigen::Vector3d(src.x, src.y, src.z);
        target_matrix.col(static_cast<Eigen::Index>(i)) = Eigen::Vector3d(dst.x, dst.y, dst.z);
    }

    teaser::RobustRegistrationSolver::Params solver_params;
    solver_params.noise_bound = 0.5;
    solver_params.cbar2 = 1.0;
    solver_params.estimate_scaling = false;
    solver_params.rotation_estimation_algorithm =
        teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
    solver_params.rotation_max_iterations = 100;
    solver_params.rotation_gnc_factor = 1.4;
    solver_params.inlier_selection_mode =
        teaser::RobustRegistrationSolver::INLIER_SELECTION_MODE::PMC_HEU;
    teaser::RobustRegistrationSolver solver(solver_params);
    solver.solve(source_matrix, target_matrix);
    const teaser::RegistrationSolution solution = solver.getSolution();
    const double translation_error =
        (solution.translation - expected_translation).norm();
    const double cosine = std::max(
        -1.0, std::min(1.0,
            ((expected_rotation.transpose() * solution.rotation).trace() - 1.0) / 2.0));
    const double rotation_error = std::acos(cosine) * 180.0 / M_PI;
    std::cout << "valid=" << solution.valid
              << " clique=" << solver.getInlierMaxClique().size()
              << " translation_error=" << translation_error << "m"
              << " rotation_error=" << rotation_error << "deg" << std::endl;
    if (!solution.valid || translation_error > 1.0 || rotation_error > 5.0) {
        std::cerr << "Map FPFH self-test FAILED" << std::endl;
        return 1;
    }

    Eigen::Matrix4f initial = Eigen::Matrix4f::Identity();
    initial.block<3, 3>(0, 0) = solution.rotation.cast<float>();
    initial.block<3, 1>(0, 3) = solution.translation.cast<float>();
    ir::FeatureParams target_filter = params;
    target_filter.voxel = 0.25;
    target_filter.min_range = 0.0;
    target_filter.max_range = 0.0;
    const ir::CloudPtr map_downsampled = ir::filter_cloud(map, target_filter);
    pcl::CropBox<ir::PointT> crop;
    crop.setInputCloud(map_downsampled);
    crop.setMin(Eigen::Vector4f(
        initial(0, 3) - 12.0f, initial(1, 3) - 12.0f,
        initial(2, 3) - 3.0f, 1.0f));
    crop.setMax(Eigen::Vector4f(
        initial(0, 3) + 12.0f, initial(1, 3) + 12.0f,
        initial(2, 3) + 3.0f, 1.0f));
    ir::CloudPtr local_map(new ir::Cloud);
    crop.filter(*local_map);
    ir::FeatureParams source_filter = params;
    source_filter.voxel = 0.20;
    const ir::CloudPtr source_gicp = ir::filter_cloud(synthetic_scan, source_filter);

    pcl::GeneralizedIterativeClosestPoint<ir::PointT, ir::PointT> gicp;
    gicp.setInputSource(source_gicp);
    gicp.setInputTarget(local_map);
    gicp.setMaxCorrespondenceDistance(1.0);
    gicp.setMaximumIterations(60);
    gicp.setTransformationEpsilon(1e-4);
    gicp.setEuclideanFitnessEpsilon(1e-5);
    ir::Cloud aligned;
    gicp.align(aligned, initial);
    const Eigen::Matrix4f refined = gicp.getFinalTransformation();
    const double refined_translation_error =
        (refined.block<3, 1>(0, 3).cast<double>() - expected_translation).norm();
    const Eigen::Matrix3d refined_rotation = refined.block<3, 3>(0, 0).cast<double>();
    const double refined_cosine = std::max(
        -1.0, std::min(1.0,
            ((expected_rotation.transpose() * refined_rotation).trace() - 1.0) / 2.0));
    const double refined_rotation_error = std::acos(refined_cosine) * 180.0 / M_PI;
    const double gicp_fitness = gicp.getFitnessScore(1.0);
    std::cout << "GICP converged=" << gicp.hasConverged()
              << " fitness=" << gicp_fitness
              << " translation_error=" << refined_translation_error << "m"
              << " rotation_error=" << refined_rotation_error << "deg" << std::endl;
    if (!gicp.hasConverged() || gicp_fitness > 0.30 ||
        refined_translation_error > 0.20 || refined_rotation_error > 2.0) {
        std::cerr << "Map FPFH+TEASER+GICP self-test FAILED" << std::endl;
        return 1;
    }
    std::cout << "Map FPFH+TEASER+GICP self-test PASSED" << std::endl;
    return 0;
}
