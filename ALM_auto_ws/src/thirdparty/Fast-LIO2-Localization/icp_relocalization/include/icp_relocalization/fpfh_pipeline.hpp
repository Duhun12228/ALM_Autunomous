#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

namespace icp_relocalization
{

using PointT = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointT>;
using CloudPtr = Cloud::Ptr;
using CloudConstPtr = Cloud::ConstPtr;
using NormalT = pcl::Normal;
using NormalCloud = pcl::PointCloud<NormalT>;
using NormalCloudPtr = NormalCloud::Ptr;
using FeatureT = pcl::FPFHSignature33;
using FeatureCloud = pcl::PointCloud<FeatureT>;
using FeatureCloudPtr = FeatureCloud::Ptr;

struct FeatureParams
{
    double voxel = 0.5;
    double normal_radius = 1.0;
    double feature_radius = 2.5;
    double min_range = 0.0;
    double max_range = 0.0;
    double z_min = -0.35;
    double z_max = 1.0;
    double min_curvature = 0.0;
    int max_features = 0;
    int num_threads = 4;
};

struct FeatureSet
{
    CloudPtr points{new Cloud};
    NormalCloudPtr normals{new NormalCloud};
    FeatureCloudPtr features{new FeatureCloud};
    std::size_t input_size = 0;
    std::size_t filtered_size = 0;
};

struct FeatureMatch
{
    int source = -1;
    int target = -1;
    float ratio = std::numeric_limits<float>::infinity();
};

inline bool finite_feature(const FeatureT &feature)
{
    for (float value : feature.histogram) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    return true;
}

inline CloudPtr filter_cloud(const CloudConstPtr &input, const FeatureParams &params)
{
    CloudPtr cropped(new Cloud);
    cropped->reserve(input->size());
    const double min_range_sq = params.min_range * params.min_range;
    const double max_range_sq = params.max_range * params.max_range;
    for (const auto &point : *input) {
        if (!pcl::isFinite(point) || point.z < params.z_min || point.z > params.z_max) {
            continue;
        }
        const double range_sq = static_cast<double>(point.x) * point.x +
                                static_cast<double>(point.y) * point.y;
        if (params.min_range > 0.0 && range_sq < min_range_sq) {
            continue;
        }
        if (params.max_range > 0.0 && range_sq > max_range_sq) {
            continue;
        }
        cropped->push_back(point);
    }
    cropped->width = static_cast<std::uint32_t>(cropped->size());
    cropped->height = 1;

    CloudPtr downsampled(new Cloud);
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(cropped);
    const float leaf = static_cast<float>(params.voxel);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.filter(*downsampled);
    return downsampled;
}

inline FeatureSet compute_features(const CloudConstPtr &input, const FeatureParams &params)
{
    if (params.voxel <= 0.0 || params.normal_radius <= params.voxel ||
        params.feature_radius <= params.normal_radius) {
        throw std::invalid_argument(
            "FPFH requires 0 < voxel < normal_radius < feature_radius");
    }

    FeatureSet result;
    result.input_size = input->size();
    const CloudPtr surface = filter_cloud(input, params);
    result.filtered_size = surface->size();
    if (surface->size() < 20) {
        return result;
    }

    NormalCloudPtr surface_normals(new NormalCloud);
    pcl::NormalEstimationOMP<PointT, NormalT> normal_estimator;
    normal_estimator.setNumberOfThreads(std::max(1, params.num_threads));
    normal_estimator.setInputCloud(surface);
    normal_estimator.setSearchMethod(typename pcl::search::KdTree<PointT>::Ptr(
        new pcl::search::KdTree<PointT>));
    normal_estimator.setRadiusSearch(params.normal_radius);
    normal_estimator.compute(*surface_normals);

    FeatureCloudPtr surface_features(new FeatureCloud);
    pcl::FPFHEstimationOMP<PointT, NormalT, FeatureT> feature_estimator;
    feature_estimator.setNumberOfThreads(std::max(1, params.num_threads));
    feature_estimator.setInputCloud(surface);
    feature_estimator.setInputNormals(surface_normals);
    feature_estimator.setSearchMethod(typename pcl::search::KdTree<PointT>::Ptr(
        new pcl::search::KdTree<PointT>));
    feature_estimator.setRadiusSearch(params.feature_radius);
    feature_estimator.compute(*surface_features);

    std::vector<int> valid;
    valid.reserve(surface->size());
    for (std::size_t i = 0; i < surface->size(); ++i) {
        const auto &normal = (*surface_normals)[i];
        if (!pcl::isFinite(normal) || !finite_feature((*surface_features)[i]) ||
            normal.curvature < params.min_curvature) {
            continue;
        }
        valid.push_back(static_cast<int>(i));
    }

    if (params.max_features > 0 &&
        valid.size() > static_cast<std::size_t>(params.max_features)) {
        std::partial_sort(
            valid.begin(), valid.begin() + params.max_features, valid.end(),
            [&surface_normals](int lhs, int rhs) {
                return (*surface_normals)[lhs].curvature >
                       (*surface_normals)[rhs].curvature;
            });
        valid.resize(static_cast<std::size_t>(params.max_features));
        std::sort(valid.begin(), valid.end());
    }

    result.points->reserve(valid.size());
    result.normals->reserve(valid.size());
    result.features->reserve(valid.size());
    for (int index : valid) {
        result.points->push_back((*surface)[index]);
        result.normals->push_back((*surface_normals)[index]);
        result.features->push_back((*surface_features)[index]);
    }
    result.points->width = static_cast<std::uint32_t>(result.points->size());
    result.points->height = 1;
    result.normals->width = static_cast<std::uint32_t>(result.normals->size());
    result.normals->height = 1;
    result.features->width = static_cast<std::uint32_t>(result.features->size());
    result.features->height = 1;
    return result;
}

inline std::vector<FeatureMatch> match_features(
    const FeatureCloudPtr &source, const FeatureCloudPtr &target,
    double ratio_threshold, int max_matches)
{
    pcl::KdTreeFLANN<FeatureT> target_tree;
    target_tree.setInputCloud(target);
    pcl::KdTreeFLANN<FeatureT> source_tree;
    source_tree.setInputCloud(source);

    std::vector<FeatureMatch> matches;
    matches.reserve(source->size());
    std::vector<int> indices(2);
    std::vector<float> distances(2);
    std::vector<int> reverse_index(1);
    std::vector<float> reverse_distance(1);
    const float ratio_squared = static_cast<float>(ratio_threshold * ratio_threshold);
    for (std::size_t i = 0; i < source->size(); ++i) {
        if (target_tree.nearestKSearch((*source)[i], 2, indices, distances) != 2 ||
            distances[1] <= 1e-12f || distances[0] > ratio_squared * distances[1]) {
            continue;
        }
        if (source_tree.nearestKSearch(
                (*target)[static_cast<std::size_t>(indices[0])], 1,
                reverse_index, reverse_distance) != 1 ||
            reverse_index[0] != static_cast<int>(i)) {
            continue;
        }
        matches.push_back(FeatureMatch{
            static_cast<int>(i), indices[0], distances[0] / distances[1]});
    }
    std::sort(matches.begin(), matches.end(),
        [](const FeatureMatch &lhs, const FeatureMatch &rhs) {
            return lhs.ratio < rhs.ratio;
        });
    if (max_matches > 0 && matches.size() > static_cast<std::size_t>(max_matches)) {
        matches.resize(static_cast<std::size_t>(max_matches));
    }
    return matches;
}

inline std::string file_fingerprint(const std::string &path)
{
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        return "unavailable";
    }
    std::uint64_t hash = 1469598103934665603ULL;
    std::array<char, 65536> buffer{};
    while (input) {
        input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        const std::streamsize count = input.gcount();
        for (std::streamsize i = 0; i < count; ++i) {
            hash ^= static_cast<unsigned char>(buffer[static_cast<std::size_t>(i)]);
            hash *= 1099511628211ULL;
        }
    }
    std::ostringstream output;
    output << std::hex << std::setfill('0') << std::setw(16) << hash;
    return output.str();
}

inline std::map<std::string, std::string> read_metadata(const std::string &path)
{
    std::map<std::string, std::string> values;
    std::ifstream input(path);
    std::string line;
    while (std::getline(input, line)) {
        const std::size_t delimiter = line.find('=');
        if (delimiter == std::string::npos) {
            continue;
        }
        values[line.substr(0, delimiter)] = line.substr(delimiter + 1);
    }
    return values;
}

inline bool close_parameter(double lhs, double rhs)
{
    return std::abs(lhs - rhs) <= 1e-6;
}

}  // namespace icp_relocalization
