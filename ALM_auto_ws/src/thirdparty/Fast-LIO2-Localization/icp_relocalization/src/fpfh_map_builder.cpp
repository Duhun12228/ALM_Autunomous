#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include <pcl/io/pcd_io.h>

#include "icp_relocalization/fpfh_pipeline.hpp"

using icp_relocalization::Cloud;
using icp_relocalization::FeatureParams;
using icp_relocalization::FeatureSet;

namespace
{

struct Arguments
{
    std::string map_path;
    std::string output_prefix;
    FeatureParams params;
};

void usage(const char *program)
{
    std::cout
        << "Usage: " << program << " --map MAP.pcd --output-prefix PREFIX [options]\n"
        << "Options:\n"
        << "  --voxel 0.5 --normal-radius 1.0 --feature-radius 2.5\n"
        << "  --z-min -0.35 --z-max 1.0 --min-curvature 0.0\n"
        << "  --max-features 20000 --threads 4\n";
}

Arguments parse_arguments(int argc, char **argv)
{
    Arguments args;
    args.params.max_features = 20000;
    for (int i = 1; i < argc; ++i) {
        const std::string key(argv[i]);
        if (key == "--help" || key == "-h") {
            usage(argv[0]);
            std::exit(0);
        }
        if (i + 1 >= argc) {
            throw std::invalid_argument("missing value after " + key);
        }
        const std::string value(argv[++i]);
        if (key == "--map") args.map_path = value;
        else if (key == "--output-prefix") args.output_prefix = value;
        else if (key == "--voxel") args.params.voxel = std::stod(value);
        else if (key == "--normal-radius") args.params.normal_radius = std::stod(value);
        else if (key == "--feature-radius") args.params.feature_radius = std::stod(value);
        else if (key == "--z-min") args.params.z_min = std::stod(value);
        else if (key == "--z-max") args.params.z_max = std::stod(value);
        else if (key == "--min-curvature") args.params.min_curvature = std::stod(value);
        else if (key == "--max-features") args.params.max_features = std::stoi(value);
        else if (key == "--threads") args.params.num_threads = std::stoi(value);
        else throw std::invalid_argument("unknown option: " + key);
    }
    if (args.map_path.empty() || args.output_prefix.empty()) {
        throw std::invalid_argument("--map and --output-prefix are required");
    }
    return args;
}

double seconds_since(const std::chrono::steady_clock::time_point &start)
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

}  // namespace

int main(int argc, char **argv)
{
    try {
        const Arguments args = parse_arguments(argc, argv);
        const auto total_start = std::chrono::steady_clock::now();
        Cloud::Ptr map(new Cloud);
        std::cout << "[fpfh_db] map loading: " << args.map_path << std::endl;
        if (pcl::io::loadPCDFile(args.map_path, *map) < 0) {
            throw std::runtime_error("failed to load map PCD");
        }
        std::cout << "[fpfh_db] loaded " << map->size() << " points; preprocessing..."
                  << std::endl;

        const auto feature_start = std::chrono::steady_clock::now();
        const FeatureSet set = icp_relocalization::compute_features(map, args.params);
        if (set.points->size() < 30) {
            throw std::runtime_error("fewer than 30 valid map FPFH features");
        }
        std::cout << "[fpfh_db] input=" << set.input_size
                  << " voxel_points=" << set.filtered_size
                  << " selected_features=" << set.points->size()
                  << " feature_time=" << seconds_since(feature_start) << "s" << std::endl;

        const std::string points_path = args.output_prefix + "_points.pcd";
        const std::string normals_path = args.output_prefix + "_normals.pcd";
        const std::string features_path = args.output_prefix + "_fpfh.pcd";
        const std::string metadata_path = args.output_prefix + ".meta";
        if (pcl::io::savePCDFileBinary(points_path, *set.points) < 0 ||
            pcl::io::savePCDFileBinary(normals_path, *set.normals) < 0 ||
            pcl::io::savePCDFileBinary(features_path, *set.features) < 0) {
            throw std::runtime_error("failed to save FPFH DB PCD files");
        }

        std::ofstream metadata(metadata_path);
        if (!metadata) {
            throw std::runtime_error("failed to write metadata: " + metadata_path);
        }
        metadata << std::setprecision(12)
                 << "format_version=1\n"
                 << "map_path=" << args.map_path << "\n"
                 << "map_fingerprint=" << icp_relocalization::file_fingerprint(args.map_path) << "\n"
                 << "map_input_points=" << set.input_size << "\n"
                 << "map_voxel_points=" << set.filtered_size << "\n"
                 << "feature_count=" << set.points->size() << "\n"
                 << "voxel=" << args.params.voxel << "\n"
                 << "normal_radius=" << args.params.normal_radius << "\n"
                 << "feature_radius=" << args.params.feature_radius << "\n"
                 << "z_min=" << args.params.z_min << "\n"
                 << "z_max=" << args.params.z_max << "\n"
                 << "min_curvature=" << args.params.min_curvature << "\n"
                 << "max_features=" << args.params.max_features << "\n";
        metadata.close();

        std::cout << "[fpfh_db] saved prefix=" << args.output_prefix
                  << " total_time=" << seconds_since(total_start) << "s" << std::endl;
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "[fpfh_db] ERROR: " << error.what() << std::endl;
        return 1;
    }
}
