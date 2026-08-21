#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>

#include <Eigen/Geometry>
#include <teaser/registration.h>

int main()
{
    constexpr int count = 80;
    constexpr int inlier_count = 50;
    std::mt19937 generator(20260805);
    std::uniform_real_distribution<double> coordinate(-5.0, 5.0);
    std::normal_distribution<double> noise(0.0, 0.005);

    const Eigen::Matrix3d expected_rotation =
        Eigen::AngleAxisd(0.55, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d expected_translation(4.0, -2.5, 0.2);
    Eigen::Matrix<double, 3, Eigen::Dynamic> source(3, count);
    Eigen::Matrix<double, 3, Eigen::Dynamic> target(3, count);
    for (int i = 0; i < count; ++i) {
        source.col(i) = Eigen::Vector3d(
            coordinate(generator), coordinate(generator), 0.5 * coordinate(generator));
        if (i < inlier_count) {
            target.col(i) = expected_rotation * source.col(i) + expected_translation +
                            Eigen::Vector3d(
                                noise(generator), noise(generator), noise(generator));
        } else {
            target.col(i) = Eigen::Vector3d(
                3.0 * coordinate(generator), 3.0 * coordinate(generator),
                3.0 * coordinate(generator));
        }
    }

    teaser::RobustRegistrationSolver::Params params;
    params.noise_bound = 0.03;
    params.cbar2 = 1.0;
    params.estimate_scaling = false;
    params.rotation_estimation_algorithm =
        teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
    params.rotation_max_iterations = 100;
    params.rotation_gnc_factor = 1.4;
    params.inlier_selection_mode =
        teaser::RobustRegistrationSolver::INLIER_SELECTION_MODE::PMC_HEU;

    teaser::RobustRegistrationSolver solver(params);
    solver.solve(source, target);
    const teaser::RegistrationSolution solution = solver.getSolution();
    const double translation_error =
        (solution.translation - expected_translation).norm();
    const double cosine = std::max(
        -1.0, std::min(1.0,
            ((expected_rotation.transpose() * solution.rotation).trace() - 1.0) / 2.0));
    const double rotation_error_deg = std::acos(cosine) * 180.0 / M_PI;
    const std::size_t clique = solver.getInlierMaxClique().size();

    std::cout << "TEASER++ self-test: valid=" << solution.valid
              << " clique=" << clique << "/" << count
              << " translation_error=" << translation_error << "m"
              << " rotation_error=" << rotation_error_deg << "deg" << std::endl;
    if (!solution.valid || clique < 20 || translation_error > 0.05 ||
        rotation_error_deg > 0.5) {
        std::cerr << "TEASER++ self-test FAILED" << std::endl;
        return 1;
    }
    std::cout << "TEASER++ self-test PASSED" << std::endl;
    return 0;
}
