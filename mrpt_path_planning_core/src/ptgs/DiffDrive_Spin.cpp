/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/ptgs/DiffDrive_Spin.h>
#include <mrpt/serialization/CArchive.h>

#include <algorithm>
#include <cmath>

using namespace mpp::ptg;

IMPLEMENTS_SERIALIZABLE(
    DiffDrive_Spin, mrpt::nav::CParameterizedTrajectoryGenerator, mpp::ptg)

namespace
{
// Remaining rotation [rad] at which a trajectory ends:
constexpr double kEndTolerance = 1e-5;
}  // namespace

void DiffDrive_Spin::loadDefaultParams()
{
    DiffDriveCollisionGridBased::loadDefaultParams();
    m_alphaValuesCount = 2;
    V_MAX              = 0;
    updateRefDistance();
}

void DiffDrive_Spin::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& cfg, const std::string& sSection)
{
    // Not calling the base classes loaders, which require parameters
    // meaningless for rotations in place (see class docs):
    {
        uint64_t numPaths = m_alphaValuesCount > 0 ? m_alphaValuesCount : 2;
        MRPT_LOAD_HERE_CONFIG_VAR(num_paths, uint64_t, numPaths, cfg, sSection);
        ASSERT_GE_(numPaths, 2U);
        m_alphaValuesCount = static_cast<uint16_t>(numPaths);
    }
    MRPT_LOAD_HERE_CONFIG_VAR(
        score_priority, double, m_score_priority, cfg, sSection);
    CPTG_RobotShape_Polygonal::loadShapeFromConfigFile(cfg, sSection);

    MRPT_LOAD_HERE_CONFIG_VAR_NO_DEFAULT(
        resolution, double, m_resolution, cfg, sSection);
    MRPT_LOAD_HERE_CONFIG_VAR_DEGREES_NO_DEFAULT(
        w_max_dps, double, W_MAX, cfg, sSection);
    MRPT_LOAD_CONFIG_VAR(turningRadiusReference, double, cfg, sSection);
    MRPT_LOAD_HERE_CONFIG_VAR(clearance, double, m_clearance, cfg, sSection);
    ASSERT_GE_(m_clearance, 0.0);
    ASSERT_GT_(turningRadiusReference, 0.0);

    V_MAX = 0;
    updateRefDistance();
}

void DiffDrive_Spin::saveToConfigFile(
    mrpt::config::CConfigFileBase& cfg, const std::string& sSection) const
{
    DiffDriveCollisionGridBased::saveToConfigFile(cfg, sSection);
}

void DiffDrive_Spin::updateRefDistance()
{
    // A small margin, so the integration never stops by distance before
    // reaching the target rotation:
    refDistance = 1.01 * M_PI * turningRadiusReference;
}

void DiffDrive_Spin::serializeFrom(
    mrpt::serialization::CArchive& in, uint8_t version)
{
    DiffDriveCollisionGridBased::internal_readFromStream(in);

    switch (version)
    {
        case 0:
            break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    };
}

uint8_t DiffDrive_Spin::serializeGetVersion() const { return 0; }
void    DiffDrive_Spin::serializeTo(mrpt::serialization::CArchive& out) const
{
    DiffDriveCollisionGridBased::internal_writeToStream(out);
}

std::string DiffDrive_Spin::getDescription() const { return "DiffDrive_Spin"; }

double DiffDrive_Spin::rotationForAlpha(double alpha) const
{
    const double alphaMax = index2alpha(m_alphaValuesCount - 1);
    ASSERT_GT_(alphaMax, 0.0);
    return M_PI * alpha / alphaMax;
}

double DiffDrive_Spin::getTrajectoryRotation(uint16_t k) const
{
    return rotationForAlpha(index2alpha(k));
}

void DiffDrive_Spin::ptgDiffDriveSteeringFunction(
    float alpha, [[maybe_unused]] float t, [[maybe_unused]] float x,
    [[maybe_unused]] float y, float phi, float& v, float& w) const
{
    v = 0;
    // Full speed, except for the last integration step, which ends exactly at
    // the target rotation:
    const double remaining = rotationForAlpha(alpha) - phi;
    w                      = static_cast<float>(
        mrpt::sign(remaining) *
        std::min(W_MAX, std::abs(remaining) / m_stepTimeDuration));
}

bool DiffDrive_Spin::ptgDiffDriveTrajectoryEnded(
    float alpha, [[maybe_unused]] float t, [[maybe_unused]] float x,
    [[maybe_unused]] float y, float phi) const
{
    return std::abs(rotationForAlpha(alpha) - phi) < kEndTolerance;
}

bool DiffDrive_Spin::PTG_IsIntoDomain(
    [[maybe_unused]] double x, [[maybe_unused]] double y) const
{
    return false;
}

std::optional<std::pair<int, double>> DiffDrive_Spin::inverseMap_WS2TP(
    [[maybe_unused]] double x, [[maybe_unused]] double y,
    [[maybe_unused]] double tolerance_dist) const
{
    return std::nullopt;
}

std::optional<std::pair<uint16_t, uint32_t>>
    DiffDrive_Spin::rotationToTrajectoryStep(double angle) const
{
    ASSERTMSG_(!m_trajectory.empty(), "PTG has not been initialized!");
    if (std::abs(angle) > M_PI + kEndTolerance) { return std::nullopt; }

    const auto k =
        static_cast<uint16_t>(angle > 0 ? m_alphaValuesCount - 1 : 0);
    const auto& traj = m_trajectory.at(k);

    // Headings along the trajectory are monotonic: find the closest one.
    const double target = std::abs(angle);
    const auto   it     = std::lower_bound(
              traj.begin(), traj.end(), target,
              [](const TCPoint& p, double a) { return std::abs(p.phi) < a; });
    auto step = static_cast<uint32_t>(std::distance(traj.begin(), it));
    if (step >= traj.size()) { step = static_cast<uint32_t>(traj.size() - 1); }
    if (step > 0 && std::abs(std::abs(traj[step - 1].phi) - target) <
                        std::abs(std::abs(traj[step].phi) - target))
    {
        step--;
    }
    if (step == 0) { return std::nullopt; }
    return std::make_pair(k, step);
}
