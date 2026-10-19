/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/ptgs/DiffDriveCollisionGridBased.h>

#include <cstdint>
#include <optional>
#include <utility>

/** Feature macro: the mpp::ptg::DiffDrive_Spin PTG exists. */
#define MPP_HAS_SPIN_PTG 1

namespace mpp::ptg
{
/** A PTG for rotations in place of differential-driven vehicles.
 * - **Compatible kinematics**: differential-driven (not Ackermann steering)
 * - **Compatible robot shape**: Arbitrary 2D polygon
 *
 * Trajectory `k` rotates in place (v=0) at the maximum angular speed until
 * the heading changes by:
 *
 * \f[ \theta_k = \pi \dfrac{\alpha_k}{\alpha_{max}} \f]
 *
 * so the two extreme trajectories turn by +pi (left) and -pi (right), and the
 * others by intermediate angles. Any rotation of up to pi is a prefix of one
 * of the extreme trajectories, so the default number of trajectories is 2.
 *
 * The TP-Space distance of a rotation by `theta` is
 * `|theta| * turningRadiusReference`, like the rotational part of other
 * differential-drive PTGs. Since all trajectories stay at the origin, the
 * workspace-to-TP-Space inverse map is not defined (see inverseMap_WS2TP()):
 * use rotationToTrajectoryStep() instead.
 *
 * Configuration parameters (with the usual `PTG<n>_` prefix):
 *  - `w_max_dps`: Rotation speed [deg/s] (required).
 *  - `resolution`: Collision grid cell size [m] (required).
 *  - `num_paths`: Number of trajectories (default: 2).
 *  - `turningRadiusReference`, `clearance`, robot shape: as in other PTGs.
 *
 * `refDistance` and `v_max_mps` are not used: the former is set from the
 * maximum rotation, and the linear speed is always zero.
 */
class DiffDrive_Spin : public DiffDriveCollisionGridBased
{
    DEFINE_SERIALIZABLE(DiffDrive_Spin, mpp::ptg)
   public:
    DiffDrive_Spin() = default;
    DiffDrive_Spin(
        const mrpt::config::CConfigFileBase& cfg, const std::string& sSection)
    {
        DiffDrive_Spin::loadFromConfigFile(cfg, sSection);
    }
    void loadFromConfigFile(
        const mrpt::config::CConfigFileBase& cfg,
        const std::string&                   sSection) override;
    void saveToConfigFile(
        mrpt::config::CConfigFileBase& cfg,
        const std::string&             sSection) const override;

    std::string getDescription() const override;

    /** Always nullopt: all trajectories stay at the origin. */
    [[nodiscard]] std::optional<std::pair<int, double>> inverseMap_WS2TP(
        double x, double y, double tolerance_dist = 0.10) const override;
    bool PTG_IsIntoDomain(double x, double y) const override;
    void ptgDiffDriveSteeringFunction(
        float alpha, float t, float x, float y, float phi, float& v,
        float& w) const override;
    bool ptgDiffDriveTrajectoryEnded(
        float alpha, float t, float x, float y, float phi) const override;
    void loadDefaultParams() override;

    /** Signed rotation [rad] at the end of trajectory `k`. */
    double getTrajectoryRotation(uint16_t k) const;

    /** Trajectory index and step that rotate in place by `angle` [rad], up
     * to the time resolution of the trajectories. Requires initialize().
     * \return nullopt if |angle| exceeds pi, or if it is reached at step 0.
     */
    std::optional<std::pair<uint16_t, uint32_t>> rotationToTrajectoryStep(
        double angle) const;

   private:
    /** Rotation [rad] of the trajectory with parameter `alpha`. */
    double rotationForAlpha(double alpha) const;

    /** Sets refDistance to the TP-Space distance of the maximum rotation. */
    void updateRefDistance();
};
}  // namespace mpp::ptg
