/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mpp/ptgs/DiffDriveCollisionGridBased.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/kinematics/CVehicleVelCmd_DiffDriven.h>
#include <mrpt/math/geometry.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/system/CTicTac.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <utility>

using namespace mpp::ptg;
using mrpt::d2f;
using mrpt::format;

namespace
{
// Time step used to simulate the PTG trajectories [s]
constexpr float kTrajectoryTimeStep = 1.0e-3f;

double pointSegmentDistance(
    const mrpt::math::TPoint2D& p, const mrpt::math::TPoint2D& a,
    const mrpt::math::TPoint2D& b)
{
    const double abx  = b.x - a.x;
    const double aby  = b.y - a.y;
    const double len2 = abx * abx + aby * aby;
    double       t    = 0;
    if (len2 > 0)
    {
        t = ((p.x - a.x) * abx + (p.y - a.y) * aby) / len2;
        t = std::clamp(t, 0.0, 1.0);
    }
    return std::hypot(p.x - (a.x + t * abx), p.y - (a.y + t * aby));
}

struct Box2D
{
    double x0 = 0;
    double x1 = 0;
    double y0 = 0;
    double y1 = 0;
};

double pointBoxDistance(const mrpt::math::TPoint2D& p, const Box2D& b)
{
    const double dx = std::max({b.x0 - p.x, 0.0, p.x - b.x1});
    const double dy = std::max({b.y0 - p.y, 0.0, p.y - b.y1});
    return std::hypot(dx, dy);
}

// Liang-Barsky clipping test: does segment [a,b] touch the closed box?
bool segmentIntersectsBox(
    const mrpt::math::TPoint2D& a, const mrpt::math::TPoint2D& b,
    const Box2D& box)
{
    double       t0   = 0;
    double       t1   = 1;
    const double dx   = b.x - a.x;
    const double dy   = b.y - a.y;
    const double p[4] = {-dx, dx, -dy, dy};
    const double q[4] = {
        a.x - box.x0, box.x1 - a.x, a.y - box.y0, box.y1 - a.y};
    for (int i = 0; i < 4; i++)
    {
        if (p[i] == 0)
        {
            if (q[i] < 0) { return false; }
            continue;
        }
        const double r = q[i] / p[i];
        if (p[i] < 0) { t0 = std::max(t0, r); }
        else { t1 = std::min(t1, r); }
        if (t0 > t1) { return false; }
    }
    return true;
}

// Exact Euclidean distance between an axis-aligned box and a (possibly
// non-convex) simple polygon region. Zero if they overlap.
double boxPolygonDistance(const Box2D& box, const mrpt::math::TPolygon2D& poly)
{
    const mrpt::math::TPoint2D center(
        0.5 * (box.x0 + box.x1), 0.5 * (box.y0 + box.y1));
    if (poly.contains(center)) { return 0; }

    const mrpt::math::TPoint2D corners[4] = {
        {box.x0, box.y0}, {box.x1, box.y0}, {box.x1, box.y1}, {box.x0, box.y1}};

    double       dmin = std::numeric_limits<double>::max();
    const size_t N    = poly.size();
    for (size_t i = 0; i < N; i++)
    {
        const auto& a = poly[i];
        const auto& b = poly[(i + 1) % N];
        if (segmentIntersectsBox(a, b, box)) { return 0; }
        // Two disjoint convex sets (segment, box): the closest pair involves a
        // vertex of one of them.
        for (const auto& c : corners)
        {
            dmin = std::min(dmin, pointSegmentDistance(c, a, b));
        }
        dmin = std::min(dmin, pointBoxDistance(a, box));
    }
    return dmin;
}

}  // namespace

/** Constructor: possible values in "params":
 *   - ref_distance: The maximum distance in PTGs
 *   - resolution: The cell size
 *   - v_max, w_max: Maximum robot speeds.
 */
DiffDriveCollisionGridBased::DiffDriveCollisionGridBased()
    : m_collisionGrid(-1, 1, -1, 1, 0.5, this)
{
}

void DiffDriveCollisionGridBased::loadDefaultParams()
{
    CParameterizedTrajectoryGenerator::loadDefaultParams();
    CPTG_RobotShape_Polygonal::loadDefaultParams();

    m_resolution = 0.10;
    m_clearance  = 0.0;
    V_MAX        = 1.0;
    W_MAX        = mrpt::DEG2RAD(120);
}

void DiffDriveCollisionGridBased::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& cfg, const std::string& sSection)
{
    CParameterizedTrajectoryGenerator::loadFromConfigFile(cfg, sSection);
    CPTG_RobotShape_Polygonal::loadShapeFromConfigFile(cfg, sSection);

    MRPT_LOAD_HERE_CONFIG_VAR_NO_DEFAULT(
        resolution, double, m_resolution, cfg, sSection);
    MRPT_LOAD_HERE_CONFIG_VAR_NO_DEFAULT(
        v_max_mps, double, V_MAX, cfg, sSection);
    MRPT_LOAD_HERE_CONFIG_VAR_DEGREES_NO_DEFAULT(
        w_max_dps, double, W_MAX, cfg, sSection);
    MRPT_LOAD_CONFIG_VAR(turningRadiusReference, double, cfg, sSection);
    MRPT_LOAD_HERE_CONFIG_VAR(clearance, double, m_clearance, cfg, sSection);
    ASSERT_GE_(m_clearance, 0.0);
}
void DiffDriveCollisionGridBased::saveToConfigFile(
    mrpt::config::CConfigFileBase& cfg, const std::string& sSection) const
{
    MRPT_START
    const int WN = 25, WV = 30;

    CParameterizedTrajectoryGenerator::saveToConfigFile(cfg, sSection);

    cfg.write(
        sSection, "resolution", m_resolution, WN, WV,
        "Resolution of the collision-check look-up-table [m].");
    cfg.write(
        sSection, "v_max_mps", V_MAX, WN, WV,
        "Maximum linear velocity for trajectories [m/s].");
    cfg.write(
        sSection, "w_max_dps", mrpt::RAD2DEG(W_MAX), WN, WV,
        "Maximum angular velocity for trajectories [deg/s].");
    cfg.write(
        sSection, "turningRadiusReference", turningRadiusReference, WN, WV,
        "An approximate dimension of the robot (not a critical parameter) "
        "[m].");
    cfg.write(
        sSection, "clearance", m_clearance, WN, WV,
        "Certified clearance between footprint and obstacles [m].");

    CPTG_RobotShape_Polygonal::saveToConfigFile(cfg, sSection);

    MRPT_END
}

mrpt::serialization::CArchive& mpp::ptg::operator<<(
    mrpt::serialization::CArchive& o, const TCPoint& p)
{
    o << p.x << p.y << p.phi << p.t << p.dist << p.v << p.w;
    return o;
}
mrpt::serialization::CArchive& mpp::ptg::operator>>(
    mrpt::serialization::CArchive& i, TCPoint& p)
{
    i >> p.x >> p.y >> p.phi >> p.t >> p.dist >> p.v >> p.w;
    return i;
}

/*---------------------------------------------------------------
                    simulateTrajectories
    Solve trajectories and fill cells.
  ---------------------------------------------------------------*/
void DiffDriveCollisionGridBased::simulateTrajectories(
    float max_time, float max_dist, float dt)
{
    using mrpt::square;

    internal_deinitialize();  // Free previous paths

    m_stepTimeDuration = dt;

    // Reserve the size in the buffers:
    m_trajectory.resize(m_alphaValuesCount);

    // For the grid:
    float x_min = 1e3f, x_max = -1e3;
    float y_min = 1e3f, y_max = -1e3;

    for (unsigned int k = 0; k < m_alphaValuesCount; k++)
    {
        // Simulate / evaluate the trajectory selected by this "alpha":
        // ------------------------------------------------------------
        const float alpha = index2alpha(k);

        TCPointVector points;

        float t = .0f, dist = .0f, turned = .0f;
        float x = .0f, y = .0f, phi = .0f, v = .0f, w = .0f;

        // Add the first, initial point:
        points.push_back(TCPoint(x, y, phi, t, dist, v, w));

        // Simulate until...
        while (t < max_time && dist < max_dist && fabs(turned) < 1.95 * M_PI)
        {
            // Compute new movement command (v,w):
            ptgDiffDriveSteeringFunction(alpha, t, x, y, phi, v, w);

            // Finite difference equation:
            x += cos(phi) * v * dt;
            y += sin(phi) * v * dt;
            phi += w * dt;

            // Counters:
            turned += w * dt;

            float v_inTPSpace =
                sqrt(square(v) + square(w * turningRadiusReference));

            dist += v_inTPSpace * dt;

            t += dt;

            // Set the (v,w) to the last record:
            points.back().v = v;
            points.back().w = w;

            // And add the new record:
            points.push_back(TCPoint(x, y, phi, t, dist, v, w));

            // for the grid:
            x_min = std::min(x_min, x);
            x_max = std::max(x_max, x);
            y_min = std::min(y_min, y);
            y_max = std::max(y_max, y);
        }

        // Add the final point:
        points.back().v = v;
        points.back().w = w;
        points.push_back(TCPoint(x, y, phi, t, dist, v, w));

        // Save data to C-Space path structure:
        m_trajectory[k] = std::move(points);

    }  // end for "k"

    // --------------------------------------------------------
    // Build the speeding-up grid for lambda function:
    // --------------------------------------------------------
    const TCellForLambdaFunction defaultCell;
    m_lambdaFunctionOptimizer.setSize(
        x_min - 0.5f, x_max + 0.5f, y_min - 0.5f, y_max + 0.5f, 0.25f,
        &defaultCell);

    for (uint16_t k = 0; k < m_alphaValuesCount; k++)
    {
        const auto M = static_cast<uint32_t>(m_trajectory[k].size());
        for (uint32_t n = 0; n < M; n++)
        {
            TCellForLambdaFunction* cell = m_lambdaFunctionOptimizer.cellByPos(
                m_trajectory[k][n].x, m_trajectory[k][n].y);
            ASSERT_(cell);
            // Keep limits:
            mrpt::keep_min(cell->k_min, k);
            mrpt::keep_max(cell->k_max, k);
            mrpt::keep_min(cell->n_min, n);
            mrpt::keep_max(cell->n_max, n);
        }
    }
}

/** In this class, `out_action_cmd` contains: [0]: linear velocity (m/s),  [1]:
 * angular velocity (rad/s) */
mrpt::kinematics::CVehicleVelCmd::Ptr
    DiffDriveCollisionGridBased::directionToMotionCommand(uint16_t k) const
{
    float v, w;
    ptgDiffDriveSteeringFunction(index2alpha(k), 0, 0, 0, 0, v, w);

    auto* cmd    = new mrpt::kinematics::CVehicleVelCmd_DiffDriven();
    cmd->lin_vel = v;
    cmd->ang_vel = w;
    return mrpt::kinematics::CVehicleVelCmd::Ptr(cmd);
}

/*---------------------------------------------------------------
                    getTPObstacle
  ---------------------------------------------------------------*/
const DiffDriveCollisionGridBased::TCollisionCell&
    DiffDriveCollisionGridBased::CCollisionGrid::getTPObstacle(
        const float obsX, const float obsY) const
{
    static const TCollisionCell emptyCell;
    const TCollisionCell*       cell = cellByPos(obsX, obsY);
    return cell != nullptr ? *cell : emptyCell;
}

/*---------------------------------------------------------------
    Updates the info into a cell: It updates the cell only
      if the distance d for the path k is lower than the previous value:
  ---------------------------------------------------------------*/
void DiffDriveCollisionGridBased::CCollisionGrid::updateCellInfo(
    const unsigned int icx, const unsigned int icy, const uint16_t k,
    const float dist)
{
    TCollisionCell* cell = cellByIndex(icx, icy);
    if (!cell) return;

    // For such a small number of elements, brute-force search is not such a bad
    // idea:
    auto itK = cell->end();
    for (auto it = cell->begin(); it != cell->end(); ++it)
        if (it->first == k)
        {
            itK = it;
            break;
        }

    if (itK == cell->end())
    {  // New entry:
        cell->push_back(std::make_pair(k, dist));
    }
    else
    {  // Only update that "k" if the distance is shorter now:
        if (dist < itK->second) itK->second = dist;
    }
}

/*---------------------------------------------------------------
                    Save to file
  ---------------------------------------------------------------*/
bool DiffDriveCollisionGridBased::saveColGridsToFile(
    const std::string&          filename,
    const mrpt::math::CPolygon& computed_robotShape) const
{
    try
    {
        mrpt::io::CFileGZOutputStream fo(filename);
        if (!fo.fileOpenCorrectly()) return false;

        const uint32_t n    = 1;  // for backwards compatibility...
        auto           arch = mrpt::serialization::archiveFrom(fo);
        arch << n;
        return m_collisionGrid.saveToFile(&arch, computed_robotShape);
    }
    catch (...)
    {
        return false;
    }
}

/*---------------------------------------------------------------
                    Load from file
  ---------------------------------------------------------------*/
bool DiffDriveCollisionGridBased::loadColGridsFromFile(
    const std::string& filename, const mrpt::math::CPolygon& current_robotShape)
{
    try
    {
        mrpt::io::CFileGZInputStream fi(filename);
        if (!fi.fileOpenCorrectly()) return false;
        auto arch = mrpt::serialization::archiveFrom(fi);

        uint32_t n;
        arch >> n;
        if (n != 1)
            return false;  // Incompatible (old) format, just discard and
        // recompute.

        return m_collisionGrid.loadFromFile(&arch, current_robotShape);
    }
    catch (...)
    {
        return false;
    }
}

const uint32_t COLGRID_FILE_MAGIC = 0xC0C0C0C3;

/*---------------------------------------------------------------
                    Save to file
  ---------------------------------------------------------------*/
bool DiffDriveCollisionGridBased::CCollisionGrid::saveToFile(
    mrpt::serialization::CArchive* f,
    const mrpt::math::CPolygon&    computed_robotShape) const
{
    try
    {
        if (!f) return false;

        // v1: As of jun 2012, v2: As of dec-2013, v3: conservative
        // (certified) cell marking + clearance, sep-2026.
        const uint8_t serialize_version = 3;

        // Save magic signature && serialization version:
        *f << COLGRID_FILE_MAGIC << serialize_version;

        // Robot shape:
        *f << computed_robotShape;
        *f << m_parent->m_clearance;

        // and standard PTG data:
        *f << m_parent->getDescription() << m_parent->getAlphaValuesCount()
           << d2f(m_parent->getMax_V()) << d2f(m_parent->getMax_W());

        *f << m_x_min << m_x_max << m_y_min << m_y_max;
        *f << m_resolution;

        // v1 was:  *f << m_map;
        uint32_t N = m_map.size();
        *f << N;
        for (uint32_t i = 0; i < N; i++)
        {
            uint32_t M = m_map[i].size();
            *f << M;
            for (uint32_t k = 0; k < M; k++)
                *f << m_map[i][k].first << m_map[i][k].second;
        }

        return true;
    }
    catch (...)
    {
        return false;
    }
}

/*---------------------------------------------------------------
                        loadFromFile
  ---------------------------------------------------------------*/
bool DiffDriveCollisionGridBased::CCollisionGrid::loadFromFile(
    mrpt::serialization::CArchive* f,
    const mrpt::math::CPolygon&    current_robotShape)
{
    try
    {
        if (!f) return false;

        // Return false if the file contents doesn't match what we expected:
        uint32_t file_magic;
        *f >> file_magic;

        // It doesn't seem to be a valid file or was in an old format, just
        // recompute the grid:
        if (COLGRID_FILE_MAGIC != file_magic) return false;

        uint8_t serialized_version;
        *f >> serialized_version;

        switch (serialized_version)
        {
            case 3:
            {
                mrpt::math::CPolygon stored_shape;
                *f >> stored_shape;

                const bool shapes_match =
                    (stored_shape.size() == current_robotShape.size() &&
                     std::equal(
                         stored_shape.begin(), stored_shape.end(),
                         current_robotShape.begin()));

                if (!shapes_match)
                {
                    // Must recompute if the robot shape changed.
                    return false;
                }

                double stored_clearance = 0;
                *f >> stored_clearance;
                if (std::abs(stored_clearance - m_parent->m_clearance) > 1e-9)
                {
                    return false;
                }
            }
            break;

            // Older versions used a non-conservative cell marking rule:
            // always rebuild them.
            case 1:
            case 2:
            default:
                // Unknown version: Maybe we are loading a file from a more
                // recent version of MRPT? Whatever, we can't read it: It's
                // safer just to re-generate the PTG data
                return false;
        };

        // Standard PTG data:
        const std::string expected_desc = m_parent->getDescription();
        std::string       desc;
        *f >> desc;
        if (desc != expected_desc) return false;

// and standard PTG data:
#define READ_UINT16_CHECK_IT_MATCHES_STORED(_VAR) \
    {                                             \
        uint16_t ff;                              \
        *f >> ff;                                 \
        if (ff != _VAR) return false;             \
    }
#define READ_FLOAT_CHECK_IT_MATCHES_STORED(_VAR)       \
    {                                                  \
        float ff;                                      \
        *f >> ff;                                      \
        if (std::abs(ff - _VAR) > 1e-4f) return false; \
    }
#define READ_DOUBLE_CHECK_IT_MATCHES_STORED(_VAR)     \
    {                                                 \
        double ff;                                    \
        *f >> ff;                                     \
        if (std::abs(ff - _VAR) > 1e-6) return false; \
    }

        READ_UINT16_CHECK_IT_MATCHES_STORED(m_parent->getAlphaValuesCount())
        READ_FLOAT_CHECK_IT_MATCHES_STORED(m_parent->getMax_V())
        READ_FLOAT_CHECK_IT_MATCHES_STORED(m_parent->getMax_W())

        // Cell dimensions:
        READ_DOUBLE_CHECK_IT_MATCHES_STORED(m_x_min)
        READ_DOUBLE_CHECK_IT_MATCHES_STORED(m_x_max)
        READ_DOUBLE_CHECK_IT_MATCHES_STORED(m_y_min)
        READ_DOUBLE_CHECK_IT_MATCHES_STORED(m_y_max)
        READ_DOUBLE_CHECK_IT_MATCHES_STORED(m_resolution)

        // OK, all parameters seem to be exactly the same than when we
        // precomputed the table: load it.
        // v1 was:  *f >> m_map;
        uint32_t N;
        *f >> N;
        m_map.resize(N);
        for (uint32_t i = 0; i < N; i++)
        {
            uint32_t M;
            *f >> M;
            m_map[i].resize(M);
            for (uint32_t k = 0; k < M; k++)
                *f >> m_map[i][k].first >> m_map[i][k].second;
        }

        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[CCollisionGrid::loadFromFile] " << e.what();
        return false;
    }
    catch (...)
    {
        return false;
    }
}

std::optional<std::pair<int, double>>
    DiffDriveCollisionGridBased::inverseMap_WS2TP(
        double x, double y, double tolerance_dist) const
{
    using mrpt::square;

    ASSERTMSG_(
        m_alphaValuesCount > 0,
        "Have you called simulateTrajectories() first?");

    // -------------------------------------------------------------------
    // Optimization: (24-JAN-2007 @ Jose Luis Blanco):
    //  Use a "grid" to determine the range of [k,d] values to check!!
    //  If the point (x,y) is not found in the grid, then directly skip
    //  to the next step.
    // -------------------------------------------------------------------
    uint16_t k_min = 0, k_max = m_alphaValuesCount - 1;
    uint32_t n_min = 0, n_max = 0;
    bool     at_least_one = false;

    // Cell indexes:
    int cx0 = m_lambdaFunctionOptimizer.x2idx(x);
    int cy0 = m_lambdaFunctionOptimizer.y2idx(y);

    // (cx,cy)
    for (int cx = cx0 - 1; cx <= cx0 + 1; cx++)
    {
        for (int cy = cy0 - 1; cy <= cy0 + 1; cy++)
        {
            const TCellForLambdaFunction* cell =
                m_lambdaFunctionOptimizer.cellByIndex(cx, cy);
            if (cell && !cell->isEmpty())
            {
                if (!at_least_one)
                {
                    k_min        = cell->k_min;
                    k_max        = cell->k_max;
                    n_min        = cell->n_min;
                    n_max        = cell->n_max;
                    at_least_one = true;
                }
                else
                {
                    mrpt::keep_min(k_min, cell->k_min);
                    mrpt::keep_max(k_max, cell->k_max);

                    mrpt::keep_min(n_min, cell->n_min);
                    mrpt::keep_max(n_max, cell->n_max);
                }
            }
        }
    }

    // Try to find a closest point to the paths:
    // ----------------------------------------------
    int   selected_k    = -1;
    float selected_d    = 0.0f;
    float selected_dist = std::numeric_limits<float>::max();

    if (at_least_one)  // Otherwise, don't even lose time checking...
    {
        ASSERT_LT_(k_max, m_trajectory.size());
        for (int k = k_min; k <= k_max; k++)
        {
            const size_t   n_real = m_trajectory[k].size();
            const uint32_t n_max_this =
                std::min(static_cast<uint32_t>(n_real ? n_real - 1 : 0), n_max);

            for (uint32_t n = n_min; n <= n_max_this; n++)
            {
                const float dist_a_punto = square(m_trajectory[k][n].x - x) +
                                           square(m_trajectory[k][n].y - y);
                if (dist_a_punto < selected_dist)
                {
                    selected_dist = dist_a_punto;
                    selected_k    = k;
                    selected_d    = m_trajectory[k][n].dist;
                }
            }
        }
    }

    if (selected_k != -1)
    {
        const double out_d = selected_d / refDistance;
        if (selected_dist <= square(tolerance_dist))
        {
            return std::make_pair(selected_k, out_d);
        }
        return std::nullopt;
    }

    // Not found within the simulated paths: report as not reachable by this
    // set of trajectories.
    return std::nullopt;
}

void DiffDriveCollisionGridBased::setRefDistance(const double refDist)
{
    ASSERTMSG_(
        m_trajectory.empty(),
        "Changing reference distance not allowed in this class after "
        "initialization!");
    this->refDistance = refDist;
}

void DiffDriveCollisionGridBased::internal_processNewRobotShape()
{
    ASSERTMSG_(
        m_trajectory.empty(),
        "Changing robot shape not allowed in this class after initialization!");
}

void DiffDriveCollisionGridBased::internal_deinitialize()
{
    m_trajectory.clear();  // Free trajectories
}

void DiffDriveCollisionGridBased::internal_initialize(
    const std::string& cacheFilename, const bool verbose)
{
    using namespace std;

    MRPT_START

    if (verbose)
        cout << endl
             << "[CPTG_DiffDrive_CollisionGridBased::initialize] Starting... "
                "*** THIS MAY TAKE A WHILE, BUT MUST BE COMPUTED ONLY ONCE!! **"
             << endl;

    // Sanity checks:
    ASSERTMSG_(!m_robotShape.empty(), "Robot shape was not defined");
    ASSERTMSG_(
        m_robotShape.size() >= 3, "Robot shape must have 3 or more vertices");
    ASSERT_(refDistance > 0);
    ASSERT_(V_MAX > 0);
    ASSERT_(W_MAX > 0);
    ASSERT_(m_resolution > 0);

    m_robotRadius = 0;
    for (size_t m = 0; m < m_robotShape.size(); m++)
    {
        mrpt::keep_max(
            m_robotRadius,
            std::hypot(
                m_robotShape.get_vertex_x(m), m_robotShape.get_vertex_y(m)));
    }

    mrpt::system::CTicTac tictac;
    tictac.Tic();

    if (verbose) cout << "Initializing PTG '" << cacheFilename << "'...";

    // Simulate paths:
    float trajectory_max_time = 100.0f;  // [s]

    simulateTrajectories(
        trajectory_max_time,
        refDistance,  // max.dist,
        kTrajectoryTimeStep  // timestep
    );

    // Just for debugging, etc.
    // debugDumpInFiles(n);

    // Check for collisions between the robot shape and the grid cells:
    // ----------------------------------------------------------------------------
    const double gridHalfSize = getObstacleReachDistance();
    m_collisionGrid.setSize(
        -gridHalfSize, gridHalfSize, -gridHalfSize, gridHalfSize, m_resolution);

    const size_t Ki = getAlphaValuesCount();
    ASSERTMSG_(Ki > 0, "The PTG seems to be not initialized!");

    // Load the cached version, if possible
    if (loadColGridsFromFile(cacheFilename, m_robotShape))
    {
        if (verbose) cout << "loaded from file OK" << endl;
    }
    else
    {
        // BUGFIX: In case we start reading the file and in the end detected an
        // error,
        //         we must make sure that there's space enough for the grid:
        m_collisionGrid.setSize(
            -gridHalfSize, gridHalfSize, -gridHalfSize, gridHalfSize,
            m_collisionGrid.getResolution());

        const int    grid_cx_max = m_collisionGrid.getSizeX() - 1;
        const int    grid_cy_max = m_collisionGrid.getSizeY() - 1;
        const double half_cell   = m_collisionGrid.getResolution() * 0.5;

        const size_t nVerts      = m_robotShape.size();
        double       robotRadius = 0;
        for (size_t m = 0; m < nVerts; m++)
        {
            mrpt::keep_max(
                robotRadius, std::hypot(
                                 m_robotShape.get_vertex_x(m),
                                 m_robotShape.get_vertex_y(m)));
        }

        // The robot shape at each location:
        std::vector<mrpt::math::TPoint2D> transf_shape(nVerts);

        // RECOMPUTE THE COLLISION GRIDS:
        // ---------------------------------------
        // A cell is marked with the distance of sample "n" if any point of
        // the cell square is closer than "margin" to the footprint at that
        // sample, where "margin" bounds the displacement of any footprint
        // point until the next sample, plus the user clearance. This makes the
        // stored free distances a certified lower bound for ANY obstacle point
        // inside the cell and for the continuous motion between samples.
        for (size_t k = 0; k < Ki; k++)
        {
            const size_t nPoints = getPathStepCount(k);
            ASSERT_(nPoints > 1);
            for (size_t n = 0; n < nPoints; n++)
            {
                // Translate and rotate the robot shape at this C-Space pose:
                const mrpt::math::TPose2D p = getPathPose(k, n);

                // Upper bound of the displacement of any footprint point along
                // the (constant-twist) motion until the next sample: center
                // arc length (chord times the arc/chord ratio) plus the
                // rotation lever arm.
                double sweep = 0;
                if (n + 1 < nPoints)
                {
                    const mrpt::math::TPose2D pNext = getPathPose(k, n + 1);
                    const double              chord =
                        std::hypot(pNext.x - p.x, pNext.y - p.y);
                    const double dPhi =
                        std::abs(mrpt::math::angDistance(p.phi, pNext.phi));
                    const double halfPhi = 0.5 * dPhi;
                    const double arcRatio =
                        halfPhi > 1e-9 ? halfPhi / std::sin(halfPhi) : 1.0;
                    sweep = chord * arcRatio + robotRadius * dPhi;
                }
                const double margin = sweep + m_clearance;

                mrpt::math::TPoint2D bb_min(
                    std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max());
                mrpt::math::TPoint2D bb_max(
                    -std::numeric_limits<double>::max(),
                    -std::numeric_limits<double>::max());

                for (size_t m = 0; m < nVerts; m++)
                {
                    transf_shape[m].x =
                        p.x + cos(p.phi) * m_robotShape.get_vertex_x(m) -
                        sin(p.phi) * m_robotShape.get_vertex_y(m);
                    transf_shape[m].y =
                        p.y + sin(p.phi) * m_robotShape.get_vertex_x(m) +
                        cos(p.phi) * m_robotShape.get_vertex_y(m);
                    mrpt::keep_max(bb_max.x, transf_shape[m].x);
                    mrpt::keep_max(bb_max.y, transf_shape[m].y);
                    mrpt::keep_min(bb_min.x, transf_shape[m].x);
                    mrpt::keep_min(bb_min.y, transf_shape[m].y);
                }

                // Robot shape polygon:
                const mrpt::math::TPolygon2D poly(transf_shape);

                // Range of cells that may be within "margin" of this shape:
                const int ix_min =
                    std::max(0, m_collisionGrid.x2idx(bb_min.x - margin) - 1);
                const int iy_min =
                    std::max(0, m_collisionGrid.y2idx(bb_min.y - margin) - 1);
                const int ix_max = std::min(
                    m_collisionGrid.x2idx(bb_max.x + margin) + 1, grid_cx_max);
                const int iy_max = std::min(
                    m_collisionGrid.y2idx(bb_max.y + margin) + 1, grid_cy_max);

                const float d = this->getPathDist(k, n);

                for (int ix = ix_min; ix <= ix_max; ix++)
                {
                    const double cx = m_collisionGrid.idx2x(ix);

                    for (int iy = iy_min; iy <= iy_max; iy++)
                    {
                        const double cy = m_collisionGrid.idx2y(iy);

                        // Farther than any footprint point plus the margin?
                        const double reach =
                            robotRadius + margin + half_cell * M_SQRT2;
                        if (mrpt::square(cx - p.x) + mrpt::square(cy - p.y) >
                            reach * reach)
                        {
                            continue;
                        }

                        // Samples are visited in increasing distance, so an
                        // existing entry for "k" is already the minimum:
                        if (const auto* c = m_collisionGrid.cellByIndex(ix, iy);
                            c != nullptr &&
                            std::any_of(
                                c->begin(), c->end(),
                                [k](const auto& e) { return e.first == k; }))
                        {
                            continue;
                        }

                        const Box2D cellBox{
                            cx - half_cell, cx + half_cell, cy - half_cell,
                            cy + half_cell};

                        if (boxPolygonDistance(cellBox, poly) <= margin)
                        {
                            m_collisionGrid.updateCellInfo(ix, iy, k, d);
                        }
                    }  // for iy
                }  // for ix

            }  // n

            if (verbose) cout << k << "/" << Ki << ",";
        }  // k

        if (verbose) cout << format("Done! [%.03f sec]\n", tictac.Tac());

        // save it to the cache file for the next run:
        saveColGridsToFile(cacheFilename, m_robotShape);

    }  // "else" recompute all PTG

    MRPT_END
}

double DiffDriveCollisionGridBased::getObstacleReachDistance() const
{
    double robotRadius = 0;
    for (size_t m = 0; m < m_robotShape.size(); m++)
    {
        mrpt::keep_max(
            robotRadius,
            std::hypot(
                m_robotShape.get_vertex_x(m), m_robotShape.get_vertex_y(m)));
    }
    // Trajectories never go farther than refDistance from the origin, plus at
    // most one final simulation step, so the footprint (grown by the
    // clearance) stays within this radius. One extra cell accounts for the
    // cell-square test at the border.
    const double lastStepOvershoot = V_MAX * kTrajectoryTimeStep;
    return refDistance + lastStepOvershoot + robotRadius + m_clearance +
           2 * m_resolution;
}

size_t DiffDriveCollisionGridBased::getPathStepCount(uint16_t k) const
{
    ASSERT_(k < m_trajectory.size());

    return m_trajectory[k].size();
}

mrpt::math::TPose2D DiffDriveCollisionGridBased::getPathPose(
    uint16_t k, uint32_t step) const
{
    ASSERT_(k < m_trajectory.size());
    ASSERT_(step < m_trajectory[k].size());

    return {
        m_trajectory[k][step].x, m_trajectory[k][step].y,
        m_trajectory[k][step].phi};
}

double DiffDriveCollisionGridBased::getPathDist(uint16_t k, uint32_t step) const
{
    ASSERT_(k < m_trajectory.size());
    ASSERT_(step < m_trajectory[k].size());

    return m_trajectory[k][step].dist;
}

std::optional<uint32_t> DiffDriveCollisionGridBased::getPathStepForDist(
    uint16_t k, double dist) const
{
    ASSERT_(k < m_trajectory.size());
    const size_t numPoints = m_trajectory[k].size();

    ASSERT_(numPoints > 0);

    for (size_t n = 0; n < numPoints - 1; n++)
    {
        if (m_trajectory[k][n + 1].dist >= dist)
        {
            return static_cast<uint32_t>(n);
        }
    }

    return std::nullopt;
}

bool DiffDriveCollisionGridBased::isObstacleInsideShape(
    double ox, double oy) const
{
    // O(1) rejection first: only points within the circumscribed radius can
    // be inside the polygon, so the O(V) test is rarely needed.
    if (m_robotRadius > 0 && ox * ox + oy * oy > m_robotRadius * m_robotRadius)
    {
        return false;
    }
    return isPointInsideRobotShape(ox, oy);
}

void DiffDriveCollisionGridBased::updateTPObstacle(
    double ox, double oy, std::vector<double>& tp_obstacles) const
{
    ASSERTMSG_(!m_trajectory.empty(), "PTG has not been initialized!");
    const TCollisionCell& cell = m_collisionGrid.getTPObstacle(ox, oy);
    if (cell.empty()) { return; }

    if (isObstacleInsideShape(ox, oy))
    {
        // Rare case: apply the collision-behavior policy per entry.
        for (const auto& i : cell)
        {
            internal_TPObsDistancePostprocess(
                ox, oy, i.second, tp_obstacles[i.first]);
        }
        return;
    }
    // Keep the minimum distance:
    for (const auto& i : cell)
    {
        mrpt::keep_min(tp_obstacles[i.first], static_cast<double>(i.second));
    }
}

void DiffDriveCollisionGridBased::updateTPObstacleSingle(
    double ox, double oy, uint16_t k, double& tp_obstacle_k) const
{
    ASSERTMSG_(!m_trajectory.empty(), "PTG has not been initialized!");
    const TCollisionCell& cell = m_collisionGrid.getTPObstacle(ox, oy);
    // Keep the minimum distance:
    for (const auto& i : cell)
    {
        if (i.first != k) { continue; }
        if (isObstacleInsideShape(ox, oy))
        {
            internal_TPObsDistancePostprocess(ox, oy, i.second, tp_obstacle_k);
        }
        else { mrpt::keep_min(tp_obstacle_k, static_cast<double>(i.second)); }
    }
}

void DiffDriveCollisionGridBased::internal_readFromStream(
    mrpt::serialization::CArchive& in)
{
    CParameterizedTrajectoryGenerator::internal_readFromStream(in);
    CPTG_RobotShape_Polygonal::internal_shape_loadFromStream(in);

    uint8_t version;
    in >> version;
    switch (version)
    {
        case 0:
        case 1:
            internal_deinitialize();
            in >> V_MAX >> W_MAX >> turningRadiusReference >> m_robotShape >>
                m_resolution >> m_trajectory;
            m_clearance = 0;
            if (version >= 1) { in >> m_clearance; }
            break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    };
}

void DiffDriveCollisionGridBased::internal_writeToStream(
    mrpt::serialization::CArchive& out) const
{
    CParameterizedTrajectoryGenerator::internal_writeToStream(out);
    CPTG_RobotShape_Polygonal::internal_shape_saveToStream(out);

    const uint8_t version = 1;
    out << version;

    out << V_MAX << W_MAX << turningRadiusReference << m_robotShape
        << m_resolution << m_trajectory << m_clearance;
}

mrpt::kinematics::CVehicleVelCmd::Ptr
    DiffDriveCollisionGridBased::getSupportedKinematicVelocityCommand() const
{
    return mrpt::kinematics::CVehicleVelCmd::Ptr(
        new mrpt::kinematics::CVehicleVelCmd_DiffDriven());
}

double DiffDriveCollisionGridBased::getPathStepDuration() const
{
    return m_stepTimeDuration;
}

mrpt::math::TTwist2D DiffDriveCollisionGridBased::getPathTwist(
    uint16_t k, uint32_t step) const
{
    ASSERT_(k < m_trajectory.size());
    ASSERT_(step < m_trajectory[k].size());

    auto tw = mrpt::math::TTwist2D(
        m_trajectory[k][step].v, 0, m_trajectory[k][step].w);
    tw.rotate(m_trajectory[k][step].phi);

    return tw;
}
