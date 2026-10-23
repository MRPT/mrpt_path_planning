/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * Progress reporting while initializing the PTGs of a robot description:
 *  - The callback is called once per PTG, in order, plus a final call with
 *    index == count.
 *  - `fromCache` is false while the collision grids are built, and true when
 *    initializing again from the same configuration.
 *  - No calls if PTGs are not initialized, and no callback is also valid.
 */

#include <gtest/gtest.h>
#include <mpp/data/TrajectoriesAndRobotShape.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

namespace
{
const char* kCfg = R"cfg(
[SelfDriving]
PTG_COUNT = 2

PTG0_Type        = mpp::ptg::DiffDrive_C
PTG0_resolution  = 0.10
PTG0_refDistance = 2.0
PTG0_num_paths   = 11
PTG0_v_max_mps   = 1.0
PTG0_w_max_dps   = 60.0
PTG0_K           = +1.0

PTG1_Type        = mpp::ptg::DiffDrive_C
PTG1_resolution  = 0.10
PTG1_refDistance = 2.0
PTG1_num_paths   = 11
PTG1_v_max_mps   = 1.0
PTG1_w_max_dps   = 60.0
PTG1_K           = -1.0

RobotModel_shape2D_xs = -0.20 0.30 0.30 -0.20
RobotModel_shape2D_ys = -0.18 -0.18 0.18 0.18
)cfg";

struct Call
{
    size_t      index;
    size_t      count;
    std::string description;
    bool        fromCache;
};

// Uses a private, initially empty cache directory.
class PtgInitProgress : public ::testing::Test
{
   protected:
    void SetUp() override
    {
        dir_ = std::filesystem::temp_directory_path() /
               ("mpp_ptg_init_progress_" + std::to_string(::getpid()));
        std::filesystem::remove_all(dir_);
        std::filesystem::create_directories(dir_);
        ASSERT_EQ(::setenv("XDG_CACHE_HOME", dir_.c_str(), 1), 0);
    }
    void TearDown() override
    {
        ::unsetenv("XDG_CACHE_HOME");
        std::error_code ec;
        std::filesystem::remove_all(dir_, ec);
    }

    std::vector<Call> init(bool initializePTGs = true)
    {
        std::vector<Call>               calls;
        mrpt::config::CConfigFileMemory cfg(kCfg);
        mpp::TrajectoriesAndRobotShape  trs;
        trs.initFromConfigFile(
            cfg, "SelfDriving", initializePTGs,
            [&](size_t i, size_t n, const std::string& d, bool c) {
                calls.push_back({i, n, d, c});
            });
        return calls;
    }

    std::filesystem::path dir_;
};
}  // namespace

TEST_F(PtgInitProgress, CallbackOrderAndCount)
{
    const auto calls = init();
    ASSERT_EQ(calls.size(), 3u);
    for (size_t i = 0; i < calls.size(); i++)
    {
        EXPECT_EQ(calls[i].index, i);
        EXPECT_EQ(calls[i].count, 2u);
    }
    EXPECT_FALSE(calls[0].description.empty());
    EXPECT_FALSE(calls[1].description.empty());
}

TEST_F(PtgInitProgress, FromCacheOnSecondInit)
{
    const auto first = init();
    ASSERT_EQ(first.size(), 3u);
    EXPECT_FALSE(first[0].fromCache);
    EXPECT_FALSE(first[1].fromCache);

    const auto second = init();
    ASSERT_EQ(second.size(), 3u);
    EXPECT_TRUE(second[0].fromCache);
    EXPECT_TRUE(second[1].fromCache);
}

TEST_F(PtgInitProgress, NoCallsIfPtgsNotInitialized)
{
    EXPECT_TRUE(init(false).empty());
}

TEST_F(PtgInitProgress, EmptyCallbackIsValid)
{
    mrpt::config::CConfigFileMemory cfg(kCfg);
    mpp::TrajectoriesAndRobotShape  trs;
    EXPECT_NO_THROW(trs.initFromConfigFile(cfg, "SelfDriving"));
    EXPECT_TRUE(trs.initialized());
}
