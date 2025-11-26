/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/interfaces/ObstacleSource.h>
#include <mrpt/version.h>

using namespace mpp;

ObstacleSource::~ObstacleSource() = default;

ObstacleSource::Ptr ObstacleSource::FromStaticPointcloud(
    const mrpt::maps::CPointsMap::Ptr& pc)
{
    return std::make_shared<ObstacleSourceStaticPointcloud>(pc);
}

void ObstacleSourceStaticPointcloud::apply_clipping_box(
    const std::optional<mrpt::math::TBoundingBox>& bbox)
{
    if (!bbox)
    {
        clipped_obs_ = static_obs_;
        return;
    }
    clipped_obs_ = mrpt::maps::CSimplePointsMap::Create();

    const auto bb = mrpt::math::TBoundingBoxf(
        bbox->min.cast<float>(), bbox->max.cast<float>());

    const auto& xs = static_obs_->getPointsBufferRef_x();
    const auto& ys = static_obs_->getPointsBufferRef_y();

#if MRPT_VERSION >= 0x020f00  // 2.15.0
    mrpt::maps::CPointsMap::InsertCtx ctx =
        clipped_obs_->prepareForInsertPointsFrom(*static_obs_);
#endif

    for (size_t i = 0; i < xs.size(); i++)
    {
        if (xs[i] < bb.min.x || xs[i] > bb.max.x ||  //
            ys[i] < bb.min.y || ys[i] > bb.max.y)
            continue;
#if MRPT_VERSION >= 0x020f03  // 2.15.3
        clipped_obs_->insertPointFrom(i, ctx);
#elif MRPT_VERSION >= 0x020f00  // 2.15.0
        clipped_obs_->insertPointFrom(*static_obs_, i, ctx);
#else
        clipped_obs_->insertPointFrom(*static_obs_, i);
#endif
    }
}

mrpt::maps::CPointsMap::Ptr ObstacleSourceGenericSensor::obstacles(
    [[maybe_unused]] mrpt::system::TTimeStamp t)
{
    auto lck = mrpt::lockHelper(obsMtx_);

    if (!raw_obs_) { raw_obs_ = mrpt::maps::CSimplePointsMap::Create(); }
    if (obs_)
    {
        raw_obs_->clear();
        raw_obs_->insertObservation(*obs_, robotPoseForObs_);
    }

    if (!clipping_bbox_)
    {
        clipped_obs_ = raw_obs_;  // shallow_copy (both point to the same data)
    }
    else
    {
        const auto& bbox = clipping_bbox_.value();

        clipped_obs_ = mrpt::maps::CSimplePointsMap::Create();

        const auto bb = mrpt::math::TBoundingBoxf(
            bbox.min.cast<float>(), bbox.max.cast<float>());

        const auto& xs = raw_obs_->getPointsBufferRef_x();
        const auto& ys = raw_obs_->getPointsBufferRef_y();

#if MRPT_VERSION >= 0x020f00  // 2.15.0
        mrpt::maps::CPointsMap::InsertCtx ctx =
            clipped_obs_->prepareForInsertPointsFrom(*raw_obs_);
#endif

        for (size_t i = 0; i < xs.size(); i++)
        {
            if (xs[i] < bb.min.x || xs[i] > bb.max.x ||  //
                ys[i] < bb.min.y || ys[i] > bb.max.y)
            {
                continue;
            }
#if MRPT_VERSION >= 0x020f03  // 2.15.3
            clipped_obs_->insertPointFrom(i, ctx);
#elif MRPT_VERSION >= 0x020f00  // 2.15.0
            clipped_obs_->insertPointFrom(*raw_obs_, i, ctx);
#else
            clipped_obs_->insertPointFrom(*raw_obs_, i);
#endif
        }
    }

    return clipped_obs_;
}

void ObstacleSourceGenericSensor::apply_clipping_box(
    const std::optional<mrpt::math::TBoundingBox>& bbox)
{
    auto lck = mrpt::lockHelper(obsMtx_);

    clipping_bbox_ = bbox;
}
