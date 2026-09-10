

#ifndef DYN_FILTER_H_
#define DYN_FILTER_H_

#include <memory>

#include "basic/alias.h"
#include "basic/Manifold.h"

namespace LI2Sup{

/// @brief Embeds M-detector's moving-object filter into the SuperLIO pipeline.
///
/// The M-detector headers declare symbols in the global namespace (PointType,
/// V3D, ...), so they are included only inside the implementation file. The
/// undistorted scan is filtered in place: moving points are removed and
/// reported separately, so that only static points reach the map update.
class DynObjFilterWrapper{
public:
  using Ptr = std::shared_ptr<DynObjFilterWrapper>;

  DynObjFilterWrapper();
  ~DynObjFilterWrapper();
  DynObjFilterWrapper(const DynObjFilterWrapper&) = delete;
  DynObjFilterWrapper& operator=(const DynObjFilterWrapper&) = delete;

  /// Reads "dyn_obj/*" parameters (enable flag and M-detector settings) from
  /// the global parameter server. Must be called once before process().
  void init();

  bool enabled() const { return enabled_; }

  /// @param cloud_body          undistorted, downsampled scan (body frame). Modified
  ///                            in place: on return it holds only static points.
  /// @param points_body         body-frame mirror of cloud_body (as consumed by the
  ///                            map update). Compacted with the same mask.
  /// @param pose_end            refined world <- body pose at scan end.
  /// @param scan_end_time       scan end time stamp [s].
  /// @param dyn_cloud_world     out: points classified as moving (world frame).
  /// @param steady_cloud_world  out: points kept for mapping (world frame).
  /// @return false when the filter is disabled or the input is empty; the
  ///         clouds are then left untouched and dyn/steady outputs are empty.
  bool process(BASIC::PointCloudType& cloud_body,
               BASIC::VV3& points_body,
               const BASIC::SE3& pose_end,
               const double scan_end_time,
               BASIC::PointCloudType& dyn_cloud_world,
               BASIC::PointCloudType& steady_cloud_world);

private:
  struct Impl;  // hides the M-detector types
  std::unique_ptr<Impl> impl_;
  bool enabled_ = false;
  bool inited_  = false;
};

} // namespace END.

#endif
