
#include "lio/dyn_filter.h"

#include <ros/ros.h>

#include "basic/logs.h"

// M-detector. Deliberately included only here: its headers define
// global-namespace symbols (PointType, V3D, ...) that would clash with
// BASIC::PointType in the rest of super_lio.
#include <m-detector/DynObjFilter.h>

namespace LI2Sup{

struct DynObjFilterWrapper::Impl{
  std::shared_ptr<DynObjFilter> filter;
};


DynObjFilterWrapper::DynObjFilterWrapper() = default;

DynObjFilterWrapper::~DynObjFilterWrapper() = default;


void DynObjFilterWrapper::init(){
  ros::NodeHandle nh;   // root handle: M-detector reads flat "dyn_obj/*" keys

  nh.param<bool>("dyn_obj/en", enabled_, false);
  if(!enabled_){
    gINFO("[DynFilter]: disabled.");
    return;
  }

  impl_.reset(new Impl());
  impl_->filter.reset(new DynObjFilter());
  impl_->filter->init(nh);
  inited_ = true;
  gINFO("[DynFilter]: M-detector enabled.");
}


bool DynObjFilterWrapper::process(BASIC::PointCloudType& cloud_body,
                                  BASIC::VV3& points_body,
                                  const BASIC::SE3& pose_end,
                                  const double scan_end_time,
                                  BASIC::PointCloudType& dyn_cloud_world,
                                  BASIC::PointCloudType& steady_cloud_world)
{
  if(!inited_ || !enabled_){
    return false;
  }

  const std::size_t size = cloud_body.size();
  if(size == 0 || points_body.size() != size){
    return false;
  }

  const M3D rot_end = pose_end.R_.cast<double>();
  const V3D pos_end = pose_end.t_.cast<double>();

  /// The filter needs its own point layout (PointXYZINormal).
  PointCloudXYZI::Ptr feats(new PointCloudXYZI());
  feats->reserve(size);
  for(const auto& pt : cloud_body.points){
    PointType p;   // pcl::PointXYZINormal (M-detector point type)
    p.x = pt.x;
    p.y = pt.y;
    p.z = pt.z;
    p.intensity = pt.intensity;
    p.curvature = 0.0f;
    feats->points.push_back(p);
  }

  impl_->filter->filter(feats, rot_end, pos_end, scan_end_time);

  /// Final per-point verdict: the clustering stage (when enabled) refines the
  /// raw occupancy tags, 1 marks a moving point in both cases.
  const std::vector<int>& final_tag = impl_->filter->cluster_coupled ?
      impl_->filter->dyn_tag_cluster : impl_->filter->dyn_tag_origin;

  dyn_cloud_world.clear();
  steady_cloud_world.clear();
  dyn_cloud_world.reserve(size);
  steady_cloud_world.reserve(size);

  BASIC::PointCloudType cloud_static;
  cloud_static.reserve(size);
  BASIC::VV3 points_static;
  points_static.reserve(size);

  const BASIC::M3& R = pose_end.R_;
  const BASIC::V3& t = pose_end.t_;

  for(std::size_t i = 0; i < size; ++i){
    const auto& pt = cloud_body.points[i];
    const BASIC::V3 pw = R * BASIC::V3(pt.x, pt.y, pt.z) + t;

    BASIC::PointType po_w;
    po_w.x = pw[0];
    po_w.y = pw[1];
    po_w.z = pw[2];
    po_w.intensity = pt.intensity;

    if(final_tag[i] == 1){
      dyn_cloud_world.points.push_back(po_w);
    }else{
      steady_cloud_world.points.push_back(po_w);
      cloud_static.points.push_back(pt);
      points_static.push_back(points_body[i]);
    }
  }

  /// Only static points continue into the map update and the saved map.
  cloud_body.swap(cloud_static);
  points_body.swap(points_static);
  return true;
}

} // namespace END.
