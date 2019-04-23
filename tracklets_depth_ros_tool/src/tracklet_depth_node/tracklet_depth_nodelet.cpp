#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "tracklet_depth.h"

namespace tracklets_depth_ros_tool {

class TrackletDepthNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<TrackletDepth>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<TrackletDepth> impl_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace tracklets_depth_ros_tool

PLUGINLIB_EXPORT_CLASS(tracklets_depth_ros_tool::TrackletDepthNodelet, nodelet::Nodelet);
