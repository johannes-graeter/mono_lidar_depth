#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "mark_depth_outlier.hpp"

namespace matches_conversion_ros_tool {

class MarkDepthOutlierNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<MarkDepthOutlier>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<MarkDepthOutlier> impl_;
};
} // namespace matches_conversion_ros_tool

PLUGINLIB_EXPORT_CLASS(matches_conversion_ros_tool::MarkDepthOutlierNodelet, nodelet::Nodelet);
