#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "add_outlier_flag.hpp"

namespace matches_conversion_ros_tool {

class AddOutlierFlagNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<AddOutlierFlag>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<AddOutlierFlag> impl_;
};
} // namespace matches_conversion_ros_tool

PLUGINLIB_EXPORT_CLASS(matches_conversion_ros_tool::AddOutlierFlagNodelet, nodelet::Nodelet);
