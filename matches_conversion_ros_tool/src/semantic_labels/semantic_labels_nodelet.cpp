#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "semantic_labels.hpp"

namespace matches_conversion_ros_tool {

class SemanticLabelsNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<SemanticLabels>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<SemanticLabels> impl_;
};
} // namespace matches_conversion_ros_tool

PLUGINLIB_EXPORT_CLASS(matches_conversion_ros_tool::SemanticLabelsNodelet, nodelet::Nodelet);
