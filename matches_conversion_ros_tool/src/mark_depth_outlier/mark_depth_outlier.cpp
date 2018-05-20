#include "mark_depth_outlier.hpp"

namespace matches_conversion_ros_tool {

MarkDepthOutlier::MarkDepthOutlier(ros::NodeHandle nh_public, ros::NodeHandle nh_private)
        : interface_{nh_private}, reconfigureServer_{nh_private} {

    /**
     * Initialization
     */
    interface_.fromParamServer();


    /**
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/MarkDepthOutlier.if file.
     * Don't forget to register your callbacks here!
     */
    reconfigureServer_.setCallback(
        boost::bind(&MarkDepthOutlier::reconfigureRequest, this, _1, _2));
    {
        // setup synchronizer
        sync_ = std::make_unique<Synchronizer>(SyncPolicy(100),
                                               *(interface_.subscriber_depth),
                                               *(interface_.subscriber_outliers));
        sync_->registerCallback(boost::bind(&MarkDepthOutlier::callbackSubscriber, this, _1, _2));
    }

    rosinterface_handler::showNodeInfo();
}

void MarkDepthOutlier::callbackSubscriber(const InputDepth::ConstPtr& input1,
                                          const InputOutliers::ConstPtr& input2) {

    OutputOutliers out_msg;
    out_msg.header = input2->header;

    out_msg.stamps = input1->stamps;

    auto input1_iter = input1->tracks.cbegin();
    auto input2_iter = input2->tracks.cbegin();
    if (input1->tracks.size() != input2->tracks.size()) {
        throw std::runtime_error("input1->data.size()=" + std::to_string(input1->tracks.size()) +
                                 " != input2->data.size()=" +
                                 std::to_string(input2->tracks.size()));
    }
    for (; input1_iter != input1->tracks.cend() && input2_iter != input2->tracks.cend();
         ++input1_iter, ++input2_iter) {
        const auto& track_depth = *input1_iter;
        const auto& track_outlier = *input2_iter;

        matches_msg_depth_ros::TrackletWithOutlierFlag cur_track;
        cur_track.feature_points = track_depth.feature_points;
        cur_track.age = track_depth.age;

        cur_track.id = track_outlier.id;
        cur_track.is_outlier = track_outlier.is_outlier;
        cur_track.error = track_outlier.error;
        cur_track.label = track_outlier.label;

        out_msg.tracks.push_back(cur_track);
    }
    ROS_DEBUG_STREAM("MarkDepthOutlier: publish_msg");

    interface_.publisher_depth_outliers.publish(out_msg);
}

/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure
 * window
*/
void MarkDepthOutlier::reconfigureRequest(const ReconfigureConfig& config, uint32_t level) {
    interface_.fromConfig(config);
}


} // namespace matches_conversion_ros_tool
