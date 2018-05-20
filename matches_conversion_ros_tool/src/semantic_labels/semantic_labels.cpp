#include "semantic_labels.hpp"

#include "cv.hpp"
#include "cv_bridge/cv_bridge.h"

namespace matches_conversion_ros_tool {

SemanticLabels::SemanticLabels(ros::NodeHandle nh_public, ros::NodeHandle nh_private)
        : interface_{nh_private}, reconfigureServer_{nh_private} {

    /**
     * Initialization
     */
    interface_.fromParamServer();


    /**
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/SemanticLabels.if file.
     * Don't forget to register your callbacks here!
     */
    reconfigureServer_.setCallback(boost::bind(&SemanticLabels::reconfigureRequest, this, _1, _2));

    {
        // setup synchronizer
        sync_ = std::make_unique<Synchronizer>(
            SyncPolicy(100), *(interface_.subscriber_matches), *(interface_.subscriber_labels));
        sync_->registerCallback(boost::bind(&SemanticLabels::callbackSubscriber, this, _1, _2));
    }

    rosinterface_handler::showNodeInfo();
}

namespace {
std::map<int, int> calcLabelOccurence(int min_label, int max_label, const cv::Mat& roi) {
    std::map<int, int> out;
    // Go through labels and count occurences.
    for (int i = min_label; i < max_label; ++i) {
        int occ = cv::countNonZero(roi == i);
        if (occ > 0) {
            out[i] = occ;
        }
    }

    return out;
}

void assignLabels(const cv::Mat& m, cv::Size roi_size, matches_msg_depth_ros::MatchesMsgWithOutlierFlag& matches_msg) {
    // Get Minimum and maximum label.
    double min_val_d, max_val_d;
    cv::minMaxLoc(m, &min_val_d, &max_val_d);
    const int min_val = static_cast<int>(min_val_d);
    const int max_val = static_cast<int>(max_val_d);

    // assign label with max occurence
    for (auto& track : matches_msg.tracks) {
        // Get occurences aroudn tracklet point
        cv::Point2i p(track.feature_points[0].u, track.feature_points[0].v);

        int min_u = std::max(0, p.x - roi_size.width / 2);
        int max_u = std::min(m.cols, p.x + roi_size.width / 2);

        int min_v = std::max(0, p.y - roi_size.height / 2);
        int max_v = std::min(m.rows, p.y + roi_size.height / 2);

        auto occ = calcLabelOccurence(min_val, max_val, m(cv::Range(min_v, max_v), cv::Range(min_u, max_u)));

        // Take maximum occurence
        auto it = std::max_element(
            occ.cbegin(), occ.cend(), [](const auto& a, const auto& b) { return a.second < b.second; });

        // Assign semantic according to map int->string
        int label = it->first;
        track.label = label; // Label is an int, LUT has to be provided by user
    }
}
}
void SemanticLabels::callbackSubscriber(const MatchesMsg::ConstPtr& input1,
                                        const sensor_msgs::Image::ConstPtr& input2) {
    // copy msg
    MatchesMsg out_msg = *input1;

    // get input
    cv_bridge::CvImageConstPtr input_image;
    try {
        input_image = cv_bridge::toCvShare(input2, sensor_msgs::image_encodings::MONO8);
    } catch (const cv_bridge::Exception& e) {
        ROS_WARN_STREAM("In SemanticLabels: " << e.what());
        interface_.matches_publisher.publish(out_msg);
        return;
    }

    // publish incoming message if semantics are invalid
    if (input_image->image.rows > 0 && input_image->image.cols > 0) {
        assignLabels(input_image->image, cv::Size(interface_.roi_width, interface_.roi_height), out_msg);
    } else {
        ROS_WARN_STREAM("In SemanticLabels: Semantic image invalid");
    }

    interface_.matches_publisher.publish(out_msg);
}

/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure
 * window
*/
void SemanticLabels::reconfigureRequest(const ReconfigureConfig& config, uint32_t level) {
    interface_.fromConfig(config);
}


} // namespace matches_conversion_ros_tool
