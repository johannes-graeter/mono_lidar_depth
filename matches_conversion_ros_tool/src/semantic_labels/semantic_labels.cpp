#include "semantic_labels.hpp"

#include <chrono>
#include <unordered_map>
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
std::unordered_map<int, int> calcLabelOccurence(const cv::Mat& roi) {
    std::unordered_map<int, int> out;
    for (int i = 0; i < roi.rows; ++i) {
        for (int j = 0; j < roi.cols; ++j) {
            int label = static_cast<int>(roi.at<uchar>(i, j));
            out[label] = out[label] + 1;
        }
    }

    return out;
}

void assignLabels(const cv::Mat& m, cv::Size roi_size, matches_msg_depth_ros::MatchesMsgWithOutlierFlag& matches_msg) {
    // assign label with max occurence
    for (auto& track : matches_msg.tracks) {
        // Get occurences aroudn tracklet point
        cv::Point2i p(track.feature_points[0].u, track.feature_points[0].v);

        int min_u = std::max(0, p.x - roi_size.width / 2);
        int max_u = std::min(m.cols, p.x + roi_size.width / 2);

        int min_v = std::max(0, p.y - roi_size.height / 2);
        int max_v = std::min(m.rows, p.y + roi_size.height / 2);

        auto occ = calcLabelOccurence(m(cv::Range(min_v, max_v), cv::Range(min_u, max_u)));

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
    auto start_semantic = std::chrono::steady_clock::now();

    // copy msg
    MatchesMsg out_msg = *input1;

    // get input
    cv_bridge::CvImageConstPtr input_image;
    try {
        input_image = cv_bridge::toCvShare(input2, sensor_msgs::image_encodings::MONO8);
        if (input_image->encoding != "mono8") {
            throw std::runtime_error("In SemanticLabels: wrong encoding of semantic image. Must be mono8.");
        }
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

    ROS_INFO_STREAM("Duration assign semantic labels=" << std::chrono::duration_cast<std::chrono::milliseconds>(
                                                              std::chrono::steady_clock::now() - start_semantic)
                                                              .count()
                                                       << " ms");
}

/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure
 * window
*/
void SemanticLabels::reconfigureRequest(const ReconfigureConfig& config, uint32_t level) {
    interface_.fromConfig(config);
}


} // namespace matches_conversion_ros_tool
