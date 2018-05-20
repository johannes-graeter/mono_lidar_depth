#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include <matches_msg_depth_ros/MatchesMsgWithOutlierFlag.h>
#include <sensor_msgs/Image.h>

#include <message_filters/sync_policies/exact_time.h>

#include "matches_conversion_ros_tool/SemanticLabelsInterface.h"

namespace matches_conversion_ros_tool {

class SemanticLabels {

    using Interface = SemanticLabelsInterface;
    using ReconfigureConfig = SemanticLabelsConfig;
    using ReconfigureServer = dynamic_reconfigure::Server<ReconfigureConfig>;

    using MatchesMsg = matches_msg_depth_ros::MatchesMsgWithOutlierFlag;

    using SyncPolicy = message_filters::sync_policies::ExactTime<MatchesMsg, sensor_msgs::Image>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;


public:
    SemanticLabels(ros::NodeHandle, ros::NodeHandle);

private: // process method
    ///@brief process the input from ros, execute whatever, publish it
    void callbackSubscriber(const MatchesMsg::ConstPtr&, const sensor_msgs::Image::ConstPtr&);
    void reconfigureRequest(const ReconfigureConfig&, uint32_t);

    std::unique_ptr<Synchronizer> sync_;

    Interface interface_;
    ReconfigureServer reconfigureServer_;

    //    const std::map<int, std::string> label_map_{{7, "street"},
    //                                                {8, "sidewalk"},
    //                                                {11, "building"},
    //                                                {13, "building"},
    //                                                {17, "building"},
    //                                                {20, "street_sign}"},
    //                                                {21, "vegetation"},
    //                                                {22, "vegetation"},
    //                                                {23, "sky"},
    //                                                {25, "vehicle"},
    //                                                {26, "vehicle"}};
};
} // namespace matches_conversion_ros_tool
