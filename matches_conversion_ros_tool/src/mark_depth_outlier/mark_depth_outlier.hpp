#pragma once

#include <dynamic_reconfigure/server.h>

#include <matches_msg_depth_ros/MatchesMsg.h>
#include <matches_msg_depth_ros/MatchesMsgWithOutlierFlag.h>
#include <matches_msg_ros/MatchesMsgWithOutlierFlag.h>

#include <message_filters/subscriber.h>
#include <ros/ros.h>
#include <message_filters/sync_policies/exact_time.h>

#include "matches_conversion_ros_tool/MarkDepthOutlierInterface.h"

namespace matches_conversion_ros_tool {

class MarkDepthOutlier {

    using Interface = MarkDepthOutlierInterface;
    using ReconfigureConfig = MarkDepthOutlierConfig;
    using ReconfigureServer = dynamic_reconfigure::Server<ReconfigureConfig>;

    using InputDepth = matches_msg_depth_ros::MatchesMsg;
    using InputOutliers = matches_msg_ros::MatchesMsgWithOutlierFlag;
    using OutputOutliers = matches_msg_depth_ros::MatchesMsgWithOutlierFlag;

    using SyncPolicy = message_filters::sync_policies::ExactTime<InputDepth, InputOutliers>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

public:
    MarkDepthOutlier(ros::NodeHandle, ros::NodeHandle);

private:
    ///@brief process the input from ros, execute whatever, publish it
    void callbackSubscriber(const InputDepth::ConstPtr&, const InputOutliers::ConstPtr&);
    void reconfigureRequest(const ReconfigureConfig&, uint32_t);

    ///@brief sync subscribers
    std::unique_ptr<Synchronizer> sync_;

    Interface interface_;
    ReconfigureServer reconfigureServer_;
};
} // namespace matches_conversion_ros_tool
