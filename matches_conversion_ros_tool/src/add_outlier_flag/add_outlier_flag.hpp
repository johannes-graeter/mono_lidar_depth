#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include <matches_msg_depth_ros/MatchesMsg.h>
#include <matches_msg_depth_ros/MatchesMsgWithOutlierFlag.h>

#include "matches_conversion_ros_tool/AddOutlierFlagInterface.h"

namespace matches_conversion_ros_tool {

class AddOutlierFlag {

    using Interface = AddOutlierFlagInterface;
    using ReconfigureConfig = AddOutlierFlagConfig;
    using ReconfigureServer = dynamic_reconfigure::Server<ReconfigureConfig>;

    using InputMsg = matches_msg_depth_ros::MatchesMsg;
    using OutputMsg = matches_msg_depth_ros::MatchesMsgWithOutlierFlag;

public:
    AddOutlierFlag(ros::NodeHandle, ros::NodeHandle);

private:
    void callbackSubscriber(const InputMsg::ConstPtr& msg);
    void reconfigureRequest(const ReconfigureConfig&, uint32_t);

    Interface interface_;
    ReconfigureServer reconfigureServer_;
};
} // namespace matches_conversion_ros_tool
