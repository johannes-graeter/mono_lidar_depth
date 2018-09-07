#include "add_outlier_flag.hpp"

#include <matches_msg_conversions_ros/convert.hpp>

namespace matches_conversion_ros_tool {

AddOutlierFlag::AddOutlierFlag(ros::NodeHandle nh_public, ros::NodeHandle nh_private)
        : interface_{nh_private}, reconfigureServer_{nh_private} {

    /**
     * Initialization
     */
    interface_.fromParamServer();


    /**
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/AddOutlierFlag.if file.
     * Don't forget to register your callbacks here!
     */
    reconfigureServer_.setCallback(boost::bind(&AddOutlierFlag::reconfigureRequest, this, _1, _2));

    if (interface_.has_depth) {
        interface_.subscriber_depth->registerCallback(&AddOutlierFlag::callbackSubscriberDepth, this);
    } else {
        interface_.subscriber->registerCallback(&AddOutlierFlag::callbackSubscriber, this);
    }

    rosinterface_handler::showNodeInfo();
}

void AddOutlierFlag::callbackSubscriber(const InputMsg::ConstPtr& msg) {
    InputMsgDepth msg_depth = matches_msg_conversions_ros::ConvertToDepth(msg);
    callbackSubscriberDepth(boost::make_shared<const InputMsgDepth>(msg_depth));
}

void AddOutlierFlag::callbackSubscriberDepth(const InputMsgDepth::ConstPtr& msg) {
    OutputMsg new_msg = matches_msg_conversions_ros::Convert(msg, std::vector<bool>(msg->tracks.size(), false));
    interface_.publisher.publish(new_msg);
}

/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
void AddOutlierFlag::reconfigureRequest(const ReconfigureConfig& config, uint32_t level) {
    interface_.fromConfig(config);
}


} // namespace matches_conversion_ros_tool
