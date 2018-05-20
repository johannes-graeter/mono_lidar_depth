#include "mark_depth_outlier.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "mark_depth_outlier_node");

    matches_conversion_ros_tool::MarkDepthOutlier mark_depth_outlier(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
