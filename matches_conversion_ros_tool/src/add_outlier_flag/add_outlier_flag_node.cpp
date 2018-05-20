#include "add_outlier_flag.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "add_outlier_flag_node");

    matches_conversion_ros_tool::AddOutlierFlag add_outlier_flag(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
