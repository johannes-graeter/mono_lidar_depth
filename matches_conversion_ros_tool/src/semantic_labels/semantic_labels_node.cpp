#include "semantic_labels.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "semantic_labels_node");

    matches_conversion_ros_tool::SemanticLabels semantic_labels(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
