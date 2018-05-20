/*
 * imglidarnode.cpp
 *
 *  Created on: Dec 5, 2016
 *      Author: wilczynski
 */

#include "tracklet_depth.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "tracklet_depth_node");
    std::cout << "Starting node: TrackletDepth Estimator " << std::endl;

    try {
        tracklets_depth_ros_tool::TrackletDepth trackletDepth(ros::NodeHandle(), ros::NodeHandle("~"));
        ros::spin();

        return EXIT_SUCCESS;
    } catch (std::runtime_error& e) {
        std::cerr << std::string("Failed to initialize tracklets_depth_ros_tool: ") << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    } catch (char const* e) {
        std::cerr << std::string("Failed to initialize tracklets_depth_ros_tool: ") << e << std::endl;
        std::exit(EXIT_FAILURE);
    } catch (std::string& str) {
        std::cerr << "ERROR" << str << std::endl;
        std::exit(EXIT_FAILURE);
    }
}
