/*
 * tracklet_depth.h
 *
 *  Created on: Sep 13, 2017
 *      Author: wilczynski
 */

#pragma once

#include <map>

#include <ros/ros.h>

#include <monolidar_fusion/DepthEstimator.h>
#include <monolidar_fusion/camera_pinhole.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <matches_msg_depth_ros/MatchesMsg.h>
#include <matches_msg_ros/MatchesMsg.h>


#include <feature_tracking_core/tracker_libviso.h>

#include <image_transport/subscriber_filter.h>


#include "tracklets_depth/TempTrackletFrame.h"
#include "tracklets_depth/tracklet_depth_module.h"
#include "tracklets_depth/parameters.h"

namespace tracklets_depth_ros_tool {
class TrackletDepthInterface {
    using Point = pcl::PointXYZI;
    using Cloud = pcl::PointCloud<Point>;
    using SubscriberCloud = message_filters::Subscriber<Cloud>;

    using CameraInfo = sensor_msgs::CameraInfo;
    using SubscriberCameraInfo = message_filters::Subscriber<CameraInfo>;

    using Tracklet = matches_msg_ros::MatchesMsg;
    using SubscriberTracklets = message_filters::Subscriber<Tracklet>;
    using SubscriberSemantics = message_filters::Subscriber<sensor_msgs::Image>;

    using Policy = message_filters::sync_policies::ApproximateTime<Cloud, matches_msg_ros::MatchesMsg, CameraInfo>;
    using Synchronizer = message_filters::Synchronizer<Policy>;

    using Policy2 = message_filters::sync_policies::
        ApproximateTime<Cloud, matches_msg_ros::MatchesMsg, CameraInfo, sensor_msgs::Image>;
    using Synchronizer2 = message_filters::Synchronizer<Policy2>;

    using TypeTrackletKey = u_int64_t;
    using StampType = std_msgs::Header::_stamp_type;

public:
    TrackletDepthInterface(ros::NodeHandle, ros::NodeHandle);

private:
    void callbackRansac(const Cloud::ConstPtr& cloud_in,
                        const matches_msg_ros::MatchesMsgConstPtr& tracklets_in,
                        const CameraInfo::ConstPtr& camInfo);

    void callbackSemantic(const Cloud::ConstPtr& cloud_in,
                          const matches_msg_ros::MatchesMsgConstPtr& tracklets_in,
                          const CameraInfo::ConstPtr& camInfo,
                          const sensor_msgs::Image::ConstPtr& img);


    // _____ Initializer methods _____

    void InitSubscriber(ros::NodeHandle& nh, bool use_semantics = false);
    void InitPublisher(ros::NodeHandle& nh);
    bool InitStaticTransforms();

    // ____ Publisher methods _____

    ///@brief Publishes the point cloud in Camera coordinates (useful to check if the transformation between camera and
    /// lidar has been initialized correctly)
    void PublishPointCloudCameraCs();

    ///@brief Publishes depth enchanced image feature points as 3D points in a point cloud
    void PublishPointCloudInterpolated();

    ///@brief Publishes an image with the projected point cloud
    void PublishImageProjectionCloud(int imgWidth, int imgHeight);

    ///@brief Publishes the statistics which show the count of successful calculated depths and the type of outliers
    void PublishDepthCalcStats();


public: // attributes
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    ///@brief init flags
    bool _isCameraInitialized = false;
    bool _isDepthEstimatorInitialized = false;

    ///@brief  Ros
    ros::NodeHandlePtr _nodeHandle;

    ///@brief subscriber
    image_transport::ImageTransport _image_transport;
    std::unique_ptr<SubscriberCloud> _subscriber_cloud;
    std::unique_ptr<SubscriberCameraInfo> _subscriber_camera_info;
    std::unique_ptr<SubscriberTracklets> _subscriber_matches;
    std::unique_ptr<SubscriberSemantics> _subscriber_semantics;
    ///@brief synchronizers, one for smenatic image
    std::unique_ptr<Synchronizer> _sync;
    std::unique_ptr<Synchronizer2> _sync2;

    ///@ brief publisher

    ///@ brief The pointcloud in camera coordinate frame (debug purpose for checking the camera<->lidar transform)
    ros::Publisher _publisher_cloud_camera_cs;

    ///@ brief publisher Cloud consisting of feature 3d points using the estimated depths
    ros::Publisher _publisher_cloud_interpolated;

    ///@ brief publisher 2D Image with the projected original pointcloud (debug purpose for checking the camera<->lidar
    /// transform)
    image_transport::Publisher _publisher_image_projection_cloud;

    ///@brief Publishes the statistics which show the count of successful calculated depths and the type of outliers
    ros::Publisher _publisher_depthcalc_stats;

    ///@brief Original tracklets with added depth information
    ros::Publisher _publisher_matches;

    ///@brief parameters

    ///@brief File Location of the depth estimator config file
    std::string _path_config_depthEstimator;

    ///@brief Parameters
    tracklets_depth::TrackletDepthParameters _params;
    Mono_Lidar::DepthEstimatorParameters depth_estimator_parameters_;

    std::unique_ptr<tracklets_depth::TrackletDepthModule> tracklet_depth_module_; 
};
}
