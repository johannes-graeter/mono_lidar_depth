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


#include "TempTrackletFrame.h"
#include "parameters.h"

namespace tracklets_depth_ros_tool {
class TrackletDepth {
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
    TrackletDepth(ros::NodeHandle, ros::NodeHandle);

private:
    ///@brief process is the main function of the callbacks and will be called with a certain type of groundplane
    /// depending on the node inputs(callbacks)
    void process(const Cloud::ConstPtr& cloud_in,
                 const matches_msg_ros::MatchesMsgConstPtr& tracklets_in,
                 const CameraInfo::ConstPtr& camInfo,
                 Mono_Lidar::GroundPlane::Ptr gp);

    void callbackRansac(const Cloud::ConstPtr& cloud_in,
                        const matches_msg_ros::MatchesMsgConstPtr& tracklets_in,
                        const CameraInfo::ConstPtr& camInfo);

    void callbackSemantic(const Cloud::ConstPtr& cloud_in,
                          const matches_msg_ros::MatchesMsgConstPtr& tracklets_in,
                          const CameraInfo::ConstPtr& camInfo,
                          const sensor_msgs::Image::ConstPtr& img);

    /*
     * Extracts all newly arrived frames with feature points from incoming tracklets
     * The chosen features have not been observed in prior frames
     *
     * @param tracklets_in Incoming tracklets from feature matcher
     * @param newFrames a list of frames containing new feature points
     */
    std::pair<int, int> ExractNewTrackletFrames(const matches_msg_ros::MatchesMsgConstPtr& tracklets_in,
                                                std::vector<std::shared_ptr<TempTrackletFrame>>& newFrames);

    /*
     * Estimates the depth of a list of 2d features
     *
     * @param cloud_in Current pointcloud
     * @param newFrames New Frames containing 2d-Features (contains 1 or 2 feature points)
     * @param featureCount Count of new feature points
     * @param Calcuated feature depths (dimension equals the featureCount-value)
     */
    void CalculateFeatureDepthsLastFrame(const Cloud::ConstPtr& cloud_in_last,
                                         const std::vector<std::shared_ptr<TempTrackletFrame>>& newFrames,
                                         const int frameCount,
                                         Eigen::VectorXd& depths,
                                         Mono_Lidar::GroundPlane::Ptr& ransacPlaneOld);

    void CalculateFeatureDepthsCurFrame(const Cloud::ConstPtr& cloud_in_cur,
                                        const std::vector<std::shared_ptr<TempTrackletFrame>>& lastFrames,
                                        const int featureCount,
                                        Eigen::VectorXd& depths,
                                        Mono_Lidar::GroundPlane::Ptr& ransacPlane);

    /*
     * Converts all tracklets with a given key to the message format
     *
     * @param trackletIds Dictionary keys (Ids) of all Tracklets to convert into the msg format
     * @param matchesMsg Converted Tracklets
     *
     * @return First param: Count of successfully estimated feature depths; Second param: Count of features without
     * estimated depth
     */
    std::pair<int, int> convert_tracklets_to_matches_msg(const matches_msg_ros::MatchesMsgConstPtr& tracklets_in,
                                                         const std::vector<TypeTrackletKey>& trackletIds,
                                                         matches_msg_depth_ros::MatchesMsg& matchesMsg);

    /*
     * Tidy up all old tracklets which have not been updated in the current frame
     *
     * @param updatedIds Ids of the tracklets which have been updated in the current frame
     */
    void TidyUpTracklets(const std::vector<TypeTrackletKey>& updatedIds);

    /*
     * Adjusts the length of the timestamps to the length of the longest tracklet
     */
    void TidyUpTimeStamps();


    /*
     * Integrates the newly arrived and depth enhanced feature points into the Tracklet-Dictionary
     *
     * @param newFrames Allm new frames/features
     * @param depths Calculated depths for the new features
     * @param updatedIds List of tracklet keys which have been updated in the current frame
     */
    std::pair<int, int> SaveFeatureDepths(const std::vector<std::shared_ptr<TempTrackletFrame>>& newFrames,
                                          const Eigen::VectorXd& depthsLastFrame,
                                          const Eigen::VectorXd& depthsNewFrame,
                                          std::vector<TypeTrackletKey>& updatedIds);

    // _____ Initializer methods _____

    void InitSubscriber(ros::NodeHandle& nh, bool use_semantics = false);
    void InitPublisher(ros::NodeHandle& nh);
    bool InitDepthEstimatorPre();
    bool InitDepthEstimatorPost();
    void InitCamera(const CameraInfo::ConstPtr& camInfo);
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

    // ______ Misc _____

    ///@brief init flags
    bool _isCameraInitialized = false;
    bool _isDepthEstimatorInitialized = false;

    ///@brief  Ros
    ros::NodeHandlePtr _nodeHandle;

    ///@brief saved ids in a map
    std::map<TypeTrackletKey, feature_tracking::Tracklet> _trackletMap;

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

    ///@brief Misc
    Cloud::ConstPtr _cloud_last_frame;
    Mono_Lidar::GroundPlane::Ptr groundPlaneLast_;

    std::deque<StampType> _timestamps;
    int _msgCount = 0;

    ///@brief pinhole Camera model
    std::shared_ptr<CameraPinhole> _camera;

    ///@brief Depth Estimator object to estimate the depth of 2d feature points
    Mono_Lidar::DepthEstimator _depthEstimator;

    ///@brief Transformation of the Lidar in the camera coordinate frame
    Eigen::Affine3d _camLidarTransform;

    ///@brief parameters

    ///@brief File Location of the depth estimator config file
    std::string _path_config_depthEstimator;

    ///@brief Parameters
    TrackletDepthParameters _params;
    Mono_Lidar::DepthEstimatorParameters depth_estimator_parameters_;
};
}
