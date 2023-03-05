
#include "tracklet_depth_interface.h"

#include <image_geometry/pinhole_camera_model.h>

#include "std_msgs/String.h"

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <chrono>

namespace tracklets_depth_ros_tool {

TrackletDepthInterface::TrackletDepthInterface(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
        : _image_transport{node_handle} {
    _nodeHandle = {boost::make_shared<ros::NodeHandle>(node_handle)};

    // parameters
    std::cout << "Initalize parameters" << std::endl;

    // get paths to config files
    std::string settingsPath;

    if (!private_node_handle.getParam("config_tracklet_depth", settingsPath)) {
        throw std::runtime_error("No configuration specified for: config_tracklet_depth");
    }

    std::cout << "config_tracklets_depth: " << settingsPath << std::endl;

    if (!private_node_handle.getParam("config_depth_estimator", _path_config_depthEstimator)) {
        throw std::runtime_error("No configuration specified for: config_depth_estimator");
    }

    std::cout << "config_tracklets_depth: " << _path_config_depthEstimator << std::endl;

    _params.fromFile(settingsPath);
    //    _params.print();

    depth_estimator_parameters_.fromFile(_path_config_depthEstimator);

    tracklet_depth_module_.reset(new tracklets_depth::TrackletDepthModule(_params, depth_estimator_parameters_));

    // Initialize
    this->InitDepthEstimatorPre();
    this->InitSubscriber(node_handle, _params.subscriber_msg_name_semantics != "");
    this->InitPublisher(node_handle);
    this->InitStaticTransforms();

    //    // clear debug write
    //    std::stringstream ss;
    //    ss << "/tmp/gp.txt";
    //    std::ofstream file(ss.str().c_str());
    //    file.close();
}

void TrackletDepthInterface::InitSubscriber(ros::NodeHandle& nh, bool use_semantics) {
    if (!use_semantics) {
        ROS_DEBUG_STREAM("TrackletDepthInterface: Subscribe to camera and lidar");
        _subscriber_cloud =
            std::make_unique<SubscriberCloud>(nh, _params.subscriber_msg_name_cloud, _params.msg_queue_size);

        _subscriber_matches =
            std::make_unique<SubscriberTracklets>(nh, _params.subscriber_msg_name_tracklets, _params.msg_queue_size);

        _subscriber_camera_info =
            std::make_unique<SubscriberCameraInfo>(nh, _params.subscriber_msg_name_camera_info, _params.msg_queue_size);

        _sync = std::make_unique<Synchronizer>(
            Policy(_params.msg_queue_size), *_subscriber_cloud, *_subscriber_matches, *_subscriber_camera_info);
        _sync->registerCallback(boost::bind(&TrackletDepthInterface::callbackRansac, this, _1, _2, _3));

    } else {
        ROS_DEBUG_STREAM("TrackletDepthInterface: Subscribe to camera, lidar and semantic image");
        _subscriber_cloud =
            std::make_unique<SubscriberCloud>(nh, _params.subscriber_msg_name_cloud, _params.msg_queue_size);

        _subscriber_matches =
            std::make_unique<SubscriberTracklets>(nh, _params.subscriber_msg_name_tracklets, _params.msg_queue_size);

        _subscriber_camera_info =
            std::make_unique<SubscriberCameraInfo>(nh, _params.subscriber_msg_name_camera_info, _params.msg_queue_size);

        _subscriber_semantics =
            std::make_unique<SubscriberSemantics>(nh, _params.subscriber_msg_name_semantics, _params.msg_queue_size);

        _sync2 = std::make_unique<Synchronizer2>(Policy2(_params.msg_queue_size),
                                                 *_subscriber_cloud,
                                                 *_subscriber_matches,
                                                 *_subscriber_camera_info,
                                                 *_subscriber_semantics);

        _sync2->registerCallback(boost::bind(&TrackletDepthInterface::callbackSemantic, this, _1, _2, _3, _4));
    }
}

void TrackletDepthInterface::callbackRansac(const Cloud::ConstPtr& cloud_in,
                                   const matches_msg_ros::MatchesMsgConstPtr& tracklets_in,
                                   const CameraInfo::ConstPtr& camInfo) {

    // Do estimation.
    tracklet_depth_module_->process(cloud_in, tracklets_in, camInfo);
}

void TrackletDepthInterface::callbackSemantic(const Cloud::ConstPtr& cloud_in,
                                     const matches_msg_ros::MatchesMsgConstPtr& tracklets_in,
                                     const CameraInfo::ConstPtr& camInfo,
                                     const sensor_msgs::Image::ConstPtr& img) {

    
    ROS_DEBUG_STREAM("TrackletDepthInterface: use semantics for ground segmentation");
    // Do estimation.
    tracklet_depth_module_->process(cloud_in, tracklets_in, camInfo, img);
}



bool TrackletDepthInterface::InitStaticTransforms() {
    using namespace std;

    tf::TransformListener listener;
    tf::StampedTransform transform;
    Eigen::Affine3d transformEigen;

    bool tf_success = false;

    string targetFrame = _params.tf_frame_name_cameraLeft; // target frame to which the coordinates will be transformed
    string sourceFrame = _params.tf_frame_name_velodyne;   // frame in which the coordinates are currently in

    ROS_INFO_STREAM("Waiting for the ROSBAG to start");

    while (!tf_success) {
        try {
            // ros::spinOnce();

            if (!listener.frameExists(targetFrame)) {
                // ROS_INFO_STREAM("waiting for target frame "<< targetFrame);
                // continue;
            }

            if (!listener.frameExists(sourceFrame)) {
                // ROS_INFO_STREAM("waiting for source frame "<< sourceFrame);
                // continue;
            }
            listener.lookupTransform(targetFrame, sourceFrame, ros::Time(0), transform);
            tf_success = true;
        } catch (tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(0.5).sleep();
        }
    }

    // Apply additional camera transformation (correction)
    tf::transformTFToEigen(transform, transformEigen);

    tracklet_depth_module_->SetCameraLidarTransform(transformEigen);
    // _camLidarTransform = transformEigen;

    ROS_INFO_STREAM("Got the tf transformations: " << endl << _camLidarTransform.matrix());

    return true;
}


void TrackletDepthInterface::InitCamera(const CameraInfo::ConstPtr& cam_info) {
    // Check preconditions
    if (_isCameraInitialized)
        return;

    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(cam_info);
    assert(model.fx() == model.fy()); // we only support undistorted images

    // Extract camera parameters from calibration matrix
    double focalLengthX = model.fx();
    double principlePointX = model.cx();
    double principlePointY = model.cy();

    // Create camera model
    _camera = std::make_shared<CameraPinhole>(
        cam_info->width, cam_info->height, focalLengthX, principlePointX, principlePointY);

    _isCameraInitialized = true;

    // debug
    ROS_INFO_STREAM("Got the camera info:" << std::endl
                                           << "Got the camera info:"
                                           << std::endl
                                           << "img_width: "
                                           << cam_info->width
                                           << std::endl
                                           << "img_height: "
                                           << cam_info->height
                                           << std::endl
                                           << "focal_length_x: "
                                           << focalLengthX
                                           << std::endl
                                           << "principalPointX: "
                                           << principlePointX
                                           << std::endl
                                           << "principalPointY: "
                                           << principlePointY
                                           << std::endl);
}


void TrackletDepthInterface::InitPublisher(ros::NodeHandle& nh) {
  // Pusblisher
  std::cout << "Pusblisher name pointcloud camera cs: "
            << _params.publisher_msg_name_cloud_camera_cs << std::endl;
  if (_params.publisher_msg_name_cloud_camera_cs != "") {
    _publisher_cloud_camera_cs = nh.advertise<Cloud>(
        _params.publisher_msg_name_cloud_camera_cs, _params.msg_queue_size);
  }
  std::cout << "Publisher name pointcloud interpolated: "
            << _params.publisher_msg_name_cloud_interpolated << std::endl;
  if (_params.publisher_msg_name_cloud_interpolated != "") {
    _publisher_cloud_interpolated = nh.advertise<Cloud>(
        _params.publisher_msg_name_cloud_interpolated, _params.msg_queue_size);
  }
  std::cout << "Publisher name image projection cloud: "
            << _params.publisher_msg_name_image_projection_cloud << std::endl;
  if (_params.publisher_msg_name_image_projection_cloud != "") {
    _publisher_image_projection_cloud = _image_transport.advertise(
        _params.publisher_msg_name_image_projection_cloud,
        _params.msg_queue_size);
  }
  std::cout << "Publisher name calc depth stats: "
            << _params.publisher_msg_name_depthcalc_stats << std::endl;
  if (_params.publisher_msg_name_depthcalc_stats != "") {
    _publisher_depthcalc_stats = nh.advertise<std_msgs::String>(
        _params.publisher_msg_name_depthcalc_stats, _params.msg_queue_size);
  }

  std::cout << "Publisher name tracklets depth with depth: "
            << _params.publisher_msg_name_tracklets_depth << std::endl;
  _publisher_matches = nh.advertise<matches_msg_depth_ros::MatchesMsg>(
      _params.publisher_msg_name_tracklets_depth, _params.msg_queue_size);
}

void TrackletDepthInterface::PublishPointCloudCameraCs() {
    std::cout << "Publich pointcloud camera cs" << std::endl;
    Cloud::Ptr cloud_camera_cs = {boost::make_shared<Cloud>()};

    // _depthEstimator.getCloudCameraCs(cloud_camera_cs);
    tracklet_depth_module_->getCloudCameraCs(cloud_camera_cs);
    cloud_camera_cs->header.frame_id = _params.tf_frame_name_cameraLeft;
    cloud_camera_cs->header.stamp = ros::Time::now().toNSec();
    _publisher_cloud_camera_cs.publish(cloud_camera_cs);
}

void TrackletDepthInterface::PublishPointCloudInterpolated() {
    std::cout << "Publish pointcloud interpolated" << std::endl;
    Cloud::Ptr cloud_interpolated = {boost::make_shared<Cloud>()};

    // _depthEstimator.getCloudInterpolated(cloud_interpolated);
    tracklet_depth_module_->getCloudInterpolated(cloud_interpolated);
    cloud_interpolated->header.frame_id = _params.tf_frame_name_cameraLeft;
    cloud_interpolated->header.stamp = ros::Time::now().toNSec();
    _publisher_cloud_interpolated.publish(cloud_interpolated);
}


void TrackletDepthInterface::PublishImageProjectionCloud(int imgWidth, int imgHeight) {
    std::cout << "Publish image projection cloud" << std::endl;

    Eigen::Matrix2Xd points;
    // _depthEstimator.getPointsCloudImageCs(points);
tracklet_depth_module_->getPointsCloudImageCs(points);
    cv::Mat img = cv::Mat(imgHeight, imgWidth, CV_8UC1);
    // Init image with white color
    img.setTo(255);

    for (int i = 0; i < points.cols(); i++) {
        uchar grayValue = (uchar)(0);
        int x = (int)points(0, i);
        int y = (int)points(1, i);
        img.at<uchar>(y, x) = grayValue;
    }

    cv::cvtColor(img, img, CV_GRAY2RGB);

    // publish image
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    _publisher_image_projection_cloud.publish(msg);
}


void TrackletDepthInterface::PublishDepthCalcStats() {
    // get statistics
    auto stats = tracklet_depth_module_->getDepthCalcStats();
    int pointCount = stats.getPointCount();
    int pointsSuccess = stats.getSuccess();
    int radiusSearchInsufficientPoints = stats.getRadiusSearchInsufficientPoints();
    int histogramNoLocalMax = stats.getHistogramNoLocalMax();
    int tresholdDepthGlobalGreaterMax = stats.getTresholdDepthGlobalGreaterMax();
    int tresholdDepthGlobalSmallerMin = stats.getTresholdDepthGlobalSmallerMin();
    int tresholdDepthLocalGreaterMax = stats.getTresholdDepthLocalGreaterMax();
    int tresholdDepthLocalSmallerMin = stats.getTresholdDepthLocalSmallerMin();
    //    int insufficientRoadPoints = stats.getInsufficientRoadPoints();
    int pcaIsCubic = stats.getPCAIsCubic();
    int pcaIsLine = stats.getPCAIsLine();
    int pcaIsPoint = stats.getPCAIsPoint();
    int triangleNotPlanar = stats.getTriangleNotPlanar();
    int pointsBehindCamera = stats.getCornerBehindCamera();
    int notPlanarInsufficientPoints = stats.getTriangleNotPlanarInsufficientPoints();
    int viewrayPlaneNotOrthogonal = stats.getPlaneViewrayNotOrthogonal();

    // publish statistics as a string
    std_msgs::String msg;
    std::stringstream ss;

    using namespace std;
    ss << "Depth calculation point statistics:" << endl;
    ss << "Points Count: " << pointCount << endl;
    ss << "Depth Calc Success count: " << pointsSuccess << endl;
    ss << "Radius Search Insufficient points: " << radiusSearchInsufficientPoints << endl;
    ss << "Histogram no Local max: " << histogramNoLocalMax << endl;
    ss << "Global Treshold Depth Greater Max: " << tresholdDepthGlobalGreaterMax << endl;
    ss << "Global Treshold Depth Smaller min: " << tresholdDepthGlobalSmallerMin << endl;
    ss << "Local" << tresholdDepthLocalGreaterMax << endl;
    ss << "Local Treshold Depth Smaller min: " << tresholdDepthLocalSmallerMin << endl;
    ss << "Triangle not planar: " << triangleNotPlanar << endl;
    ss << "Triangle not planar insuficien points: " << notPlanarInsufficientPoints << endl;
    ss << "PCA is cubic: " << pcaIsCubic << endl;
    ss << "PCA is line: " << pcaIsLine << endl;
    ss << "PCA is point: " << pcaIsPoint << endl;
    ss << "ViewRay plane not orthogonal: " << viewrayPlaneNotOrthogonal << endl;
    ss << "Points interpolated behind camera: " << pointsBehindCamera << endl;

    msg.data = ss.str();
    _publisher_depthcalc_stats.publish(msg);
}
}
