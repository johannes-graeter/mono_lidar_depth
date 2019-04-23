
#include "tracklet_depth.h"

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

TrackletDepth::TrackletDepth(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
        : _image_transport{node_handle} {
    _nodeHandle = {boost::make_shared<ros::NodeHandle>(node_handle)};
    _cloud_last_frame = nullptr;

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

    // Initialize
    this->InitDepthEstimatorPre();
    this->InitSubscriber(node_handle, _params.subscriber_msg_name_semantics != "");
    this->InitPublisher(node_handle);
    this->InitStaticTransforms();

    // ransac plane will be initialializedin depth estimator
    groundPlaneLast_ = nullptr;

    //    // clear debug write
    //    std::stringstream ss;
    //    ss << "/tmp/gp.txt";
    //    std::ofstream file(ss.str().c_str());
    //    file.close();
}

void TrackletDepth::InitSubscriber(ros::NodeHandle& nh, bool use_semantics) {
    if (!use_semantics) {
        ROS_DEBUG_STREAM("TrackletDepth: Subscribe to camera and lidar");
        _subscriber_cloud =
            std::make_unique<SubscriberCloud>(nh, _params.subscriber_msg_name_cloud, _params.msg_queue_size);

        _subscriber_matches =
            std::make_unique<SubscriberTracklets>(nh, _params.subscriber_msg_name_tracklets, _params.msg_queue_size);

        _subscriber_camera_info =
            std::make_unique<SubscriberCameraInfo>(nh, _params.subscriber_msg_name_camera_info, _params.msg_queue_size);

        _sync = std::make_unique<Synchronizer>(
            Policy(_params.msg_queue_size), *_subscriber_cloud, *_subscriber_matches, *_subscriber_camera_info);
        _sync->registerCallback(boost::bind(&TrackletDepth::callbackRansac, this, _1, _2, _3));

    } else {
        ROS_DEBUG_STREAM("TrackletDepth: Subscribe to camera, lidar and semantic image");
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

        _sync2->registerCallback(boost::bind(&TrackletDepth::callbackSemantic, this, _1, _2, _3, _4));
    }
}

std::pair<int, int> TrackletDepth::ExractNewTrackletFrames(const matches_msg_ros::MatchesMsgConstPtr& tracklets_in,
                                                           std::vector<std::shared_ptr<TempTrackletFrame>>& newFrames) {
    int frameCountNew = 0, frameCountOld = 0;

    for (const auto& tracklet : tracklets_in->tracks) {
        int id = tracklet.id;

        // check if tracklet already exists in cache
        bool trackExists = _trackletMap.count(id);

        std::shared_ptr<TempTrackletFrame> tempFrame;

        if (!trackExists) {
            // add the last two features if tracklet is completely new
            const auto& matchNew = tracklet.feature_points.at(0);
            const auto& matchOld = tracklet.feature_points.at(1);

            tempFrame = std::make_unique<TempTrackletFrameExp>(
                id, std::make_pair(matchNew.u, matchNew.v), std::make_pair(matchOld.u, matchOld.v));

            frameCountNew++;
            frameCountOld++;
        } else {
            // add the newest feature if trcaklet already exists
            const auto& matchNew = tracklet.feature_points.at(0);

            tempFrame = std::make_unique<TempTrackletFrame>(id, std::make_pair(matchNew.u, matchNew.v));

            frameCountNew++;
        }

        newFrames.push_back(tempFrame);
    }

    return std::make_pair(frameCountNew, frameCountOld);
}

void TrackletDepth::CalculateFeatureDepthsCurFrame(const Cloud::ConstPtr& cloud_in_cur,
                                                   const std::vector<std::shared_ptr<TempTrackletFrame>>& newFrames,
                                                   const int frameCount,
                                                   Eigen::VectorXd& depths,
                                                   Mono_Lidar::GroundPlane::Ptr& ransacPlane) {
    // Convert the feature points to the interface format for the DepthEstimator
    depths.resize(frameCount);
    Eigen::Matrix2Xd featureCoordinates(2, frameCount);

    int i = 0;
    for (const auto& featureNew : newFrames) {
        // insert features of the current frame
        featureCoordinates(0, i) = featureNew->_feature.first;
        featureCoordinates(1, i) = featureNew->_feature.second;
        i++;
    }

    this->_depthEstimator.CalculateDepth(cloud_in_cur, featureCoordinates, depths, ransacPlane);
}


void TrackletDepth::CalculateFeatureDepthsLastFrame(const Cloud::ConstPtr& cloud_in_last,
                                                    const std::vector<std::shared_ptr<TempTrackletFrame>>& newFrames,
                                                    const int frameCount,
                                                    Eigen::VectorXd& depths,
                                                    Mono_Lidar::GroundPlane::Ptr& ransacPlaneOld) {
    // Convert the feature points to the interface format for the DepthEstimator
    depths.resize(frameCount);

    if (cloud_in_last == nullptr) {
        depths.setConstant(-1);
        return;
    }

    Eigen::Matrix2Xd featureCoordinates(2, frameCount);

    int i = 0;
    for (const auto& featureNew : newFrames) {
        // if the tracklet of the frame has been added this frame, there are exist two new features in successive frames
        const auto featureLast = std::dynamic_pointer_cast<TempTrackletFrameExp>(featureNew);

        if (featureLast != nullptr) {
            // feature from a new tracklet
            // Add feature of the last frame
            featureCoordinates(0, i) = featureLast->_featureLast.first;
            featureCoordinates(1, i) = featureLast->_featureLast.second;
            i++;
        }
    }

    this->_depthEstimator.CalculateDepth(cloud_in_last, featureCoordinates, depths, ransacPlaneOld);
}

std::pair<int, int> TrackletDepth::SaveFeatureDepths(const std::vector<std::shared_ptr<TempTrackletFrame>>& newFrames,
                                                     const Eigen::VectorXd& depthsLastFrame,
                                                     const Eigen::VectorXd& depthsCurFrame,
                                                     std::vector<TypeTrackletKey>& updatedIds) {
    int i = 0, j = 0;
    int newCount = 0;
    int oldCount = 0;

    for (const auto featureNew : newFrames) {
        int id = featureNew->_keyFrameId;

        const auto featureLast = std::dynamic_pointer_cast<TempTrackletFrameExp>(featureNew);

        if (featureLast != nullptr) {
            // New tracklet --> Create
            _trackletMap.emplace(id, feature_tracking::Tracklet());
            _trackletMap[id].id_ = id;
            _trackletMap[id].age_ = 0;

            int u = featureLast->_featureLast.first;
            int v = featureLast->_featureLast.second;

            auto match = feature_tracking::Match(u, v);
            // use world point to save feature's depth
            match.x_ = std::make_shared<feature_tracking::WorldPoint>();
            match.x_->data[2] = depthsLastFrame[j];
            _trackletMap[id].push_front(match);

            j++;
            newCount++;
        } else
            oldCount++;

        int u = featureNew->_feature.first;
        int v = featureNew->_feature.second;
        auto match = feature_tracking::Match(u, v);
        // use world point to save feature's depth temporarily
        match.x_ = std::make_shared<feature_tracking::WorldPoint>();
        match.x_->data[2] = depthsCurFrame[i];
        _trackletMap[id].push_front(match);

        i++;

        // mark tracklet to send for this frame
        updatedIds.push_back(id);
    }

    return std::make_pair(oldCount, newCount);
}

void TrackletDepth::callbackRansac(const Cloud::ConstPtr& cloud_in,
                                   const matches_msg_ros::MatchesMsgConstPtr& tracklets_in,
                                   const CameraInfo::ConstPtr& camInfo) {
    // Init plane ransac
    Mono_Lidar::GroundPlane::Ptr gp = std::make_shared<Mono_Lidar::RansacPlane>(
        std::make_shared<Mono_Lidar::DepthEstimatorParameters>(depth_estimator_parameters_));

    // Do estimation.
    process(cloud_in, tracklets_in, camInfo, gp);
}

void TrackletDepth::callbackSemantic(const Cloud::ConstPtr& cloud_in,
                                     const matches_msg_ros::MatchesMsgConstPtr& tracklets_in,
                                     const CameraInfo::ConstPtr& camInfo,
                                     const sensor_msgs::Image::ConstPtr& img) {

    // Init groundplane.
    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(camInfo);
    Mono_Lidar::SemanticPlane::Camera cam;
    cam.f = model.fx();
    cam.cu = model.cx();
    cam.cv = model.cy();
    cam.transform_cam_lidar = _camLidarTransform;

    cv_bridge::CvImageConstPtr img_ptr = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::MONO8);
    std::set<int> gp_labels{6, 7, 8, 9};

    double plane_inlier_threshold = depth_estimator_parameters_.ransac_plane_refinement_treshold;
    Mono_Lidar::GroundPlane::Ptr gp =
        std::make_shared<Mono_Lidar::SemanticPlane>(img_ptr->image, cam, gp_labels, plane_inlier_threshold);
    ROS_DEBUG_STREAM("TrackletDepth: use semantics for ground segmentation");
    // Do estimation.
    process(cloud_in, tracklets_in, camInfo, gp);
}

void TrackletDepth::process(const Cloud::ConstPtr& cloud_in,
                            const matches_msg_ros::MatchesMsgConstPtr& tracklets_in,
                            const CameraInfo::ConstPtr& camInfo,
                            Mono_Lidar::GroundPlane::Ptr groundPlaneCur) {
    if (cloud_in->points.size() < 100) {
        ROS_WARN_STREAM("In tracklet_depth: received less than 100 points.");
    }

    // Timer
    auto start_time = std::chrono::steady_clock::now();

    std::stringstream stats;

    _msgCount++;

    _timestamps.push_front(tracklets_in->header.stamp);

    // Initialize depthestimator it not done
    if (!_isDepthEstimatorInitialized) {
        this->InitCamera(camInfo);
        this->InitDepthEstimatorPost();
    }

    double sec = tracklets_in->header.stamp.toSec();
    stats << "Received tracklet-lidar pair at: " << sec << std::endl;
    stats << "Total messages recieved: " << _msgCount << std::endl;
    stats << "Tracklet length: " << tracklets_in->tracks.size() << std::endl;

    // Save all new feature points of the incoming tracklets which haven't been estimated in previous frames
    std::vector<std::shared_ptr<TempTrackletFrame>> tempFrames;
    auto frameCount = ExractNewTrackletFrames(tracklets_in, tempFrames);
    int frameCountNew = frameCount.first;
    int frameCountOld = frameCount.second;

    Eigen::VectorXd depthsCurFrame, depthsLastFrame;

    try {
        // Use the DespthEstiamtor to calculate the depth all newly arrived feature points
        CalculateFeatureDepthsLastFrame(
            _cloud_last_frame, tempFrames, frameCountOld, depthsLastFrame, groundPlaneLast_);
    } catch (const Mono_Lidar::GroundPlane::ExceptionPclInvalid& e) {
        ROS_ERROR_STREAM(e.what());
        ROS_WARN_STREAM("TrackletDepth: Old frame continue with invalid depths");

        depthsLastFrame.resize(frameCountOld);
        depthsLastFrame.setConstant(-1);
    }
    try {
        CalculateFeatureDepthsCurFrame(cloud_in, tempFrames, frameCountNew, depthsCurFrame, groundPlaneCur);
        if (groundPlaneCur == nullptr) {
            ROS_ERROR_STREAM("TrackletDepth: Plane not calculated");
        }

        // remember cloud
        _cloud_last_frame = cloud_in;
    } catch (const Mono_Lidar::GroundPlane::ExceptionPclInvalid& e) {
        ROS_ERROR_STREAM(e.what());
        ROS_WARN_STREAM("TrackletDepth: Cur frame continue with invalid depths");

        depthsCurFrame.resize(frameCountNew);
        depthsCurFrame.setConstant(-1);

        // mark plane invalid so it will be recalculated next time.
        groundPlaneCur = nullptr;

        // declare cloud invalid
        _cloud_last_frame = nullptr;
    }

    // remember the ransac plane
    groundPlaneLast_ = groundPlaneCur;

    // Write Results
    std::vector<TypeTrackletKey> updatetIds;
    auto trackletsCount = SaveFeatureDepths(tempFrames, depthsLastFrame, depthsCurFrame, updatetIds);

    stats << "Old tracklets Count: " << trackletsCount.first << std::endl;
    stats << "New tracklets Count: " << trackletsCount.second << std::endl;

    // Convert newly updated tracklets to the msg format and publish
    matches_msg_depth_ros::MatchesMsg msgOut;
    auto matchesSuccess = convert_tracklets_to_matches_msg(tracklets_in, updatetIds, msgOut);
    // msgOut.header.stamp = tracklets_in->header.stamp;
    _publisher_matches.publish(msgOut);

    if(_params.publisher_msg_name_image_projection_cloud!=""){
        PublishImageProjectionCloud(camInfo->width, camInfo->height);
    }
    stats << "Feature Estimation success count: " << matchesSuccess.first << std::endl;
    stats << "Feature Estimation fail count: " << matchesSuccess.second << std::endl;

    // Delete old tracklets (tracklets with no updates in the current frame)
    TidyUpTracklets(updatetIds);
    TidyUpTimeStamps();


    //    if (groundPlaneLast_ != nullptr) {
    //        std::stringstream ss;
    //        ss << "/tmp/gp.txt";
    //        std::ofstream file(ss.str().c_str(), std::ios_base::app);
    //        file.precision(12);
    //        Eigen::Vector4f plane_params = groundPlaneLast_->getModelCoeffs();
    //        file << plane_params[0] << " " << plane_params[1] << " " << plane_params[2] << " " << plane_params[3]
    //             << std::endl;
    //        file.close();
    //    }

    ROS_DEBUG_STREAM("TrackletDepthRosTool: " + stats.str());
    ROS_INFO_STREAM(
        "Duration tracklet_depth_ros_tool="
        << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count()
        << " ms");
}


void TrackletDepth::TidyUpTracklets(const std::vector<TypeTrackletKey>& updatedIds) {
    std::vector<TypeTrackletKey> toDelete;

    for (const auto& keyAvailable : _trackletMap){
        toDelete.push_back(keyAvailable.first);
    }
    
    for (const auto& keyStay : updatedIds) {
        for (int i = 0; i < int(toDelete.size()); i++) {
            if (toDelete.at(i) == keyStay) {
                toDelete.erase(toDelete.begin() + i);
                break;
            }
        }
    }

    // delete old tracklets which haven't been updated this frame
    for (const auto id : toDelete) {
        auto it = _trackletMap.find(id);
        _trackletMap.erase(it);
    }
}

void TrackletDepth::TidyUpTimeStamps() {
    int maxLength = 0;

    for (const auto& tracklet : _trackletMap) {
        if (int(tracklet.second.size()) > maxLength) {
            maxLength = tracklet.second.size();
        }
    }

    while (int(_timestamps.size()) > maxLength) {
        _timestamps.pop_back();
    }
}

std::pair<int, int> TrackletDepth::convert_tracklets_to_matches_msg(
    const matches_msg_ros::MatchesMsgConstPtr& tracklets_in,
    const std::vector<TypeTrackletKey>& trackletIds,
    matches_msg_depth_ros::MatchesMsg& Out) {
    using MsgFeature = matches_msg_depth_ros::FeaturePoint;
    using MsgTracklet = matches_msg_depth_ros::Tracklet;

    Out.tracks.reserve(trackletIds.size());

    int success = 0;
    int failed = 0;

    // Insert tracklets to new message
    for (const auto trackletId : trackletIds) {
        const auto& curTracklet = _trackletMap[trackletId];
        MsgTracklet t;

        t.id = curTracklet.id_;
        t.age = curTracklet.age_;
        t.feature_points.reserve(curTracklet.size());

        for (const auto& cur_match : curTracklet) {
            const auto& cur_point = cur_match.p1_;
            const float depth = cur_match.x_->data[2];
            MsgFeature msg_feature;

            depth >= 0 ? success++ : failed++;

            msg_feature.u = float(cur_point.u_);
            msg_feature.v = float(cur_point.v_);
            msg_feature.d = depth;

            t.feature_points.push_back(std::move(msg_feature));
        }

        assert(t.feature_points.size() == curTracklet.size());
        Out.tracks.push_back(std::move(t));
    }

    assert(Out.tracks.size() == trackletIds.size());

    // Insert timestamps
    for (const auto& stamp : tracklets_in->stamps) {
        Out.stamps.push_back(stamp);
    }

    // Init header of new message
    Out.header = tracklets_in->header;

    return std::make_pair(success, failed);
}


void TrackletDepth::InitPublisher(ros::NodeHandle& nh) {
    // Pusblisher
    std::cout << "Pusblisher name pointcloud camera cs: " << _params.publisher_msg_name_cloud_camera_cs << std::endl;
    if(_params.publisher_msg_name_cloud_camera_cs!=""){
    _publisher_cloud_camera_cs =
        nh.advertise<Cloud>(_params.publisher_msg_name_cloud_camera_cs, _params.msg_queue_size);
    }
    std::cout << "Publisher name pointcloud interpolated: " << _params.publisher_msg_name_cloud_interpolated
              << std::endl;
    if(_params.publisher_msg_name_cloud_interpolated!=""){
    _publisher_cloud_interpolated =
        nh.advertise<Cloud>(_params.publisher_msg_name_cloud_interpolated, _params.msg_queue_size);
    }
    std::cout << "Publisher name image projection cloud: " << _params.publisher_msg_name_image_projection_cloud
              << std::endl;
    if(_params.publisher_msg_name_image_projection_cloud!=""){
    _publisher_image_projection_cloud =
        _image_transport.advertise(_params.publisher_msg_name_image_projection_cloud, _params.msg_queue_size);
    }
    std::cout << "Publisher name calc depth stats: " << _params.publisher_msg_name_depthcalc_stats << std::endl;
    if(_params.publisher_msg_name_depthcalc_stats!=""){
    _publisher_depthcalc_stats =
        nh.advertise<std_msgs::String>(_params.publisher_msg_name_depthcalc_stats, _params.msg_queue_size);
    }
    
    std::cout << "Publisher name tracklets depth with depth: " << _params.publisher_msg_name_tracklets_depth
              << std::endl;
    _publisher_matches = nh.advertise<matches_msg_depth_ros::MatchesMsg>(_params.publisher_msg_name_tracklets_depth,
                                                                         _params.msg_queue_size);
}

bool TrackletDepth::InitDepthEstimatorPre() {
    std::cout << "Initialize DepthEstimator Pre" << std::endl;

    if (_path_config_depthEstimator == "") {
        std::cout << "No config file for DepthEstimator set. Loading from parameter server." << std::endl;

        if (!_depthEstimator.InitConfig())
            throw "Error in 'initConfig' of DepthEstimator.";
    } else {
        std::cout << "Config file DepthEstimator: " << _path_config_depthEstimator << std::endl;
    }

    if (!_depthEstimator.InitConfig(_path_config_depthEstimator))
        throw "Error in 'initConfig' of DepthEstimator.";

    return true;
}

bool TrackletDepth::InitDepthEstimatorPost() {
    std::cout << "Initialize DepthEstimator Post" << std::endl;

    if (!_depthEstimator.Initialize(_camera, _camLidarTransform))
        throw "Error in 'Initialize' of DepthEstimator.";

    std::cout << "DepthEstimator successful initialized" << std::endl;

    _isDepthEstimatorInitialized = true;

    return true;
}


bool TrackletDepth::InitStaticTransforms() {
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

    _camLidarTransform = transformEigen;

    ROS_INFO_STREAM("Got the tf transformations: " << endl << _camLidarTransform.matrix());

    return true;
}


void TrackletDepth::InitCamera(const CameraInfo::ConstPtr& cam_info) {
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

void TrackletDepth::PublishPointCloudCameraCs() {
    std::cout << "Publich pointcloud camera cs" << std::endl;
    Cloud::Ptr cloud_camera_cs = {boost::make_shared<Cloud>()};

    _depthEstimator.getCloudCameraCs(cloud_camera_cs);
    cloud_camera_cs->header.frame_id = _params.tf_frame_name_cameraLeft;
    cloud_camera_cs->header.stamp = ros::Time::now().toNSec();
    _publisher_cloud_camera_cs.publish(cloud_camera_cs);
}

void TrackletDepth::PublishPointCloudInterpolated() {
    std::cout << "Publish pointcloud interpolated" << std::endl;
    Cloud::Ptr cloud_interpolated = {boost::make_shared<Cloud>()};

    _depthEstimator.getCloudInterpolated(cloud_interpolated);
    cloud_interpolated->header.frame_id = _params.tf_frame_name_cameraLeft;
    cloud_interpolated->header.stamp = ros::Time::now().toNSec();
    _publisher_cloud_interpolated.publish(cloud_interpolated);
}


void TrackletDepth::PublishImageProjectionCloud(int imgWidth, int imgHeight) {
    std::cout << "Publish image projection cloud" << std::endl;

    Eigen::Matrix2Xd points;
    _depthEstimator.getPointsCloudImageCs(points);

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


void TrackletDepth::PublishDepthCalcStats() {
    // get statistics
    auto stats = this->_depthEstimator.getDepthCalcStats();
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
