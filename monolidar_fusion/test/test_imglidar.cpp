// google test docs
// wiki page: https://code.google.com/p/googletest/w/list
// primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
// FAQ: https://code.google.com/p/googletest/wiki/FAQ
// advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
// samples: https://code.google.com/p/googletest/wiki/V1_7_Samples
//
// List of some basic tests fuctions:
// Fatal assertion                      Nonfatal assertion                   Verifies / Description
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// ASSERT_EQ(expected, actual);         EXPECT_EQ(expected, actual);         expected == actual
// ASSERT_NE(val1, val2);               EXPECT_NE(val1, val2);               val1 != val2
// ASSERT_LT(val1, val2);               EXPECT_LT(val1, val2);               val1 < val2
// ASSERT_LE(val1, val2);               EXPECT_LE(val1, val2);               val1 <= val2
// ASSERT_GT(val1, val2);               EXPECT_GT(val1, val2);               val1 > val2
// ASSERT_GE(val1, val2);               EXPECT_GE(val1, val2);               val1 >= val2
//
// ASSERT_FLOAT_EQ(expected, actual);   EXPECT_FLOAT_EQ(expected, actual);   the two float values are almost equal (4
// ULPs)
// ASSERT_DOUBLE_EQ(expected, actual);  EXPECT_DOUBLE_EQ(expected, actual);  the two double values are almost equal (4
// ULPs)
// ASSERT_NEAR(val1, val2, abs_error);  EXPECT_NEAR(val1, val2, abs_error);  the difference between val1 and val2
// doesn't exceed the given absolute error
//
// Note: more information about ULPs can be found here:
// http://www.cygnus-software.com/papers/comparingfloats/comparingfloats.htm
//
// Example of two unit test:
// TEST(Math, Add) {
//    ASSERT_EQ(10, 5+ 5);
//}
//
// TEST(Math, Float) {
//	  ASSERT_FLOAT_EQ((10.0f + 2.0f) * 3.0f, 10.0f * 3.0f + 2.0f * 3.0f)
//}
//=======================================================================================================================================================
#include "gtest/gtest.h"

#include <Eigen/Eigen>

#include <memory>
#include <vector>

// test classes
#include "HelperLidarRowSegmentation.h"
#include "HistogramPointDepth.h"
#include "NeighborFinderPixel.h"
#include "PointcloudData.h"

#include "camera_pinhole.h"


// A google test function (uncomment the next function, add code and
// change the names TestGroupName and TestName)
// TEST(TestGroupName, TestName) {
// TODO: Add your test code here
//}

namespace {

Eigen::Vector2d project(Eigen::Vector3d p, Eigen::Matrix3d intrinsics) {
    Eigen::Vector3d proj = intrinsics * p;
    proj /= proj[2];

    return Eigen::Vector2d(proj[0], proj[1]);
}

Eigen::Vector3d projectKeepDepth(Eigen::Vector3d p, Eigen::Matrix3d intrinsics) {
    Eigen::Vector3d proj = intrinsics * p;
    proj /= proj[2];

    return Eigen::Vector3d(proj[0], proj[1], p.z());
}
}

TEST(NeigborFinder, findByPixel) {
    const int img_width = 100;
    const int img_height = 100;
    const double cam_p_u = 50;
    const double cam_p_v = 50;
    const double cam_f = 600;
    const int nf_search_witdh = 3;
    const int nf_search_height = 5;

    // init objects
    std::shared_ptr<CameraPinhole> cam;
    cam = std::make_shared<CameraPinhole>(img_width, img_height, cam_f, cam_p_u, cam_p_v);
    Mono_Lidar::NeighborFinderPixel neighborFinder(img_width, img_height, nf_search_witdh, nf_search_height);

    // init points
    const int point_count = 50;

    Eigen::Matrix2Xd points_2d_orig;
    points_2d_orig.resize(2, point_count);
    Eigen::Matrix3Xd points_3d_cam;
    points_3d_cam.resize(3, point_count);

    for (int i = 0; i < point_count; i++) {
        const int u = std::rand() % 10;
        const int v = std::rand() % 10;
        points_2d_orig(0, i) = u;
        points_2d_orig(1, i) = v;
    }

    Eigen::Matrix3Xd support_points;
    support_points.resize(3, point_count);
    Eigen::Matrix3Xd directions;
    directions.resize(3, point_count);

    cam->getViewingRays(points_2d_orig, support_points, directions);

    // init features depth
    Eigen::MatrixXd features_depth;
    features_depth.resize(point_count, 1);

    for (int i = 0; i < point_count; i++) {
        features_depth(i, 0) = std::rand() % 10 + 1;
    }

    points_3d_cam = support_points + features_depth * directions;

    // project points on image plane again
    Eigen::Matrix2Xd points_2d_projected;
    points_2d_projected.resize(2, point_count);

    cam->getImagePoints(points_3d_cam, points_2d_projected);


    // Initialize neighbor finder
    std::vector<int> point_index;

    for (int i = 0; i < point_count; i++) {
        point_index.push_back(i);
    }

    neighborFinder.InitializeLidarProjection(points_2d_orig, points_3d_cam, point_index);

    // get neighbors from tested method
    for (int i = 0; i < point_count; i++) {
        Eigen::Vector2d feature;
        feature.x() = points_2d_orig(0, i);
        feature.x() = points_2d_orig(1, i);

        std::vector<int> index_out;

        neighborFinder.getNeighbors(feature, points_3d_cam, point_index, index_out);

        // test if neighbor pos is in tolerance distance from feature point
        for (const auto& index : index_out) {
            Eigen::Vector2d neighbor_projected;
            neighbor_projected.x() = points_2d_projected(0, index);
            neighbor_projected.y() = points_2d_projected(1, index);

            Eigen::Vector2d neighbor_calc;
            neighbor_calc.x() = points_2d_orig(0, index);
            neighbor_calc.y() = points_2d_orig(1, index);

            auto points_dist = (neighbor_projected - neighbor_calc).norm();
            auto dist_from_feature_u = fabs(neighbor_calc.x() - feature.x());
            auto dist_from_feature_v = fabs(neighbor_calc.y() - feature.y());

            ASSERT_NEAR(points_dist, 0., 0.01);
            ASSERT_LE(dist_from_feature_u, (float)(nf_search_witdh)*0.5f + 0.01);
            ASSERT_LE(dist_from_feature_v, (float)(nf_search_height)*0.5f + 0.01);
        }
    }
}

TEST(LidarSegmenter, test1) {
    // Init camera
    const int img_width = 2000;
    const int img_height = 2000;
    const double cam_p_u = img_width * 0.5;
    const double cam_p_v = img_height * 0.5;
    const double cam_f = 600;

    std::shared_ptr<CameraPinhole> cam;
    cam = std::make_shared<CameraPinhole>(img_width, img_height, cam_f, cam_p_u, cam_p_v);

    // Create point cloud
    Mono_Lidar::PointcloudData points_cloud;

    double points_min_x = -10;
    double points_max_x = 10;
    double points_delta_x = 1;
    double points_min_y = -10;
    double points_max_y = 10;
    double points_delta_y = 1;
    double points_z = 20;

    int points_count =
        (int)(((points_max_x - points_min_x) / points_delta_x) * ((points_max_y - points_min_y) / points_delta_y));

    points_cloud._points_cs_lidar.resize(3, points_count);
    points_cloud._points_cs_camera.resize(3, points_count);
    points_cloud._points_cs_image.resize(2, points_count);
    points_cloud._points_cs_image_visible.resize(2, points_count);
    points_cloud._pointIndex.resize(points_count);
    points_cloud._pointsInImgRange.resize(points_count);
    int i = 0;

    for (double y = points_min_y; y < points_max_y; y += points_delta_y) {
        for (double x = points_max_x; x > points_min_x; x -= points_delta_x) {
            // create 3d points
            points_cloud._points_cs_lidar(0, i) = x;
            points_cloud._points_cs_lidar(1, i) = y;
            points_cloud._points_cs_lidar(2, i) = points_z;

            // misc
            points_cloud._pointIndex.at(i) = i;
            points_cloud._pointsInImgRange[i] = true;

            i++;
        }
    }

    points_cloud._points_cs_camera = points_cloud._points_cs_lidar;

    // project into camera
    points_cloud._points_cs_image.resize(2, points_count);
    cam->getImagePoints(points_cloud._points_cs_camera, points_cloud._points_cs_image);

    points_cloud._points_cs_image_visible = points_cloud._points_cs_image;

    // create row segmentation object
    auto row_segentation = std::make_shared<Mono_Lidar::HelperLidarRowSegmentation>();

    // test row segmenter
    std::vector<int> points_segmented_index = points_cloud._pointIndex;
    Eigen::Vector2d feature_point(cam_p_u, cam_p_v);

    double max_dist_treshold = 0;
    double max_dist_seed_to_seed_start = 5;
    double max_dist_seed_to_seed_gradient = 0;
    double max_dist_neighbor_to_seed_start = 5;
    double max_dist_neighbor_to_seed_gradient = 0;
    double max_dist_neighbor_start = 2;
    double max_dist_eighbor_gradient = 0;
    int maxPointCount = -1;

    row_segentation->SegmentPoints(points_cloud._points_cs_image_visible);

    int result = row_segentation->calculateNeighborPoints(points_cloud,
                                                          feature_point,
                                                          max_dist_treshold,
                                                          max_dist_seed_to_seed_start,
                                                          max_dist_seed_to_seed_gradient,
                                                          max_dist_neighbor_to_seed_start,
                                                          max_dist_neighbor_to_seed_gradient,
                                                          max_dist_neighbor_start,
                                                          max_dist_eighbor_gradient,
                                                          maxPointCount,
                                                          points_segmented_index);

    ASSERT_EQ(result, 1);

    std::cout << "Calculation result code: " << result << std::endl;
    std::cout << "Neighbor points count: " << points_segmented_index.size() << std::endl;

    for (const auto& index : points_segmented_index) {
        Eigen::Vector3d result_point(points_cloud._points_cs_camera(0, index),
                                     points_cloud._points_cs_camera(1, index),
                                     points_cloud._points_cs_camera(2, index));

        std::cout << "Result point: "
                  << "x: " << result_point.x() << ", y: " << result_point.y() << ", z: " << result_point.z()
                  << std::endl;
    }

    ASSERT_EQ(points_segmented_index.size(), 8);
}

TEST(Histogram, GetNearestPoint) {
    // init point-list
    const int points_count = 10;
    std::vector<Eigen::Vector3d> input_points;
    std::vector<int> points_index;
    Eigen::VectorXd input_depths;
    input_depths.resize(points_count);

    float depth = 5;
    for (int i = 0; i < points_count; i++) {
        input_points.push_back(Eigen::Vector3d(0, 0, depth));
        points_index.push_back(i);
        input_depths[i] = depth;
        depth += 0.5f;
    }

    // test method
    std::vector<Eigen::Vector3d> output;
    std::vector<int> output_index;
    Mono_Lidar::PointHistogram::GetNearestPoint(input_points, points_index, input_depths, output, output_index);

    // test results
    ASSERT_EQ(1, output.size());
    ASSERT_EQ(1, output_index.size());
    ASSERT_EQ(output.front(), input_points.front());
    ASSERT_EQ(output_index.front(), points_index.front());
}

// Test the method to find a local maximum using a depth segmenting histogram
TEST(Histogram, FilterPointsMinDistBlob) {
    std::vector<Eigen::Vector3d> input_points;

    // init points
    input_points.push_back(Eigen::Vector3d(0, 0, 2.2));
    input_points.push_back(Eigen::Vector3d(0, 0, 3.5));
    input_points.push_back(Eigen::Vector3d(0, 0, 4.2));
    input_points.push_back(Eigen::Vector3d(0, 0, 5.2));
    input_points.push_back(Eigen::Vector3d(0, 0, 5.2));
    input_points.push_back(Eigen::Vector3d(0, 0, 6.2));
    input_points.push_back(Eigen::Vector3d(0, 0, 7.2));
    input_points.push_back(Eigen::Vector3d(0, 0, 8.2));
    input_points.push_back(Eigen::Vector3d(0, 0, 8.3));
    input_points.push_back(Eigen::Vector3d(0, 0, 8.4));
    input_points.push_back(Eigen::Vector3d(0, 0, 9.2));
    input_points.push_back(Eigen::Vector3d(0, 0, 10.2));
    input_points.push_back(Eigen::Vector3d(0, 0, 10.5));
    int points_count = input_points.size();

    std::vector<int> points_index;
    Eigen::VectorXd input_depths;
    input_depths.resize(points_count);

    for (int i = 0; i < points_count; i++) {
        input_depths[i] = input_points.at(i).z();
        points_index.push_back(i);
    }

    // histogram values
    const double bin_width = 1;
    const int minimum_max_size = 3;
    std::vector<Eigen::Vector3d> output;
    std::vector<int> output_index;
    double higher_border;
    double lower_border;

    auto result = Mono_Lidar::PointHistogram::FilterPointsMinDistBlob(input_points,
                                                                      points_index,
                                                                      input_depths,
                                                                      bin_width,
                                                                      minimum_max_size,
                                                                      output,
                                                                      output_index,
                                                                      lower_border,
                                                                      higher_border);

    std::cout << "Lower border: " << lower_border << std::endl;
    std::cout << "Higher border: " << higher_border << std::endl;
    std::cout << "Output depths: ";

    for (const auto& p : output) {
        std::cout << p.z() << ", ";
    }

    std::cout << std::endl;

    ASSERT_EQ(result, true);

    // check asserts
    for (const auto& p : output) {
        ASSERT_GE(p.z(), lower_border);
        ASSERT_LE(p.z(), higher_border);
    }

    ASSERT_EQ(output.size(), 3);
    ASSERT_EQ(output.at(0).z(), 8.2);
    ASSERT_EQ(output.at(1).z(), 8.3);
    ASSERT_EQ(output.at(2).z(), 8.4);
}
