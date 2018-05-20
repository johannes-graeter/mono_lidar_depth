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
#include <matches_msg_depth_ros/MatchesMsg.h>
#include <matches_msg_ros/MatchesMsg.h>
#include <ros/ros.h>
#include "gtest/gtest.h"

#include <cmath>
#include <memory>
#include <vector>

#include <Eigen/Eigen>

// namespace {
//
// void CreateTrackletMsg(matches_msg_ros::MatchesMsg& trackletMsg)
//{
//	trackletMsg.header.frame_id = "test";
//
//	// create timestamps
//	int stamp_count = 10;
//
//	for (const int i = stamp_count - 1; i >= 0; i--)
//	{
//		trackletMsg.stamps.push_back(ros::Time(i));
//	}
//
//	trackletMsg.header.stamp = trackletMsg.stamps.front();
//
//	// insert tracklets
//	int tracklets_count = std::rand()%1000 + 100;
//
//	for (const int i = 0; i < tracklets_count; i++)
//	{
//		// init track
//		matches_msg_ros::Tracklet track;
//		const int tracklet_length_count = std::rand()%10 + 2;
//		track.id = i;
//		track.age = tracklet_length_count - 1;
//		track.feature_points.reserve(tracklet_length_count);
//
//		// fill track with matches
//		for (const int j = 0; j < tracklet_length_count; j++)
//		{
//			matches_msg_ros::FeaturePoint feature;
//			feature.u = std::rand()%1000;
//			feature.v = std::rand()%1000;
//
//			track.feature_points.push_back(std::move(feature));
//		}
//
//		trackletMsg.tracks.push_back(std::move(track));
//	}
//
//}
//
//}
//
//
//
////A google test function (uncomment the next function, add code and
////change the names TestGroupName and TestName)
// TEST(Tracklets, Converter) {
//	// Init tracklet msg
//	matches_msg_ros::MatchesMsg trackletMsg;
//	matches_msg_depth_ros::MatchesMsg trackletDepthMsg;
//	CreateTrackletMsg(trackletMsg);
//
//	// process
//	std::vector<std::shared_ptr<tracklet_depth::TempTrackletFrame>> tempFrames;
//	tracklet_depth::ExractNewTrackletFrames(tracklets_in, tempFrames);
//
//	Eigen::VectorXd depthsLastFrame;
//	Eigen::VectorXd depthsCurFrame;
//	depthsLastFrame.setConstant(trackletMsg.tracks.size(), -1);
//	depthsCurFrame.setConstant(trackletMsg.tracks.size(), -1);
//
//	std::vector<TypeTrackletKey> updatetIds;
//	auto trackletsCount = tracklet_depth::SaveFeatureDepths(tempFrames, depthsLastFrame, depthsCurFrame, updatetIds);
//
//
//
//
//	depthsLastFrame.setConstant(10, -1);

// convert tracklet


// Check if the resulting tracklet msg equals the inital message (depth values ignored)


//}

// int main(int argc, char **argv) {
//    ros::init(argc, argv, "unittest");
//    testing::InitGoogleTest(&argc, argv);
//    return RUN_ALL_TESTS();
//}
