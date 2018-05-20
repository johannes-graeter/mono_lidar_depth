/*
 * NeighborFinderPixel.cpp
 *
 *  Created on: Feb 6, 2017
 *      Author: wilczynski
 */

#include "NeighborFinderPixel.h"
#include "Logger.h"

namespace Mono_Lidar {

NeighborFinderPixel::NeighborFinderPixel(const int imgWitdh,
                                         const int imgHeight,
                                         const int pixelSearchWidth,
                                         const int pixelSearchHeight)
        : _imgWitdth(imgWitdh), _imgHeight(imgHeight), _pixelSearchWidth(pixelSearchWidth),
          _pixelSearchHeight(pixelSearchHeight) {
    _img_points_lidar.resize(_imgWitdth, _imgHeight);
}

void NeighborFinderPixel::Initialize(std::shared_ptr<DepthEstimatorParameters>& parameters) {
    NeighborFinderBase::Initialize(parameters);

    _pixelSearchWidth = _parameters->pixelarea_search_witdh;
    _pixelSearchHeight = _parameters->pixelarea_search_height;
}

void NeighborFinderPixel::InitializeLidarProjection(const Eigen::Matrix2Xd& lidarPoints_img_cs,
                                                    const Eigen::Matrix3Xd& points_cs_camera,
                                                    const std::vector<int>& pointIndex) {
    Logger::Instance().Log(Logger::MethodStart, "DepthEstimator::TransformLidarPointsToImageFrame");

    int doubleProjectionCount = 0;
    using namespace std;

    int pointCount = lidarPoints_img_cs.cols();
    _img_points_lidar.setConstant(POINT_NOT_DEFINED);

    for (int i = 0; i < pointCount; i++) {
        int x_img = lidarPoints_img_cs(0, i);
        int y_img = lidarPoints_img_cs(1, i);

        int indexRaw = pointIndex[i];

        Eigen::Vector3d lidarPoint3D(
            points_cs_camera(0, indexRaw), points_cs_camera(1, indexRaw), points_cs_camera(2, indexRaw));

        // Check if there are multiple lidar point projections at the same image cooridnate and that the point is in
        // front of the camera
        if ((_img_points_lidar(x_img, y_img) == POINT_NOT_DEFINED) && (lidarPoint3D.z() > 0)) {
            // set the index to a lidar point to a image pixel
            _img_points_lidar(x_img, y_img) = i;
        }
    }

    Logger::Instance().Log(Logger::MethodEnd, "DepthEstimator::TransformLidarPointsToImageFrame");
}

void NeighborFinderPixel::getNeighbors(const Eigen::Vector2d& featurePoint_image_cs,
                                       const Eigen::Matrix3Xd& points_cs_camera,
                                       const std::vector<int>& pointIndex,
                                       std::vector<int>& pcIndicesCut,
                                       const std::shared_ptr<DepthCalcStatsSinglePoint>& calcStats,
                                       const float scaleWidth,
                                       const float scaleHeight) {
    double x = featurePoint_image_cs(0);
    double y = featurePoint_image_cs(1);

    double halfSizeX = (double)(_pixelSearchWidth)*0.5d * scaleWidth;
    double halfSizeY = (double)(_pixelSearchHeight)*0.5d * scaleHeight;

    double leftEdgeX = x - halfSizeX;
    double rightEdgeX = x + halfSizeX;
    double topEdgeY = y - halfSizeY;
    double bottomEdgeY = y + halfSizeY;

    if (leftEdgeX < 0)
        leftEdgeX = 0;
    if (rightEdgeX > (_imgWitdth - 1))
        rightEdgeX = _imgWitdth - 1;
    if (topEdgeY < 0)
        topEdgeY = 0;
    if (bottomEdgeY > (_imgHeight - 1))
        bottomEdgeY = _imgHeight - 1;

    using namespace std;

    for (int i = topEdgeY; i <= bottomEdgeY; i++) {
        for (int j = leftEdgeX; j <= rightEdgeX; j++) {
            if (_img_points_lidar(j, i) != POINT_NOT_DEFINED) {
                // get the index from the visible (in camera img) point cloud
                int index = _img_points_lidar(j, i);

                if (index == POINT_NOT_DEFINED)
                    continue;

                // get the index of the cut point cloud
                pcIndicesCut.push_back(index);
            }
        }
    }

    // debug log neighbors finder
    if (calcStats != NULL) {
        calcStats->_searchRectTopLeft = std::pair<int, int>((int)leftEdgeX, (int)topEdgeY);
        calcStats->_searchRectBottomRight = std::pair<int, int>((int)rightEdgeX, (int)bottomEdgeY);
    }
}
}
