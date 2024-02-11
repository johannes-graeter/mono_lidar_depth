// Copyright 2018. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once
#include <iostream>
#include <vector>
#include <Eigen/Eigen>

/**
* @class CameraPinhole
* @par
*
* interface for a pinhole camera
*/
class CameraPinhole final {
public:
    // Specify Eigen Alignment, should be obsolete with c++17
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public: // Attributes.
  using Ptr=std::shared_ptr<CameraPinhole>;
  using ConstPtr=std::shared_ptr<const CameraPinhole>;
public: // Public methods.
    // Default constructor.
    explicit CameraPinhole(
        int width, int height, double focal_length, double principal_point_x, double principal_point_y)
            : width_(width), height_(height), focal_length_(focal_length), principal_point_x_(principal_point_x),
              principal_point_y_(principal_point_y) {
        ;
    }

    // Default destructor.
    ~CameraPinhole() = default;

    // Default move.
    CameraPinhole(CameraPinhole&& other) = default;
    CameraPinhole& operator=(CameraPinhole&& other) = default;

    // Default copy.
    CameraPinhole(const CameraPinhole& other) = default;
    CameraPinhole& operator=(const CameraPinhole& other) = default;

    void getImageSize(int& width, int& height) const {
        width = width_;
        height = height_;
    }

    template <int N>
    void getViewingRays(const Eigen::Matrix<double, 2, N>& image_points,
                        Eigen::Matrix<double, 3, N>& support_points,
                        Eigen::Matrix<double, 3, N>& directions) const {
        Eigen::Matrix3d intrinsics = makeIntrinsics();

        // Copy for circumventing the copy constructor
        // (would be different for static and dynamic sized matrices).
        //        Eigen::Matrix<double, 3, N> image_points_hom;
        //        image_points_hom.setOnes(3, directions.cols());
        //        // Dynamic access in case that N==Eigen::Dynamic.
        //        image_points_hom.block(0, 0, 2, N) = image_points;
        // Get directions.
        directions = intrinsics.inverse() * image_points.colwise().homogeneous();
        directions = directions.colwise().normalized();
        // We only support SVP models.
        support_points.setZero(3, directions.cols());
    }

    // void getViewingRays(const Eigen::Matrix<double, 2, Eigen::Dynamic>& image_points,
    //                     Eigen::Matrix<double, 3, Eigen::Dynamic>& support_points,
    //                     Eigen::Matrix<double, 3, Eigen::Dynamic>& directions) const {
    //     Eigen::Matrix3d intrinsics = makeIntrinsics();

    //     Eigen::Matrix<double, 3, Eigen::Dynamic> image_points_hom = support_points;
    //     image_points_hom.setOnes();
    //     image_points_hom.block(0, 0, 2, image_points.cols()) = image_points;
    //     directions = intrinsics.inverse() * image_points_hom;
    //     directions = directions.colwise().normalized();

    //     support_points.setZero();
    // }
    template <int N>
    Eigen::Array<bool, 1, N> getImagePoints(const Eigen::Matrix<double, 3, N>& points3d,
                                            Eigen::Matrix<double, 2, N>& image_points) const {
        Eigen::Matrix3d intrinsics = makeIntrinsics();
        Eigen::Matrix<double, 3, N> image_points_3d = intrinsics * points3d;
        // https: // eigen.tuxfamily.org/dox/group__Geometry__Module.html#title42
        image_points = image_points_3d.colwise().hnormalized();

        // Calculate if output is valid (in bounds of height and width)
        Eigen::Array<bool, 1, N> is_in_bounds =
            (image_points.row(0).array() >= 0.) && (image_points.row(0).array() <= static_cast<double>(width_)) &&
            (image_points.row(1).array() >= 0.) && (image_points.row(1).array() <= static_cast<double>(height_));
        return is_in_bounds;
    }

private:
    Eigen::Matrix3d makeIntrinsics() const {
        Eigen::Matrix3d intrinsics = Eigen::Matrix3d::Identity();
        intrinsics(0, 0) = intrinsics(1, 1) = focal_length_;
        intrinsics(0, 2) = principal_point_x_;
        intrinsics(1, 2) = principal_point_y_;
        return intrinsics;
    }

private:
    int width_;
    int height_;
    double focal_length_;
    double principal_point_x_;
    double principal_point_y_;
};
