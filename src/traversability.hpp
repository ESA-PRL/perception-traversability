/*
 * This file is part of Traversability.
 * Copyright (C) 2018 Martin Azkarate, reuse of Cartographer software oroginally developed by Marco
 * Pagnamenta
 * Planetary Robotics Lab (PRL), European Space Agency (ESA)
 *
 * Traversability is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Traversability is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Traversability. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

//! ToDo: Remove unncessary includes

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// PCL  -> Delete all probably
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/crop_box.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/common/transforms.h>

// Sys -> Delete probably
#include <sys/time.h>

namespace traversability
{

class Traversability
{
  public:
    Traversability();

    // parameters setters
    void configureTraversability(float map_resolution,
                                 int slope_map_scale,
                                 float slope_threshold,
                                 float elevation_threshold,
                                 int laplacian_kernel_size,
                                 float laplacian_threshold,
                                 int obstacle_kernel_size,
                                 int obstacle_iterations,
                                 int obstacle_vicinity_kernel_size,
                                 int obstacle_vicinity_iterations,
                                 float robot_size,
                                 int dilation_iterations);
    void setElevationMap(std::vector<float>, int width, int height);

    // functionality
    void elevationMapInterpolate();
    void elevationMap2SlopeMap();
    void detectObstacles();
    void thresholdSlopeMap();
    void dilateTraversability();
    cv::Mat computeTraversability();

    // local to global rotation
    cv::Mat local2globalOrientation(cv::Mat local_map, float yaw);
    void local2globalOrientation_legacy(cv::Mat relative_map, cv::Mat relative_mask_map, float yaw);

  private:
    // map parameters
    float map_resolution;  // in meters per cell
    int slope_map_scale;

    // Obstacle laplacian parameters
    int laplacian_kernel_size;
    float laplacian_threshold;

    // Obstacle processing parameters
    int obstacle_kernel_size;
    int obstacle_iterations;
    int obstacle_vicinity_kernel_size;
    int obstacle_vicinity_iterations;

    float elevation_threshold;
    float slope_threshold;

    // Traversability dilation parameters
    float robot_size;  // in meters
    int dilation_iterations;

    cv::Mat elevation_map;              // elevationmap as converted from pc
    cv::Mat elevation_map_scaled;
    cv::Mat elevation_map_mask;         // elevationmap pixels without data mask
    cv::Mat elevation_map_mask_scaled;  // elevationmap pixels without data mask scaled to slope map
    cv::Mat elevation_map_interpolated;
    cv::Mat elevation_map_gradient_x;
    cv::Mat elevation_map_gradient_y;

    cv::Mat slope_map;              // slopemap built from subsampled gradient
    cv::Mat slope_map_thresholded;  // slopemap thresholded

    cv::Mat elevation_map_laplacian;              // laplacian of the elevationmap
    cv::Mat elevation_map_laplacian_thresholded;  // laplacian thresholded

    cv::Mat obstacle_map;  // binary map of unsurmontable obstacles

    cv::Mat traversability_map;  // accessible and inaccessible map

    // nrows and ncols of the map local mpa being integrated in the global one
    int insert_rows;
    int insert_cols;
    cv::Mat rotated_map;  // rotated input mapp to add
    cv::Mat rotated_mask_map;
};

}  // namespace traversability
