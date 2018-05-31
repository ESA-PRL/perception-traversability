/*
 * This file is part of Traversability.
 * Copyright (C) 2018 Martin Azkarate, reuse of Cartographer software oroginally developed by Marco Pagnamenta
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

#ifndef _TRAVERSABILITY_HPP_
#define _TRAVERSABILITY_HPP_


//! ToDo: Remove unncessary includes

// OpenCV
#include <opencv2/core/core.hpp>
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
	    void welcome();

	    // parameters setters
	    void configureTraversability(float max_obstacle, float max_slope);
	    void setMapParameters(float size_width, float size_height, float resolution, int scale); // Needed??
	    void setObstacleLaplacian(int kernel_size, float threshold);    // Needed??
            void setObstacleDetection(int kernel_size_o, int iteration_o, int kernel_size_s, int iteration_s); // Needed??
	    void setElevationMap(std::vector<float>, int width, int height);
	    

	    // functionality
	    void elevationMapInterpolate();
            void elevationMap2SlopeMap();
            void detectObstacles(float elevation_threshold);
            void thresholdSlopeMap(float slope_threshold);
	    void dilateObstacles(float robot_size, int iterations);
            cv::Mat computeTraversability();

	    // local to global rotation
            void local2globalOrientation(cv::Mat relative_map, cv::Mat relative_mask_map, float yaw);

	    // images/pcl/data getters

	private:
	    bool elevation_map_set;

	    // map parameters
            float map_size_width;  // in meters
            float map_size_height; // in meters
            float map_resolution;  // in meters
            float map_cells_width; // # of cells in width
            float map_cells_height;// # of cells in height
            int slope_map_scale;

            // Obstacle laplacian parameters
            int laplacian_kernel_size;
            float laplacian_threshold;

            // Obstacle processing parameters
            int obstacle_kernel_size;
            int obstacle_iterations;
            int obstacle_vicinity_kernel_size;
            int obstacle_vicinity_iterations;

	    // Obstacle dilation parameters
	    cv::Mat dilation_kernel;
            int dilation_iterations;

            cv::Mat elevation_map;     // elevationmap as converted from pc
            cv::Mat elevation_map_mask; // elevationmap pixels without data mask
            cv::Mat elevation_map_mask_scaled; // elevationmap pixels without data mask scaled to slope map
            cv::Mat elevation_map_interpolated;
            cv::Mat elevation_map_gradient_x;
            cv::Mat elevation_map_gradient_y;

            cv::Mat slope_map;      // slopemap built from subsampled gradient
            cv::Mat slope_map_mask; // slopemap pixels without data mask
            cv::Mat slope_map_thresholded; // slopemap thresholded

            cv::Mat elevation_map_laplacian; // laplacian of the elevationmap
            cv::Mat elevation_map_laplacian_thresholded; // laplacian thresholded

            cv::Mat obstacle_map; // binary map of unsurmontable obstacles

            cv::Mat traversability_map; // accessible and inaccessible map

	    int insert_rows, insert_cols; // nrows and ncols of the map local mpa being integrated in the global one
            cv::Mat rotated_map; // rotated input mapp to add
            cv::Mat rotated_mask_map;

    };
}

#endif
