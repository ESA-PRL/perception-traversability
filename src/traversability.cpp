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

#include "traversability.hpp"
#include <iostream>

using namespace std;
using namespace traversability;

// creator
Traversability::Traversability()
{
}

void Traversability::welcome()
{
    cout << "Welcome!" << endl;
}

void Traversability::configureTraversability(float max_obstacle, float max_slope)
{
    elevation_map_set = 1;
}

void Traversability::setObstacleLaplacian(int kernel_size, float threshold)
{
    laplacian_kernel_size = kernel_size;
    laplacian_threshold = threshold;
}

void Traversability::setObstacleDetection(int kernel_size_o, int iteration_o,
        int kernel_size_s, int iteration_s)
{
    obstacle_kernel_size = kernel_size_o;
    obstacle_iterations = iteration_o;
    obstacle_vicinity_kernel_size = kernel_size_s;
    obstacle_vicinity_iterations = iteration_s;
}

void Traversability::setMapParameters(float size_width, float size_height, float resolution, int scale)
{
    map_size_width = size_width;
    map_size_height = size_height;
    map_resolution = resolution;
    map_cells_width = floor(map_size_width / map_resolution);
    map_cells_height = floor(map_size_height / map_resolution);

    // prepare elevation map
    elevation_map.create(map_cells_width, map_cells_height, CV_32FC1);
    elevation_map_mask.create(map_cells_width, map_cells_height, CV_8UC1);

    slope_map_scale = scale;
}

void Traversability::setElevationMap(std::vector<float> data, int width, int height)
{
    // delete previous
    // elevation_map.reset
    // reserve space for new map
    // elevation_map.create()
    for (int i=0; i<width; i++)
    {
	for (int j=0; j<height; i++)
	{
	    //! check what to do with NaN values
	    elevation_map.at<float>(i, j) = data[i*height+j];
	}
    }
}

void Traversability::elevationMapInterpolate()
{
    // Do linear interpolation with the matrix in the y direction
    // TODO perhaps it is more justified to do it in polar coordinates with the center being at the stereo camera position
    // TODO find a way to do dilation using nearest neighbor instead of this approach which presents boundary issues
    int16_t column, row, p, start_index, end_index;
    float value, value_previous, fraction, start_value, end_value;

    elevation_map.copyTo(elevation_map_interpolated); // TODO unsure this is good idea..

    for(column = 0; column < elevation_map.cols; column++)
    {
        value_previous = 0.0f;
        start_index = 0;
        end_index = 0;

        for(row = 0; row < elevation_map.rows; row++)
        {
            // Get the pixel value in the matrix
            value = elevation_map_interpolated.at<float>(row, column);

            if(row == 0 && value == 0.0f)
            {
                // First value in row is missing, assign 0.0f for start
                start_index = 0;
                start_value = 0.0f;
            }
            else if(start_index == -1 && value == 0.0f && value_previous != 0.0f)
            {
                // Start of the missing data is the previous cell
                start_index = row - 1;
                start_value = value_previous;
            }
            else if((value != 0.0f && value_previous == 0.0f) || (value == 0.0f && row == elevation_map.rows - 1 && start_index != -1))
            {
                // End of the missing data
                end_index = row;
                end_value = value;

                // Interpolate
                for(p = start_index; p <= end_index; p++)
                {
                    // Evaluate the linear interpolation
                    fraction = (float)(p - start_index) / (float)(end_index - start_index);
                    elevation_map_interpolated.at<float>(p, column) = (start_value * (1.0f - fraction) + end_value * fraction);
                }
                start_index = -1;
                end_index = -1;
            }

            // Save the values for next iteration
            value_previous = value;
        }
    }

    //same along the other direction to prevent approximation errors when the image is taken with a ptu
    for(row = 0; row < elevation_map.rows; row++)
    {
        value_previous = 0.0f;
        start_index = 0;
        end_index = 0;

        for(column = 0; column < elevation_map.cols; column++)
        {
            // Get the pixel value in the matrix
            value = elevation_map_interpolated.at<float>(row, column);

            if(column == 0 && value == 0.0f)
            {
                // First value in row is missing, assign 0.0f for start
                start_index = 0;
                start_value = 0.0f;
            }
            else if(start_index == -1 && value == 0.0f && value_previous != 0.0f)
            {
                // Start of the missing data is the previous cell
                start_index = column - 1;
                start_value = value_previous;
            }
            else if((value != 0.0f && value_previous == 0.0f) || (value == 0.0f && column == elevation_map.cols - 1 && start_index != -1))
            {
                // End of the missing data
                end_index = column;
                end_value = value;

                // Interpolate
                for(p = start_index; p <= end_index; p++)
                {
                    // Evaluate the linear interpolation
                    fraction = (float)(p - start_index) / (float)(end_index - start_index);
                    elevation_map_interpolated.at<float>(row, p) = (start_value * (1.0f - fraction) + end_value * fraction);
                }
                start_index = -1;
                end_index = -1;
            }

            // Save the values for next iteration
            value_previous = value;
        }
    }
}

void Traversability::elevationMap2SlopeMap()
{
    float sample_scale = 1.0/slope_map_scale;
    cv::Mat interpSlope;

    // Scale interpolated elevationMap and mask
    cv::resize(elevation_map_mask, elevation_map_mask_scaled, cv::Size(), sample_scale, sample_scale, cv::INTER_NEAREST);
    cv::resize(elevation_map_interpolated, interpSlope, cv::Size(), sample_scale, sample_scale, cv::INTER_LINEAR);

    // Find gradient along x and y directions
    cv::Mat elevation_map_gradient_x, elevation_map_gradient_y;
    cv::Sobel(interpSlope, elevation_map_gradient_x, CV_32FC1, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
    cv::Sobel(interpSlope, elevation_map_gradient_y, CV_32FC1, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);

    // Square the x and y gradient matrices, this makes them positive, needed to obtain absolute slope value
    cv::pow(elevation_map_gradient_x, 2, elevation_map_gradient_x);
    cv::pow(elevation_map_gradient_y, 2, elevation_map_gradient_y);

    // Add squared slope of x and y together
    cv::add(elevation_map_gradient_x, elevation_map_gradient_y, interpSlope);

    // Take the square root to get the real slope
    cv::sqrt(interpSlope, interpSlope);

    // Apply the mask to extract only the valid data TODO also resize already?
    interpSlope.copyTo(slope_map, elevation_map_mask_scaled);
}

void Traversability::thresholdSlopeMap(float slope_threshold)
{
    // Threshold the slope map to tag slopes that are not traversable
    // TODO unterstand why it is slope_map_scale^2 (probably because scaled in two directions)
    float threshold = slope_threshold*slope_map_scale*slope_map_scale*map_resolution; // tan(st)=t/(sms*sms*hr)
    cv::resize(slope_map, slope_map, cv::Size(elevation_map.cols, elevation_map.rows), 0 , 0, cv::INTER_LINEAR);
    cv::threshold(slope_map, slope_map_thresholded, threshold, 1.0f, cv::THRESH_BINARY);
}

void Traversability::detectObstacles(float elevation_threshold)
{
    cv::Mat interpLaplacian;

    // Find the obstacles with a Laplace filter (gradients)
    cv::Laplacian(elevation_map_interpolated, interpLaplacian, CV_32FC1, laplacian_kernel_size);

    // Mask the laplacian to remove invalid interpolated data
    elevation_map_laplacian.setTo(0);
    interpLaplacian.copyTo(elevation_map_laplacian, elevation_map_mask);

    // The obstacle map is based on the laplacien image and a threshold defines probably obstacles
    cv::threshold(-elevation_map_laplacian, elevation_map_laplacian_thresholded, laplacian_threshold, 1.0f, cv::THRESH_BINARY);   // TODO sure it is so? -laplacian or abs(laplacian)[

    // find contours, dilate obstacles a bit first so that all or
    // slightly more of it is included in the contour
    cv::Mat contour_mask;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
            cv::Size( obstacle_kernel_size,obstacle_kernel_size )); // square kernel

    elevation_map_laplacian_thresholded.convertTo(contour_mask, CV_8UC1);
    cv::dilate(contour_mask, contour_mask, element, cv::Point( -1, -1 ),
            obstacle_iterations);
    std::vector < std::vector<cv::Point> > contours;
    findContours(contour_mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    // prepare variables used to iterate through the single obstacles
    cv::Mat single_contour_mask;
    cv::Mat single_surround_mask;
    cv::Mat combined_mask;
    cv::Mat obstacle_mask;
    contour_mask.copyTo(single_contour_mask); // todo unsure if 4 lines useful
    contour_mask.copyTo(single_surround_mask);
    contour_mask.copyTo(combined_mask);
    contour_mask.copyTo(obstacle_mask);
    obstacle_mask.setTo(cv::Scalar(0));
    obstacle_map.setTo(cv::Scalar(0));

    int nContours = contours.size();
    double maxVal[nContours];
    double minVal[nContours]; // todo set up so that holes can be found out too
    double inside_mean[nContours];
    double outside_mean[nContours];
    int obstacle_check[nContours];
    element = cv::getStructuringElement( cv::MORPH_RECT,
            cv::Size( obstacle_vicinity_kernel_size,obstacle_vicinity_kernel_size )); // square kernel

    // Iterate through the obstacles:
    // - dilate and mask to find the area around obstacles
    // - compare elevation of obstacles points to immediate surroundings
    for( int iContour = 0; iContour < nContours; iContour++)
    {
        single_contour_mask.setTo(cv::Scalar(0)); // initialize mask
        single_surround_mask.setTo(cv::Scalar(0));
        cv::drawContours(single_contour_mask, contours, iContour, cv::Scalar(1), CV_FILLED); // fill mask with only one contour
        cv::dilate(single_contour_mask, single_surround_mask, element, cv::Point( -1, -1 ) , obstacle_vicinity_iterations);
        cv::bitwise_xor(single_contour_mask,single_surround_mask, single_surround_mask); // keep original mask
        cv::bitwise_and(elevation_map_mask,single_contour_mask, single_contour_mask); // remove regions where we do not have elevationmap data
        cv::bitwise_and(elevation_map_mask,single_surround_mask, single_surround_mask);
        inside_mean[iContour] = cv::mean(elevation_map,single_contour_mask)[0];
        outside_mean[iContour] = cv::mean(elevation_map,single_surround_mask)[0];
        cv::minMaxLoc(elevation_map, &minVal[iContour], &maxVal[iContour], NULL, NULL, single_contour_mask); // TODO check again because now peak relies on only one point (should be like top 3 or 5 points)
        if((maxVal[iContour] - outside_mean[iContour]) > elevation_threshold) // this is indeed an obstacle
        {
            obstacle_check[iContour] = 1;
            cv::bitwise_or(single_contour_mask, obstacle_mask, obstacle_mask); // add obstacle to total obstacle mask
        }
        else
        {
            obstacle_check[iContour] = 0;
        }
    }
    obstacle_mask.convertTo(obstacle_map, CV_32FC1);
}

void Traversability::dilateObstacles(float robot_size, int iterations)
{
    // kernel size is dependant on map resolution and robot width
    int kernel_size = (int)(robot_size/map_resolution) + 1;
    dilation_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
            cv::Size(kernel_size,kernel_size)); // round kernel;
    dilation_iterations = iterations;

    // Check here dilation computation
    cv::dilate(traversability_map, traversability_map, dilation_kernel, cv::Point( -1, -1 ),
            dilation_iterations);
}

cv::Mat Traversability::computeTraversability()
{
    // combine two traversability if feasible
    cv::Mat t1, t2;
    obstacle_map.convertTo(t1, CV_8UC1); // to do decide what to do with map types *masks should be all 8UC1?
    slope_map_thresholded.convertTo(t2, CV_8UC1);
    cv::multiply(t2, 255, t2); // multiplication because of previous comment
    cv::bitwise_or(t1, t2, traversability_map);
    traversability_map.convertTo(traversability_map, CV_32FC1);
    return traversability_map;
}

cv::Mat Traversability::local2globalOrientation(cv::Mat local_map, float yaw)
{
    // First of all dilate the obstacles in local map
    
    // kernel size is dependant on map resolution and robot width
    //int kernel_size = (int)(robot_size/(map_resolution)) + 1;
    //dilation_iterations = iterations;
    
    int kernel_size = 71;
    dilation_iterations = 2;
    dilation_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
            cv::Size(kernel_size,kernel_size)); // round kernel;

    // Check here dilation computation
    cv::dilate(local_map, local_map, dilation_kernel, cv::Point( -1, -1 ),
            dilation_iterations);

    // After dilation, create enlarged map with the size that fits the rotated map in any posible rotation;
    insert_rows = ceil(sqrt(local_map.rows*local_map.rows+local_map.cols*local_map.cols)); // use ceil() function and make sure it is odd number
    insert_cols = ceil(sqrt(local_map.rows*local_map.rows+local_map.cols*local_map.cols)); // use ceil() function and make sure it is odd number

    rotated_map.create(insert_rows, insert_cols, CV_8UC1);
    rotated_map.setTo(0);

    // copy local map in the middle of enlarged map todo create roi with rect to ease readability
    local_map.copyTo(rotated_map(cv::Rect((insert_cols-local_map.cols)/2,(insert_rows-local_map.rows)/2,local_map.cols, local_map.rows)));

    // rotation center
    cv::Point2f rot_center((float)insert_rows/2.0,(float)insert_cols/2.0);

    // Rotate yaw additional -90 degress for path planner global map convention
    yaw -= M_PI/2.0;
    // yaw rotation matrix
    cv::Mat transform = cv::getRotationMatrix2D(rot_center,(yaw*(180/M_PI)),1.0);

    // apply yaw rotation
    cv::warpAffine(rotated_map,rotated_map,transform,rotated_map.size(),CV_INTER_LINEAR);

    return rotated_map;
}

void Traversability::local2globalOrientation_legacy(cv::Mat relative_map, cv::Mat relative_mask_map, float yaw)
{
    // create enlarged map with double the size of the input map
    insert_rows = relative_map.rows*2;
    insert_cols = relative_map.cols*2;

    rotated_map.create(insert_rows, insert_cols, CV_32FC1);
    rotated_mask_map.create(insert_rows, insert_cols, CV_8UC1);
    rotated_map.setTo(0.0);
    rotated_mask_map.setTo(0);

    // copy relative map in the middle of enlarged map todo create a  roi with rect to ease readability
    cv::Mat tmp;
    relative_map.convertTo(tmp,CV_32FC1); //force trav maps to be 32F so they can have Nan. TODO Find better way (or kill RAM)
    tmp.copyTo(rotated_map(cv::Rect(relative_map.cols/2,relative_map.rows,relative_map.cols, relative_map.rows)));
    relative_mask_map.copyTo(rotated_mask_map(cv::Rect(relative_mask_map.cols/2,relative_mask_map.rows,relative_mask_map.cols, relative_mask_map.rows)));

    rotated_map.setTo(std::numeric_limits<float>::quiet_NaN(),rotated_mask_map==0);

    // rotation center
    cv::Point2f rot_center((float)insert_rows/2.0,(float)insert_cols/2.0);

    // yaw rotation matrix
    cv::Mat transform = cv::getRotationMatrix2D(rot_center,(yaw/3.14159*180.0),1.0);

    // apply yaw rotation
    cv::warpAffine(rotated_map,rotated_map,transform,rotated_map.size(),CV_INTER_LINEAR);
    cv::warpAffine(rotated_mask_map,rotated_mask_map,transform,rotated_mask_map.size(),CV_INTER_LINEAR);
}

