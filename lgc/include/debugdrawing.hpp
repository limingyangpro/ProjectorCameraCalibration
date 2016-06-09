/*
 * debugdrawing.hpp
 *
 *  Created on: Apr 9, 2015
 *      Author: liming
 */

#ifndef MATCHINGLIB_DEBUGDRAWING_HPP_
#define MATCHINGLIB_DEBUGDRAWING_HPP_

#include "basiclib/basictype.h"

class DebugDrawing
{
private:
	static cv::Mat createWhiteImg(int height, int width)
	{
		cv::Mat temp_image = cv::Mat::zeros(height, width, CV_8UC3 );
		temp_image.setTo(cv::Scalar(255,255,255));

		return temp_image;
	}

public:
static void drawPoints(cv::Mat& ioImg, const std::vector<Point2D> &iPoints, int radius = 3, cv::Scalar color = cv::Scalar(255,0,0))
{
   for( int i = 0; i < iPoints.size(); i++ )
   { cv::circle( ioImg, iPoints[i], radius, color, -1, 8, 0 ); }
}

static void drawPoints(int height, int width, const std::vector<Point2D> &iPoints, int radius = 3, cv::Scalar color = cv::Scalar(255,0,0))
{
	cv::Mat temp_image = createWhiteImg(height, width);
   for( int i = 0; i < iPoints.size(); i++ )
   { cv::circle( temp_image, iPoints[i], radius, color, -1, 8, 0 ); }

   imshow("temp draw", temp_image);
   cv::waitKey(-1);
}

static void drawMultiPointVectors(int height, int width, const std::vector< vector<Point2D> > &iPoints)
{
	cv::Mat temp_image = createWhiteImg(height, width);

	for (int i = 0 ; i <iPoints.size() ; i++)
	{
		drawPoints(temp_image, iPoints[i], i);
	}

	imshow("temp draw", temp_image);
	cv::waitKey(-1);
}

};


#endif /* MATCHINGLIB_DEBUGDRAWING_HPP_ */
