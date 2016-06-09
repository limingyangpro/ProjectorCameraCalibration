/*
 * perspective2D.h
 *
 *  Created on: Nov 26, 2014
 *      Author: liming
 */

#ifndef PERSPECTIVE2D_HPP_
#define PERSPECTIVE2D_HPP_

#include "transformation.hpp"

namespace lgc2
{
class Perspective2D : public AbstractTransformation<pcl::PointXY>
{
public:
	Perspective2D();
	/**
	 * Transform one subset to another, with specified transformation
	 */
	virtual void transform(const vector<pcl::PointXY> &inputset, vector<pcl::PointXY> &outputset, const cv::Mat_<float>& trans) const;

	/**
	 * Find transformation : find transformation between two subsets using least square methods
	 * Input : two subsets
	 * Output : transformation error (return value), localtransformation matrix
	 */
	virtual float findTransformLS(const vector<pcl::PointXY> &fromsubset, const vector<pcl::PointXY> &tosubset, cv::Mat_<float>& trans) const;

	/**
	 * Find transformation : find transformation between two subsets using RANSAC methods
	 * Input : two subsets
	 * Output : transformation error (return value), localtransformation matrix
	 */
//	virtual float findTransformRANSAC(const vector<pcl::PointXY> &fromsubset, const vector<pcl::PointXY> &tosubset, float tol, cv::Mat_<float>& trans) const;
};

}


#endif /* PERSPECTIVE2D_HPP_ */
