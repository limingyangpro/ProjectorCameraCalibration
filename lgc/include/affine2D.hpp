/*
 * Affine2D.h
 *
 *  Created on: Nov 26, 2014
 *      Author: liming
 */

#ifndef AFFINE2D_HPP_
#define AFFINE2D_HPP_

#include "transformation.hpp"

namespace lgc2{

class Affine2D : public AbstractTransformation<pcl::PointXY>
{
protected:
	float rotDiff, scaleDiff;
	void getAffineRotAndScale(const cv::Mat_<float>& trans, float rot[2], float scale[2]) const;

public:
	Affine2D();
	Affine2D(float scalediff, float rotdiff);
	/**
	 * Transform one subset to another, with specified transformation
	 */
	virtual void transform(const vector<pcl::PointXY> &inputset, vector<pcl::PointXY> &outputset, const cv::Mat_<float>& trans) const;

	/**
	 * Find transformation : find transformation between two subsets using least square methods
	 * Input : two subsets
	 * Output : transformation error (return value), ordinary transformation matrix
	 */
	virtual float findTransformLS(const vector<pcl::PointXY> &fromsubset, const vector<pcl::PointXY> &tosubset, cv::Mat_<float> &ordTransform) const;

	/**
	 * Find transformation : find transformation between two subsets using RANSAC methods
	 * Input : two subsets
	 * Output : transformation error (return value), localtransformation matrix
	 */
//	virtual float findTransformRANSAC(const vector<pcl::PointXY> &refsubset, const vector<pcl::PointXY> &imgsubset, float tol, cv::Mat_<float> &ordTransform) const;

	/**
	 * Convert ordinary transformation to Geometric meaningful values (matrix size non change)
	 */
//	virtual cv::Mat_<float> toGeoTransform(const cv::Mat_<float>& ordTransform) const;

	/**
	 * Convert geometric meaningful values to ordinary transformation
	 */
//	virtual cv::Mat_<float> toOrdTransform(const cv::Mat_<float>& geoTransform) const;

	/**
	 * Check if the ordinary transformation is meaningful
	 */
	virtual bool isTransformOK(const cv::Mat_<float>& ordTransform) const;

	/**
	 * To judge if two transformation is similar
	 */
	virtual float isTransformSimilar(const cv::Mat_<float>& trans0, const cv::Mat_<float>& trans1) const;

	/**
	 * Weighted average of two transformations, in sense of geometric
	 */
//	virtual cv::Mat_<float> weightedAverage(const cv::Mat_<float>& trans0, float weight0, const cv::Mat_<float>& trans1, float weight1, int transf_mode) const;
	/**
	 * Get the acceptable error sum for local transformation
	 */
	virtual float getTransformAcceptableErrorSumCoef(int ptnum) const;

	/**
	 * Adjust translation components in the transformation
	 * we use plusDist and minus dist because the stupid PCL can not do point minus directly
	 */
	virtual void moveTransformation(cv::Mat_<float>& ioTransformation, const pcl::PointXY &plusDist, const pcl::PointXY &minusDist) const;
};

}

#endif /* AFFINE2D_HPP_ */
