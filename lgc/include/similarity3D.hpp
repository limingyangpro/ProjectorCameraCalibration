/*
 * similarity3D.hpp
 *
 *  Created on: Feb 11, 2015
 *      Author: liming
 */

#ifndef LGC2LIB_SIMILARITY3D_HPP_
#define LGC2LIB_SIMILARITY3D_HPP_

#include "transformation.hpp"

namespace lgc2{

class Similarity3D : public AbstractTransformation<pcl::PointXYZ>
{
protected:
	float rotDiff, scaleDiff, transDiff;
	void get3DRotScaleTranslation(const cv::Mat_<float>& transform, cv::Vec4f &quart, float &scale, cv::Vec3f &translation) const;

public:
	Similarity3D();
	Similarity3D(float scalediff, float rotdiff, float transdiff);
	/**
	 * Transform one subset to another, with specified transformation
	 */
	virtual void transform(const vector<pcl::PointXYZ> &inputset, vector<pcl::PointXYZ> &outputset, const cv::Mat_<float>& trans) const;

	/**
	 * Find transformation : find transformation between two subsets using least square methods
	 * Input : two subsets
	 * Output : transformation error (return value), ordinary transformation matrix
	 */
	virtual float findTransformLS(const vector<pcl::PointXYZ> &fromsubset, const vector<pcl::PointXYZ> &tosubset, cv::Mat_<float> &ordTransform) const;

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
	virtual void moveTransformation(cv::Mat_<float>& ioTransformation, const pcl::PointXYZ &plusDist, const pcl::PointXYZ &minusDist) const;

	/**
	 * Difference between two transformations
	 */
	virtual float getDifference(const cv::Mat_<float>& transf0, const cv::Mat_<float>& transf1, float &rotdiff, float &scalediff, float &transdiff) const;
	/**
	 * Generate arbitrary transformation 3D
	 */
	static cv::Mat_<float> getArbitraryTransformation();
};

}




#endif /* LGC2LIB_SIMILARITY3D_HPP_ */
