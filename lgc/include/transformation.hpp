/*
 * transformation.h
 *
 *  Created on: Nov 24, 2014
 *      Author: liming
 */

#ifndef TRANSFORMATION_HPP_
#define TRANSFORMATION_HPP_

#include "basictype.h"
#include "mathfuns.h"
#include "boost/shared_array.hpp"

namespace lgc2
{

/**
 * This is an element in the hashtable
 */
struct HashTableElem
{
	int refptID;             /**< This is the corresponding reference point id to this element */
	vector<int> neighborID;         /**< Neighbor points' N° of the reference point */
};

/**
 * This is the descriptor element, contains
 * the two smallest surface ratio among the four, this two freedoms are unconstraint, between[0,0.5]
 * Distrubtion quasi-uniform
 */
struct Descriptor
{
	cv::Point2f value;
	int config;
	HashTableElem elem;          //Record the element of this descriptor
};

#define TRANSF_MODE_ORD      100000
#define TRANSF_MODE_GEO      100001

template <typename PointT>
class AbstractTransformation
{
public:
	int dimention;            //dimention of the points for transformation
	int dof;                  //degree of freedom
	int minPtNum;             //number of points necessary to estimate such transformation
	int desPtNum;             //number of points used to form a descriptor, usually minPtNum+1
	int nConfig;              //number of configuration for the descriptor
	static const int failure_config = -1;  //configuration that can not be managed
//	int transformationSize;                //Number of floats used to record current transformation
	cv::Mat_<float> transformationTemplate;      //template of a transformation matrix, only size is important

private:
	void printAlert() const;

public:

	/**
	 * Get descriptor for specified transformation
	 * Input:
	 *      BasePt : the cooridate of the main point
	 *      vertexs : the coordiates of other points ( from nearest to farest )
	 * Output:
	 *      descList : a list of desc
	 *      desc.value :           the value of descriptor for the main point
	 *      desc.config :          the configuration of the descriptor
	 *      desc.elem.neighborID : order of "other points" (for 3 points, 012, 021, 102, 120, 210. 6 possibilities)
	 *      tol :                   tolerance of the descriptor Sigma
	 *
	 */
	virtual void getDescriptorForFirstPt(const PointT &basePt, const vector<PointT> &vertexs, float threshold, list<Descriptor>& descList, list< boost::shared_array<float> >* tol){printAlert();};

	/**
	 * Normalize descriptor into [0,1]
	 */
	virtual float normalize(float value) const {printAlert();};

	/**
	 * Transform one subset to another, with specified transformation
	 */
	virtual void transform(const vector<PointT> &inputset, vector<PointT> &outputset, const cv::Mat_<float>& trans) const{printAlert();};

	/**
	 * Find transformation : find transformation between two subsets using least square methods
	 * Input : two subsets
	 * Output : transformation error (return value), ordinary transformation matrix
	 */
	virtual float findTransformLS(const vector<PointT> &fromsubset, const vector<PointT> &tosubset, cv::Mat_<float> &ordTransform) const {printAlert(); return 0.0;};

	/**
	 * Find transformation : find transformation between two subsets using RANSAC methods
	 * Input : two subsets and tolerance of RANSAC
	 * Output : transformation error (return value), ordinary transformation matrix
	 */
//	virtual float findTransformRANSAC(const vector<PointT> &refsubset, const vector<PointT> &imgsubset, float tol, cv::Mat_<float> &ordTransform) const {printAlert(); return 0.0;};

	/**
	 * Convert ordinary transformation to Geometric meaningful values (matrix size non change)
	 */
//	virtual cv::Mat_<float> toGeoTransform(const cv::Mat_<float>& ordTransform) const {printAlert();};

	/**
	 * Convert geometric meaningful values to ordinary transformation
	 */
//	virtual cv::Mat_<float> toOrdTransform(const cv::Mat_<float>& geoTransform) const {printAlert();};

	/**
	 * Check if the ordinary transformation is meaningful
	 */
	virtual bool isTransformOK(const cv::Mat_<float>& ordTransform) const {printAlert(); return 0.0;};

	/**
	 * To judge if two transformation is similar
	 */
	virtual float isTransformSimilar(const cv::Mat_<float>& trans0, const cv::Mat_<float>& trans1) const {printAlert(); return 0.0;};

	/**
	 * Weighted average of two transformations, in sense of geometric
	 */
//	virtual cv::Mat_<float> weightedAverage(const cv::Mat_<float>& trans0, float weight0, const cv::Mat_<float>& trans1, float weight1, int transf_mode){printAlert();};

	/**
	 * Get the acceptable error sum for local transformation
	 */
	virtual float getTransformAcceptableErrorSumCoef(int ptnum) const {printAlert(); return 0.0;};

	/**
	 * Adjust translation components in the transformation
	 * we use plusDist and minus dist because the stupid PCL can not do point minus directly
	 */
	virtual void moveTransformation(cv::Mat_<float>& ioTransformation, const PointT &plusDist, const PointT &minusDist) const {};
};

//template <typename PointT> int AbstractTransformation<PointT>::dimention;
//template <typename PointT> int AbstractTransformation<PointT>::dof;
//template <typename PointT> int AbstractTransformation<PointT>::minPtNum;
//template <typename PointT> int AbstractTransformation<PointT>::desPtNum;
//template <typename PointT> int AbstractTransformation<PointT>::nConfig;
template <typename PointT> const int AbstractTransformation<PointT>::failure_config;
//template <typename PointT> cv::Mat_<float> AbstractTransformation<PointT>::transformationTemplate;

template <typename PointT>
void AbstractTransformation<PointT>::printAlert() const
{
	cout<<"Abstract Transformation cannot be used !"<<endl;
	assert(true);
}

}


#endif /* TRANSFORMATION_HPP_ */
