/*
 * data.hpp
 *
 *  Created on: Jan 18, 2015
 *      Author: ylm
 */

#ifndef DATASTRUCTURE_HPP_
#define DATASTRUCTURE_HPP_

#include "basictype.h"
#include "basicfunctions.hpp"
#include "transformation.hpp"

namespace lgc2
{

template <typename PointT>
class BoundingBox
{
public:
	PointT minPt, maxPt;

public:
//	BoundingBox() {};
//	BoundingBox(const PointT& minPt_, const PointT& maxPt_)
//	{
//		minPt = minPt_;
//		maxPt = maxPt_;
//	};
	virtual bool isInBoundingBox(const PointT& queryPt){assert(true); };
	virtual bool enlargeBoundingBox(float percentage, const BoundingBox<PointT>& limitBox){assert(true); };
	virtual bool isBiggerThanSphere(const PointT& shpereCenter, float range){assert(true); };
	virtual float getBoundingBoxVolume(){assert(true); };
};

class BoundingBox2D : public BoundingBox<pcl::PointXY>
{
public:
	pcl::PointXY minPt, maxPt;

public:
	BoundingBox2D();
	BoundingBox2D(const pcl::PointXY& minPt_, const pcl::PointXY& maxPt_);
	virtual bool isInBoundingBox(const pcl::PointXY& queryPt);
	virtual bool enlargeBoundingBox(float percentage, const BoundingBox<pcl::PointXY>& limitBox);
	virtual bool isBiggerThanSphere(const pcl::PointXY& shpereCenter, float range);
	virtual float getBoundingBoxVolume();
};

class BoundingBox3D : public BoundingBox<pcl::PointXYZ>
{
public:
	pcl::PointXYZ minPt, maxPt;

public:
	BoundingBox3D();
	BoundingBox3D(const pcl::PointXYZ& minPt_, const pcl::PointXYZ& maxPt_);
	virtual bool isInBoundingBox(const pcl::PointXYZ& queryPt);
	virtual bool enlargeBoundingBox(float percentage, const BoundingBox<pcl::PointXYZ>& limitBox);
	virtual bool isBiggerThanSphere(const pcl::PointXYZ& shpereCenter, float range);
	virtual float getBoundingBoxVolume();
};

/**
 * Parameters
 */
class Parameter
{
public:
	//For general use
	int kr_neighbor;           /**<Number of neighbors we choose to use, must be bigger than 3, total neighbors to find is kr_neighbor+param_m*/
	int ki_neighbor;           /**<Number of neighbors that are considered too near to use */
	float eta;                 /**<Jitter factor */
	float jitter;

	//For generator use
	int digitNeighbor;         /**<Number of digits to encode neighbor ID, for (model, basis) construction */
	int digitRefPatch;         /**<Number of digits to encode "model ID", for (model, basis) construction */
	float indexMax;              /**<For hash table, the max of index it can record */
	int binNumInUnit;          /**<Number of bins for each unit index, Resolution of each hash bin */
	int hypothese_threshold;   /**<Threshold of vote for a valide hypothese */
	float sigmaMax;            /**<Max threshold for incertitude of index */

	int Nmax;                  /**< Number of trial of correspondences */
	int Nlarge;                /**< Number of supporters to validate an hypotheses */
	//For correspondence dist (quality)
	int weakCondition;         /**<Corresp dist more than weakCondtion is consider more likely to be outliers */
	int strongConditon;        /**<Corresp dist less than strongCondtion is consider more likely to be inliers */
public:
	Parameter();
	Parameter(int kr, int ki, float eta_, int digitNei, int digitRef, float entryMax_, int binNumInUnit_, int hypothese_threshold_, float sigmaMax_, int Nmax_, int Nlarge_, int wc, int sc);
	void setParam(int kr, int ki, float eta_, int digitNei, int digitRef, float entryMax_, int binNumInUnit_, int hypothese_threshold_, float sigmaMax_, int Nmax_, int Nlarge_,int wc, int sc);
	void setSparsity(float sparsity);
	float getTolerance1(float alpha) const;
	float getTolerance2(float alpha) const;
	void printInfo(ostream &outstream) const;
};

/**
 * This class represents points
 */
template <typename PointT>
class BasicPointDesc_
{
public:
	int id;             /**< Point id, the place in the array */
	PointT pos;         /**< The point coordinate */

	BasicPointDesc_<PointT> *correspondpoint;    /**< The point correspond to this point in other side (image/reference) */
	                                             /**< Used in decouplage in "getSupporters" and "refiner" */
	vector< BasicPointDesc_<PointT>* > neighbors;    /**< N neighbors for the point */
	vector< BasicPointDesc_<PointT>* > mesh_neighbors;   /**< Mesh neighbors for expansion */
	BasicPointDesc_(){correspondpoint = NULL;}
	void printInfo(ostream &outstream);
	void drawNeighbors();
};

/**
 * This class contains so-called localvotingtable, records the correspondences between two subsets
 */
template <typename PointT>
class Correspondance_
{
//parameters can not be static since other application may use different correspondence settings
protected:
	const AbstractTransformation<PointT> *localTransform;
	const Parameter* param;

protected:
	cv::Mat_<float> lTransformation;

public:
	BasicPointDesc_<PointT> *refkeypt;
	BasicPointDesc_<PointT> *imgkeypt;

	/**
	 * Record the local correspondances, that is, the result of maximum assignement of localCorrespTable
	 */
	list< pair<short, short> > bestLocalCorresp;

protected:
	/**
	 * Find estimated correspondences using intial local transformation
	 * NB : DO NOT Need filtered local votes or descriptors, that is level = 0
	 */
	void findCorrespondencesHerist(const Correspondance_<PointT>* neighbor_c, vector<short> *resultptr);

public:
	Correspondance_();
	Correspondance_(const Correspondance_<PointT> &p);
	Correspondance_(const Parameter* param_, const AbstractTransformation<PointT> *localTransform_,  BasicPointDesc_<PointT> *refpt, BasicPointDesc_<PointT> *imgpt);
	~Correspondance_();

	cv::Mat_<float> getLocalTransformation() const;

	//Find the matching given an initial guess.
	//Return true if the matching is found
	bool euristicMatch(const cv::Mat_<float> &initalGuess_ord,  const vector< int >* correspdist, int imgNum);

	//Find more correspondences in this dual-patch using known correspondences
	void improveMatching();

	//Find correspondence with intial guess of transformation
	void solveWithIntialGuess(Correspondance_<PointT>& other);

	//Check if this correspondence is in coherency with another correspondence
	bool isInCoherencyWith(Correspondance_<PointT>& neighbor_corresp);

	/**
	 * out put for Debug use
	 */
	void printInfo(ostream &outstream) const;
	void draw() const;     //draw correspondences
};

/**
 * Correspondence with local voting
 */

template <typename PointT>
class CorrespondanceWithVoting_ : public Correspondance_<PointT>
{
protected:
	cv::Mat_<short> localCorrespTable;
public:
	int modelID;                           //records which modelID it belongs to

public:
	void init(const Parameter *p, const AbstractTransformation<PointT> *localTransform_, int refModelID, BasicPointDesc_<PointT> *refpt, BasicPointDesc_<PointT> *imgpt);

	//Reset : clean all records
	void resetToImgPt(BasicPointDesc_<PointT> *imgkeypt_);
	void vote(const HashTableElem &refElem, const HashTableElem &imgElem);
	bool threshold( int threhold);

	bool localMatching();
	void printInfo(ostream &outstream) const;
};

template <typename PointT>
class CorrespondanceArray
{
private:
	const Parameter *param;                     //need ki,kr and localTransform in param
//	const vector< BasicPointDesc_<PointT> > *refPtVector;   //need its size and each element's adress
	const BasicPointDesc_<PointT> *imgkeypt;                      //the image point for which it represents
	const AbstractTransformation<PointT> *localTransform;

	vector< CorrespondanceWithVoting_<PointT> > metadata;    			//the correspondences array
	vector< int > modelOffset;                              //record the offset for each referece model
	vector<int> voteSum;                      				//the vote sum for each correspondences
	vector<bool> isClassified;								//record if the correspondence is already classified (as inlier or outlier)
	multimap<int, CorrespondanceWithVoting_<PointT>*> orderedRefID;                        //the ordered reference ID in the favorable matching order <voteSum, RefID>
	typename multimap<int, CorrespondanceWithVoting_<PointT>*>::reverse_iterator currentPosition;     //current position of the list

public:
	/**
	 * Initialization with parameter and local transformation
	 */
	void init(const Parameter *param_, const AbstractTransformation<PointT> *localTransform_);

	/**
	 * Reset all information, return to the state just after the default constructor
	 */
	void reset();

	/**
	 * Add one reference model to correspondence array ( list of reference points)
	 */
	void addOneModel(vector< BasicPointDesc_<PointT> >* refPts, int modelID);

	/**
	 * Reset the correspondence list : return to the state just after init
	 * vectors are resized but empty
	 */
	void resetToImgPt(BasicPointDesc_<PointT> *imgkeypt);

	/**
	 * Vote for all the reference points
	 */
	void voteForImgPt(int modelID, const HashTableElem &refElem, const HashTableElem &imgElem);

	/**
	 * order the reference point ID into the favorable matching order
	 */
	void sort(int c_vote);

	/**
	 * Check if there's any correspondence to be processed
	 */
	bool CurrentIsNull();

	/**
	 * Next corresondence: get current correspondence and move ++currentPosition
	 */
	CorrespondanceWithVoting_<PointT>* getCurrent();


public:
	CorrespondanceArray();
	~CorrespondanceArray();

	/**
	 * out put for Debug use
	 */
	void printInfo(ostream &outstream) const;
	void draw() const;     //draw correspondences
};

template <typename PointT>
class CorrespondenceList
{
protected:
	list< boost::shared_ptr< Correspondance_<PointT> > > correspList;
	const AbstractTransformation<PointT> *globalTransform;
	const Parameter* param;
public:
	CorrespondenceList() : param(NULL), globalTransform(NULL)
	{
		correspList.clear();
	};
	CorrespondenceList(Parameter* parameter, AbstractTransformation<PointT> *globalTrans) : param(parameter), globalTransform(globalTrans)
	{
		correspList.clear();
	};

	/**
	 * Make sure that between the reference points and image points, we have one-one match.
	 */
	void decouplage(const cv::Mat_<float> bestInverseTransform);

	/**
	 * Use the correspondence list to estimate a global transformation (img->ref), and elimate the bad corresopndences
	 */
	cv::Mat_<float> estimateGlobalInvTransformation(float threshold = -1);
	cv::Mat_<float> estimateGlobalInvTransformation(const list< boost::shared_ptr< Correspondance_<PointT> > > &slave);

	/**
	 * Merge current corresponding list with another one, to find a common global transformation
	 */
	void mergeWith(list< boost::shared_ptr< Correspondance_<PointT> > >& otherlist);

	/**
	 *
	 */
	void printInfo(ostream &outstream) const;
};


}


#endif /* DATASTRUCTURE_HPP_ */
