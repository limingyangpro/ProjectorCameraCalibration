/*
 * generator.hpp
 *
 *
 * Hypotheses generator for 2D affine
 *
 *  Created on: Jan 18, 2015
 *      Author: ylm
 */

#ifndef GENERATOR2DAFF_HPP_
#define GENERATOR2DAFF_HPP_

#include "basictype.h"
#include "mathfuns.h"
#include "datastructure.hpp"
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include "abstractgenerator.hpp"
#include "hashtableint.hpp"


namespace lgc2
{

/**
 * Geometry parameters associate for each neighbor
 */
struct LocalIDForPt2D
{
	int neighborID;
	float surface;
	pcl::PointXY pos;
	RDM_Point descriptor;
};

class Generator2Daff : public AbstractGenerator<pcl::PointXY>
{
private:
	vector< boost::shared_ptr< LGCModelSet<pcl::PointXY> > > *prefmodelptrarray;
	const Parameter* param;
	const AbstractTransformation<pcl::PointXY> *localTransform;

	HashTable hashtable;
	int c_vote;

public:
	CorrespondanceArray<pcl::PointXY> correspArray;           /**< Correspondence for each reference point in each reference model, summered into one */

public:
	static void getOneEntry(LocalIDForPt2D &ptX, LocalIDForPt2D &ptY, LocalIDForPt2D &ptD, Descriptor &desc, cv::Mat_<float> &invA);
	void getDescriptorForFirstPt(const pcl::PointXY &basePt, const vector<pcl::PointXY> &vertexs, float threshold, list<Descriptor>& descList, list< boost::shared_array<float> >* tolList);
public:

	//base index and canal basis converts
	void index2Basis(const unsigned long long &index, int& modelID, int& submodelID, int& ptID1, int& ptID2, int& ptID3);
	void basis2Index(const int& modelID, const int& submodelID, const int& ptID1, const int& ptID2, int& ptID3, unsigned long long &index);

	/**
	 * Check if the hash entry is inside [-indexMax..indexMax]*[-indexMax..indexMax]
	 */
	bool isHashentryValid(RDM_Point hEntry);

	/**
	 * Register reference model,basis into hashtable
	 */
	void registerInHashTable(unsigned long long idx, int tableID, RDM_Point hashKey, float tolx, float toly);

	//Verified the base: reference base point presented in index form and image base point presented in combinaision form
	bool verifyModel(boost::shared_ptr< Correspondance_<pcl::PointXY> > corresp);

	//Find affine transformation between two subsets
	//Return 2*3 matrix
	cv::Mat_<float> findTransformLS(const PtArray &fromsubset, const PtArray &tosubset);

	//Transform inputset to outputset with transform
	void transform(const PtArray &inputset, PtArray &outputset, const cv::Mat_<float>& trans);



public:
	Generator2Daff();
	Generator2Daff (Parameter* param_, AbstractTransformation<pcl::PointXY> *localTransform);

	/**
	 * Treat one reference patch as one small model and register into the hashtable
	 */
	void setRefPatch(BasicPointDesc_<pcl::PointXY> &refPt, int modelID);
	/**
	 * Create the full hashtable from reference points
	 */
	void setRefPatches(vector< boost::shared_ptr< LGCModelSet<pcl::PointXY> > > *prefmodelsptr, int modelID = -1);
	void clearMatchingInformation();
	void setImgPatch(BasicPointDesc_<pcl::PointXY>* imgPt_);

	/**
	 * Reset generator to its intial state,
	 * 	1) prefmodelptrarray = NULL;
	 *  2) Empty the hashtable but conserving its size
	 *  3) correspArray.reset(), totally cleanup information in correspArray
	 */
	void reset();

	/*
	 * Generate corrspondence hypothese, if no hypothese, returns NULL
	 */
	boost::shared_ptr< Correspondance_<pcl::PointXY> > generate(const vector< int >* correspdist, int imgNum, int& modelID);

	/**
	 * Debug Use
	 */
	void printHashtable(ostream &outstream);
	void statistics(BasicPointDesc_<pcl::PointXY>* imgPt_, ostream &outstream);
};

}

#endif /* GENERATOR2DAFF_HPP_ */
