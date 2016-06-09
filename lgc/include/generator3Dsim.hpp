/*
 * generator.hpp
 *
 *
 * Hypotheses generator for 2D affine
 *
 *  Created on: Jan 18, 2015
 *      Author: ylm
 */

#ifndef GENERATOR3DSIM_HPP_
#define GENERATOR3DSIM_HPP_

#include "basictype.h"
#include "datastructure.hpp"
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include "abstractgenerator.hpp"
#include "hashtableint.hpp"
#include "mathfuns.h"

namespace lgc2
{

/**
 * Geometry parameters associate for each neighbor
 */
struct LocalIDForPt
{
	int neighborID;
	float unitLengthSqr;
	pcl::PointXYZ pos;
	RDM_Point descriptor;
};

class Generator3Dsim : public AbstractGenerator<pcl::PointXYZ>
{
private:
	vector< boost::shared_ptr< LGCModelSet<pcl::PointXYZ> > > *prefmodelptrarray;
	const Parameter* param;
	const AbstractTransformation<pcl::PointXYZ> *localTransform;

	HashTable hashtable;
	int c_vote;

public:

	//State of the execusion
//	BasicPointDesc_<pcl::PointXYZ>* imgPt;     //in process of the imgPts
//	int ithCombin;                          //in process of ithCombinaison
//	int baseID[2];                          //ID of the point forming the basis
//	multimap<int, unsigned long long, classcompB2S> sortByVote;     //Record current vote status
//	multimap<int, unsigned long long, classcompB2S>::iterator itSortedVote;      //Record to be processed by generate()
	CorrespondanceArray<pcl::PointXYZ> correspArray;           /**< Correspondence for each reference point in each reference model, summered into one */

public:
	static void getOneEntry(LocalIDForPt &ptX, LocalIDForPt &ptY, Descriptor &desc, pcl::PointXYZ* normal);
	void getDescriptorForFirstPt(const pcl::PointXYZ &basePt, const vector<pcl::PointXYZ> &vertexs, float threshold, list<Descriptor>& descList, list< boost::shared_ptr<float> >* tol);
public:

	//base index and canal basis converts
	void index2Basis(const unsigned long long &index, int& modelID, int& submodelID, int& ptID1, int& ptID2);
	void basis2Index(const int& modelID, const int& submodelID, const int& ptID1, const int& ptID2, unsigned long long &index);

	/**
	 * Check if the hash entry is inside [-indexMax..indexMax]*[-indexMax..indexMax]
	 */
	bool isHashentryValid(RDM_Point hEntry);

	/**
	 * Register reference model,basis into hashtable
	 */
	void registerInHashTable(unsigned long long idx, int tableID, RDM_Point hashKey, float tol);

	//Verified the base: reference base point presented in index form and image base point presented in combinaision form
	bool verifyModel(boost::shared_ptr< Correspondance_<pcl::PointXY> > corresp);

	//Find affine transformation between two subsets
	//Return 2*3 matrix
	cv::Mat_<float> findTransformLS(const PtArray &fromsubset, const PtArray &tosubset);

	//Transform inputset to outputset with transform
	void transform(const PtArray &inputset, PtArray &outputset, const cv::Mat_<float>& trans);



public:
	Generator3Dsim();
	Generator3Dsim (Parameter* param_, AbstractTransformation<pcl::PointXYZ> *localTransform);

	/**
	 * Treat one reference patch as one small model and register into the hashtable
	 */
	void setRefPatch(BasicPointDesc_<pcl::PointXYZ> &refPt, int modelID);
	/**
	 * Create the full hashtable from reference points
	 */
	void setRefPatches(vector< boost::shared_ptr< LGCModelSet<pcl::PointXYZ> > > *prefmodelsptr, int modelID = -1);
	void clearMatchingInformation();
	void reset();
	void setImgPatch(BasicPointDesc_<pcl::PointXYZ>* imgPt_);

	/*
	 * Generate corrspondence hypothese, if no hypothese, returns NULL
	 */
	boost::shared_ptr< Correspondance_<pcl::PointXYZ> > generate(const vector< int >* correspdist, int imgNum, int& modelID);

	/**
	 * Debug Use
	 */
	void printHashtable(ostream &outstream);
	void statistics(BasicPointDesc_<pcl::PointXY>* imgPt_, ostream &outstream);
};

}

#endif /* GENERATOR3DSIM_HPP_ */
