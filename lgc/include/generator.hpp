/*
 * generator.hpp
 *
 *
 * Hypotheses generator for 2D affine
 *
 *  Created on: Jan 18, 2015
 *      Author: ylm
 */

#ifndef GENERATOR_HPP_
#define GENERATOR_HPP_

#include "basictype.h"
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include "hashtableint.hpp"
#include "abstractgenerator.hpp"


namespace lgc2
{

class Generator : public AbstractGenerator<pcl::PointXY>
{
private:
	vector< boost::shared_ptr< LGCModelSet<pcl::PointXY> > > *prefmodelptrarray;
	const Parameter* param;
	const AbstractTransformation<pcl::PointXY> *localTransform;

	HashTable hashtable;
	vector< vector<int> > Ckr2, Cki2;                 //Combination of neighbor points


private:

	//State of the execusion
	BasicPointDesc_<pcl::PointXY>* imgPt;     //in process of the imgPts
	int ithCombin;                          //in process of ithCombinaison
	int baseID[2];                          //ID of the point forming the basis
	multimap<int, unsigned long long, classcompB2S> sortByVote;     //Record current vote status
	multimap<int, unsigned long long, classcompB2S>::iterator itSortedVote;      //Record to be processed by generate()

public:

	//base index and canal basis converts
	void index2Basis(const unsigned long long &index, int& modelID, int& submodelID, int& ptID1, int& ptID2);
	void basis2Index(const int& modelID, const int& submodelID, const int& ptID1, const int& ptID2, unsigned long long &index);

	/**
	 * Check if the hash entry is inside [-indexMax..indexMax]*[-indexMax..indexMax]
	 */
	bool isHashentryValid(RDM_Point hEntry);

	/**
	 * Treat one reference patch as one small model and register into the hashtable
	 */
	void setRefPatch(BasicPointDesc_<pcl::PointXY> &refPt, int modelID);

	/**
	 * Register reference model,basis into hashtable
	 */
	void registerInHashTable(unsigned long long idx, int tableID, RDM_Point hashKey, RDM_Point tol);

	//Verified the base: reference base point presented in index form and image base point presented in combinaision form
	bool verifyModel(boost::shared_ptr< Correspondance_<pcl::PointXY> > corresp);

	//Find affine transformation between two subsets
	//Return 2*3 matrix
	cv::Mat_<float> findTransformLS(const PtArray &fromsubset, const PtArray &tosubset);

	//Transform inputset to outputset with transform
	void transform(const PtArray &inputset, PtArray &outputset, const cv::Mat_<float>& trans);



public:
	Generator();
	Generator(Parameter* param_, AbstractTransformation<pcl::PointXY> *localTransform);
	/**
	 * Create the full hashtable from reference points
	 */
	void setRefPatches(vector< boost::shared_ptr< LGCModelSet<pcl::PointXY> > > *prefmodelsptr, int modelID = -1);
	void clearMatchingInformation();
	void reset();
	void setImgPatch(BasicPointDesc_<pcl::PointXY>* imgPt_);

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

#endif /* GENERATOR_HPP_ */
