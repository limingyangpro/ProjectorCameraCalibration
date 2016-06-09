/*
 * lgcmethod.hpp
 *
 *  Created on: Jan 18, 2015
 *      Author: ylm
 */

#ifndef LGCMETHOD_HPP_
#define LGCMETHOD_HPP_

#include "basictype.h"
#include "mathfuns.h"
#include "chronometer.h"
#include "datastructure.hpp"
#include "generator.hpp"
#include "generator2Daff.hpp"
#include "generator3Dsim.hpp"
#include "transformation.hpp"

#include "lgcmodelset.hpp"
#include "lgcpointset.hpp"
#include "refiner.hpp"
#include "validator.hpp"

namespace lgc2
{
template <typename PointT>
class LGCmethod
{
public:

protected:
    Parameter* param;

	vector< boost::shared_ptr<LGCModelSet<PointT> > > modelPointSetVector;
	boost::shared_ptr< LGCPointSet<PointT> > scenePointSet;

	vector< pair<BasicPointDesc_<PointT>*,const int*> > imgptRandomOrderPtr;   /**< Image points, in the order of trial sequence */

	AbstractGenerator<PointT>* generatorPtr;

	AbstractTransformation<PointT> *localTransform, *globalTransform;

	int currentModelID;

	bool generateSameRandomSeed;              /**< If true, 0 is used for srand, so random number has the same sequence in each run, see setCorrespondenceDist */

public:

//	vector< boost::shared_ptr< Correspondance_<PointT> > > matchingresult;               /**This records the best match we found */
	list< boost::shared_ptr< Correspondance_<PointT> > > resultList;
	cv::Mat_<float> homo_result;

protected:

	/**
	 * Before each matching, we have to intial correspondence map, etc
	 */
//	void initialMatching();

	/**
	 * Get result from matching result list
	 */
	cv::Mat_<float> getFinalResult(vector< pair<int, pair<int,int> > > *oCorrespondenceResultPtr);

	/**
	 * Clear correspondencemap
	 */
	void clearCorrespondancemap();

	/**
	 * Clear corresponding point and matching result
	 */
	void clearResult();

	/**
	 * Run Generator - validator loop
	 */
	void run_Generator_Validator();

	/**
	 * Run Refiner
	 */
	void run_Refiner();

	/**
	 * Find one model
	 * Before calling this function, please make sure scenePointSet is not null,
	 * that is, setImgPoints function has been called
	 */
	cv::Mat_<float> findOneModel(int& refmodelID, vector< pair<int, pair<int,int> > > *oCorrespondenceResultPtr);

public:
	//constructor
	LGCmethod();
	LGCmethod(Parameter *param_, AbstractGenerator<PointT>* generatorptr, AbstractTransformation<PointT> *local_, AbstractTransformation<PointT> *global_);

	//destructor
	~LGCmethod();

	/**
	 * This function clear all matching information, but leave all reference information untouched (hashtable, reference point information )
	 */
	void clearMatchingInformation();

	/**
	 * Clear all models
	 */
	void clearModels();

	/**
	 * Set reference point sets and renew the hashtables
	 */
	void setRefPoints(vector<PointT> const *ptsource);

	/**
	 * Set reference point sets
	 */
	bool setImgPoints(vector<PointT> const *ptsource);

	/**
	 * Set "color distance" for each correspondence
	 */
	bool setCorrespondenceDist(const vector< int >* correspdist);

	/**
	 * Find one model in the scene
	 */
	cv::Mat_<float> setImgPointsAndFindOneModel(vector<PointT> const *ptsource, const vector< int >* correspdist, int& refmodelID, vector< pair<int, pair<int,int> > > *oCorrespondenceResultPtr);

	/**
	 * Find multi-models in the scene
	 */
	bool setImgPointsAndFindMultiModel(vector<PointT> const *ptsource, vector<int> &refmodelIDs, vector< cv::Mat_<float> > &transformations, vector< vector< pair<int, pair<int,int> > > >  *oCorrespondenceResultPtr);

	void printInfo_PointArrayPlus(ostream &outstream);
	void printInfo_List(ostream &outstream, const list< boost::shared_ptr< Correspondance_<PointT> > >& matchedPtsList);
	void printInfo_MatchingResult(ostream &outstream);
	void printInfo_HashTable(ostream &outstream);
	void draw(list< Correspondance_<PointT>* > &srcList, cv::Mat& transform) const;
};


}

#endif /* LGCMETHOD_HPP_ */
