/*
 * abstractgenerator.hpp
 *
 *  Created on: Feb 12, 2015
 *      Author: liming
 *
 *      This file defines abstract generator (interface)
 */

#ifndef LGC2LIB_ABSTRACTGENERATOR_HPP_
#define LGC2LIB_ABSTRACTGENERATOR_HPP_

#include "basictype.h"
#include "datastructure.hpp"
#include "lgcmodelset.hpp"

namespace lgc2
{

template <typename PointT>
class AbstractGenerator
{
public:
	virtual ~AbstractGenerator() {};
	/**
	 * Create the full hashtable from reference points
	 */
	virtual void setRefPatches(vector< boost::shared_ptr< LGCModelSet<PointT> > > *prefmodelsptr, int modelID = -1) {};
	virtual void clearMatchingInformation() {};
	virtual void reset() {};       //clear all ref patch info, thus reset the generator
	virtual void setImgPatch(BasicPointDesc_<PointT>* imgPt_) {};
	virtual void registerInHashTable(unsigned long long idx, int tableID, RDM_Point hashKey, RDM_Point tol) {};
	/*
	 * Generate corrspondence hypothese, if no hypothese, returns NULL
	 */
	virtual boost::shared_ptr< Correspondance_<PointT> > generate(const vector< int >* correspdist, int imgNum, int& modelID) {};

	//Debug use
	virtual void printHashtable(ostream &outstream) {};
};

}



#endif /* LGC2LIB_ABSTRACTGENERATOR_HPP_ */
