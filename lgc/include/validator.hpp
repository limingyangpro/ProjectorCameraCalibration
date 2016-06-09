/*
 * validator.hpp
 *
 *  Created on: Sep 4, 2015
 *      Author: liming
 */

#ifndef MATCHINGLIB_LGCLIB_VALIDATOR_HPP_
#define MATCHINGLIB_LGCLIB_VALIDATOR_HPP_

#include "basictype.h"
#include "datastructure.hpp"
#include "../include/lgcpointset.hpp"
#include "../include/lgcmodelset.hpp"
#include "transformation.hpp"

namespace lgc2
{

template <typename PointT>
class Validator : public CorrespondenceList<PointT>
{
private:
	int bestsubregionptnum;
	boost::shared_ptr< LGCPointSet<PointT> > scenePointSetPtr;
	boost::shared_ptr< LGCModelSet<PointT> > modelPointSetPtr;
	const AbstractTransformation<PointT> *localTransform;

public:
    enum
    {
    	INLIER_FLAG  =  1, //! inlier
		OUTLIER_FLAG = -1, //! outlier
		NO_FLAG      =  0, //! not decided
    };

	Validator()
	: CorrespondenceList<PointT>(),
	  bestsubregionptnum(9),
	  scenePointSetPtr(boost::shared_ptr< LGCPointSet<PointT> >()),
	  modelPointSetPtr(boost::shared_ptr< LGCModelSet<PointT> >()),
	  localTransform(NULL)
	{

	}

	Validator(Parameter *param_, AbstractTransformation<PointT> *local_, AbstractTransformation<PointT> *global_)
	: CorrespondenceList<PointT>(param_, global_),
	  bestsubregionptnum(9),
	  scenePointSetPtr(boost::shared_ptr< LGCPointSet<PointT> >()),
	  modelPointSetPtr(boost::shared_ptr< LGCModelSet<PointT> >()),
	  localTransform(local_)
	{

	}

	void setPointSets(boost::shared_ptr< LGCModelSet<PointT> > modelPtset, boost::shared_ptr< LGCPointSet<PointT> > scenePtset);

	/**
	 * Get the neighbor correspondences of current list
	 * Make sure : each correspondence in the list contains a valid localCorrespondence
	 */
	void addNeighbors(const list< boost::shared_ptr< Correspondance_<PointT> > >& ilist, list< boost::shared_ptr< Correspondance_<PointT> > >& oneighborlist);

	/**
	 * Clear correspondpoint field for correspondance list
	 */
	void clearCorrespondPointField();

	bool validate(boost::shared_ptr< Correspondance_<PointT> > corresp);

	//Derived from template class, use "using" key words
	using CorrespondenceList<PointT>::correspList;
	using CorrespondenceList<PointT>::param;
	using CorrespondenceList<PointT>::globalTransform;
	using CorrespondenceList<PointT>::decouplage;
	using CorrespondenceList<PointT>::estimateGlobalInvTransformation;
	using CorrespondenceList<PointT>::mergeWith;

};


}



#endif /* MATCHINGLIB_LGCLIB_VALIDATOR_HPP_ */
