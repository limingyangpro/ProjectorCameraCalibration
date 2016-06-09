/*
 * refiner.hpp
 *
 *  Created on: Aug 25, 2015
 *      Author: liming
 */

#ifndef MATCHINGLIB_LGCLIB_REFINER_HPP_
#define MATCHINGLIB_LGCLIB_REFINER_HPP_

#include "basictype.h"
#include "datastructure.hpp"
#include "lgcpointset.hpp"
#include "lgcmodelset.hpp"
#include "transformation.hpp"

namespace lgc2
{

template <typename PointT>
class Refiner : public CorrespondenceList<PointT>
{
private:
	boost::shared_ptr< LGCPointSet<PointT> > scenePointSetPtr;
	boost::shared_ptr< LGCModelSet<PointT> > modelPointSetPtr;
	const AbstractTransformation<PointT> *localTransform;
	bool stopAlgorithm;
    float final_jitter; 

	/**
	 * For 2D:
	 */
	void computeMatchingRegion(const list< boost::shared_ptr< Correspondance_<pcl::PointXY> > > & matchedPtsList);
	float getIncertitude(const pcl::PointXY& targetPt);
	cv::RotatedRect matchedRegion;

	/**
	 * For 3D:
	 */
	void computeMatchingRegion(const list< boost::shared_ptr< Correspondance_<pcl::PointXYZ> > > & matchedPtsList);
	float getIncertitude(const pcl::PointXYZ& targetPt);
	pcl::PointXYZ matchingCenter;                                 //The matching box is defined by a matching center and a range
	vector<pcl::PointXYZ> matchingRegionConvexeHull;

public:
	Refiner()
	: CorrespondenceList<PointT>(),
	  scenePointSetPtr(boost::shared_ptr< LGCPointSet<PointT> >()),
	  modelPointSetPtr(boost::shared_ptr< LGCModelSet<PointT> >()),
	  localTransform(NULL),
	  stopAlgorithm(false),
      final_jitter(0.01)
	{

	}

	Refiner(Parameter *param_, AbstractTransformation<PointT> *local_, AbstractTransformation<PointT> *global_, float final_eta = -1)
	: CorrespondenceList<PointT>(param_, global_),
	  scenePointSetPtr(boost::shared_ptr< LGCPointSet<PointT> >()),
	  modelPointSetPtr(boost::shared_ptr< LGCModelSet<PointT> >()),
	  localTransform(local_),
	  stopAlgorithm(false),
      final_jitter(param_->jitter/param_->eta*final_eta) 
	{

	}

	void setPointSets(boost::shared_ptr< LGCModelSet<PointT> > modelPtset, boost::shared_ptr< LGCPointSet<PointT> > scenePtset);

	void initialize(list< boost::shared_ptr< Correspondance_<PointT> > >& result);
	bool runOneStep();
    
	void expandResultList(list< boost::shared_ptr< Correspondance_<PointT> > >& result);
    void refineResultWithFinalEta(); 
	void getResultList(list< boost::shared_ptr< Correspondance_<PointT> > >& result);

	//Derived from template class, use "using" key words
	using CorrespondenceList<PointT>::correspList;
	using CorrespondenceList<PointT>::param;
	using CorrespondenceList<PointT>::globalTransform;
	using CorrespondenceList<PointT>::decouplage;
	using CorrespondenceList<PointT>::estimateGlobalInvTransformation;
	using CorrespondenceList<PointT>::mergeWith;
	using CorrespondenceList<PointT>::printInfo;
};


}



#endif /* MATCHINGLIB_LGCLIB_REFINER_HPP_ */
