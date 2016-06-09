/*
 * lgcmodelset.hpp
 *
 *  Created on: Jul 20, 2015
 *      Author: liming
 */

#ifndef MATCHINGLIB_LGCLIB_LGCMODELSET_HPP_
#define MATCHINGLIB_LGCLIB_LGCMODELSET_HPP_

#include "lgcpointset.hpp"

namespace lgc2
{

template <typename PointT>
class LGCModelSet : public LGCPointSet<PointT>
{
public:
    enum
    {
    	INLIER_FLAG  =  1, //! inlier
		OUTLIER_FLAG = -1, //! outlier
		NO_FLAG      =  0, //! not decided
    };

public:
    float inter_point_distance;                       //each model has a sparsity, sqrt(point_set_area/point_num)
	vector< int > correspondancemap;     /**<number of reference pt * number of image pt : refID*imgnum + imgID, flags for correspondences (INLIER_FLAG, OUTLIER_FLAG, NO_FLAG) */
	list< boost::shared_ptr< Correspondance_<PointT> > > resultCorrespList;

public:
	virtual ~LGCModelSet() {};

    /**
     * Set model point set:
     * - resize normalizedPtsdescarray and fill with point postions
     * - normalize the point set
     * - construct kdtree
     * - find nearest neigbhors
     * - find inter-point distance
     * - find mesh neigbhors
     */
	bool setPointSet(vector<PointT> const *ptsource, int kNeighbors);

	/**
	 * Mark in the correspondence map that correspondences in this list are outliers
	 */
	void markList(const list< boost::shared_ptr< Correspondance_<PointT> > >& inputlist, int imgnum, int flags);

	/**
	 * Init and clear correspondence map
	 */
	void clearCorrespondenceMap(int imgNum);


	void printInfo_correspondancemap(ostream &outstream, int imgNum);
};


}

//class LGCModelSet2D : public LGCModelSet<pcl::PointXY>, public LGCPointSet2D
//{
//protected:
//	float calculate_InterPointDistance();
//};
//
//class LGCModelSet3D : public LGCModelSet<pcl::PointXYZ>, public LGCPointSet3D
//{
//protected:
//	float calculate_InterPointDistance();
//};


#endif /* MATCHINGLIB_LGCLIB_LGCMODELSET_HPP_ */
