/*
 * lgcpointset.hpp
 *
 *  Created on: Jul 20, 2015
 *      Author: liming
 */

#ifndef MATCHINGLIB_LGCLIB_LGCPOINTSET_HPP_
#define MATCHINGLIB_LGCLIB_LGCPOINTSET_HPP_


#include "basictype.h"
#include "mathfuns.h"
#include "datastructure.hpp"
#include "transformation.hpp"

//CGAL head
#if (USE_3D_POINT == true)
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <cassert>
#endif

namespace lgc2
{

class Duplicated_point_exception : public exception
{
public:
	 virtual const char* what() const throw()
	  {
	    return "Two points have the same coordinates";
	  }
};

template <typename PointT>
class LGCPointSet
{
public:
    vector< BasicPointDesc_<PointT> > normalizedPtsdescarray;		  /**< normalized point descriptions*/
    int k_Neighbors;
    pcl::KdTreeFLANN<PointT> kdTree;
    cv::Mat_<float> transform_matrix_to_Normalized_PointSets;

protected:
    void constructKDTrees();
    void findNearestNeighbors();

private:
    void findMeshNeighbors(LGCPointSet<pcl::PointXY>* type);
    void normalize(LGCPointSet<pcl::PointXY>* type);
	float calculate_InterPointDistance(LGCPointSet<pcl::PointXY>* type);

#if (USE_3D_POINT == true)
    void findMeshNeighbors(LGCPointSet<pcl::PointXYZ>* type);
    void normalize(LGCPointSet<pcl::PointXYZ>* type);
    float calculate_InterPointDistance(LGCPointSet<pcl::PointXYZ>* type);
#endif


public:
    virtual ~LGCPointSet() {};

    /**
     * Set point set:
     * - resize normalizedPtsdescarray and fill with point postions
     * - normalize the point set
     * - construct kdtree
     * - find nearest neigbhors
     */
    bool setPointSet(vector<PointT> const *ptsource, int kNeighbors);

    void findMeshNeighbors();
    void normalize();
    float calculate_InterPointDistance();

    /**
     * Clear matched scene points:
     * This function is used for multimodel presenting in the same scene
     */
    bool removeMatchedPoints();

	/**
	 * Clear matching information
	 */
	void clearResult();

	void getOrignalPointSet(vector< pcl::PointXY > &ptarray);
	void getOrignalPointSet(vector< pcl::PointXYZ > &ptarray);

	void printInfo_PointArrayPlus(ostream &outstream);
};


}



#endif /* MATCHINGLIB_LGCLIB_LGCPOINTSET_HPP_ */
