/*
 * basicfunctions.hpp
 *
 *  Created on: Sep 4, 2015
 *      Author: liming
 */

#ifndef MATCHINGLIB_LGCLIB_BASICFUNCTIONS_HPP_
#define MATCHINGLIB_LGCLIB_BASICFUNCTIONS_HPP_

#include "basictype.h"
#include "mathfuns.h"

namespace lgc2
{
inline float pointDistSqr(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2)
{
	return MathFuns::pointDistanceSqr3D(p1,p2);
}

inline float pointDistSqr(const pcl::PointXY &p1, const pcl::PointXY &p2)
{
	return MathFuns::pointDistance2(p1,p2);
}

}



#endif /* MATCHINGLIB_LGCLIB_BASICFUNCTIONS_HPP_ */
