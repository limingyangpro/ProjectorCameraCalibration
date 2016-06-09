/*
 * basictype.h
 *
 *  Created on: 30 oct. 2013
 *      Author: lyang
 */

#ifndef BASICTYPE_H_
#define BASICTYPE_H_


#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#ifdef __APPLE__
#include "OpenGL/glu.h"
#include "OpenGL/gl.h"
#include "GLUT/glut.h"
#else
#include "GL/glu.h"
#include "GL/gl.h"
#include "GL/glut.h"
#endif

#include <sys/time.h>          //for getting time in ms
#include <pcl/point_cloud.h>              //for neighbor search
#include <pcl/kdtree/kdtree_flann.h>    //for neighbor search

#include <fstream>        //for debug output
#include <vector>
#include <list>
#include <boost/utility.hpp>
#include <set>
#include <bitset>        //for binary output
using namespace std;
//using namespace cv;

//typedef double	RDM_DOUBLE ;
//typedef float		RDM_FLOAT;
//typedef int 		RDM_INT;

#define RDM_NA		-1        //Not a number
//
//class RDM_Point
//{
//public:
//	float x,y;
//
//	//Constructor
//	RDM_Point();
//	RDM_Point(float x_, float y_);
//	RDM_Point(const cv::Point_<float>& pt);
//
//	//Type convertor
//	RDM_Point& operator = (const RDM_Point& pt);
//	RDM_Point& operator = (const cv::Point2f& pt);
//	RDM_Point& operator += (const RDM_Point& pt);
//	operator cv::Point2f() const;
//	operator pcl::PointXY() const;
//
//	//Print
//	friend std::ostream& operator<<(std::ostream& os, const RDM_Point& obj);
//};
//
//class RDM_Point3D
//{
//public:
//	float x,y,z;
//
//	//Constructor
//	RDM_Point3D();
//	RDM_Point3D(float x_, float y_, float z_);
//
//	//Type convertor
//	operator cv::Point3f() const;
//	operator pcl::PointXYZ() const;
//
//
//};

/**
 * For interface use
 */
typedef cv::Point2f Point2D;
typedef cv::Point3f Point3D;

typedef cv::Point2f RDM_Point;
typedef cv::Point3f RDM_Point3D;
typedef vector<RDM_Point> PtArray;

//For sort from big to small
struct classcompB2S {
  bool operator() (const char& lhs, const char& rhs) const
  {return lhs>rhs;}
};

/**
 * A special class for generating random seed:
 * Why ?
 * Sometimes, program is too fast, it can run serveral time during one second, so, useless to use time(0)/time(NULL),
 * which get gives us the same random sequence
 * Here, we use current time in miliseconds
 */
class RandomSeedGenerator
{
public:
static unsigned int generateSeed();
};

#if (DEBUG == 1)
	extern ofstream debugoutput;
#endif


#endif /* BASICTYPE_H_ */
