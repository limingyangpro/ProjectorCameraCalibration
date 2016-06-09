/*
 * Generate random point pattern.cpp
 *
 *  Created on: 05 Mar. 2014
 *      Author: lyang
 */
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <unistd.h>
#include <time.h>       /* time */
#include <fstream>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
using namespace std;

//#include "basictype.h"

using namespace std;

void generateAndPushBack(int ptnum, cv::Point pos, int width, int height, float dist, vector<cv::Point2f>& points)
{
	//generate
	points.resize(ptnum);
	for (unsigned int i = 0 ; i < ptnum ; i++)
	{
		//Generate non overlap point sets
		cout<<i<<endl;
		bool ok = false;
		while (!ok)
		{
			points[i].x = rand() % width + pos.x;
			points[i].y = rand() % height + pos.y;
			cout<<points[i]<<endl;
			ok = true;
			for (unsigned int j = 0 ; j < i ; j++)
			{
				if (norm(points[i] - points[j]) < dist)
				{
					ok = false;
					break;
				}
			}
		}
	}
}

int main(int argc, char **argv)
{
	float tmp1,tmp2,tmp3,tmp4;
	bool drawingmode = false;
	cout<<sizeof(unsigned char)<<endl;
	char oc;
	int width,height,ptnum;
	int paperType;
	float ptsize;
	string referencefilename,imagefilename;
	int white_boarder_size = 0;
	cout<<"------------------------Functionality--------------------------"<<endl
			<<"1 - Draw random point pattern and write to \".txt\" file." <<endl
			<<"2 - Draw random point patter from \".txt\" file."<<endl
			<<"--------------------------------------------------------------"<<endl;
	while((oc = getopt(argc, argv, "?u:w:h:n:s:i:r:dA:B:")) != -1)
	{
		switch (oc)
		{
		case '?':
			 	cout<< "For Example 1 (generate mode): pointpatterngen -A4 -n200 -s16.0 -i testimage.jpg  -r testref.txt"<<endl
			 	 << endl;
			return 0;
		case 'w':
			width = atoi(optarg);
			break;
		case 'h':
			height = atoi(optarg);
			break;
		case 'A':
			paperType = atoi(optarg);
			switch (paperType)
			{
			case 4:
				height = 2100;
				width = 2970;

				break;
			case 3:
				height = 2970;
				width = 4200;

				break;
			case 2:
				height = 4200;
				width = 5940;

				break;
			};
			break;
		case 'B':
			paperType = atoi(optarg);
			switch (paperType)
			{
			case 4:
				height = 2500;
				width = 3530;
				break;
			}
			break;
		case 'n':
			ptnum = atoi(optarg);
			break;
		case 's':
			ptsize = atof(optarg);
			break;
		case 'r':
			referencefilename = optarg;
			break;
		case 'i':
			imagefilename = optarg;
			break;
		case 'd':
			drawingmode = true;
			break;
		default :
			cout<<"Incorrect Argument!! "<<endl;
			return 0;
		}
	}

	int margin = white_boarder_size + int(ptsize);
	assert(referencefilename.size() > 0);
	assert(imagefilename.size() > 0);

	fstream referencefile;
	if (!drawingmode)            //write to txt file
	{
		assert(ptsize > 0);
		referencefile.open(referencefilename.c_str(),std::fstream::out);
		if(!referencefile.is_open())
		{
			cout<<"Can't open output file correctly !"<<endl;
			return 0;
		}
		referencefile<<"0 0 "<<width<<" "<<height<<endl;
		referencefile<<ptnum<<endl;
	}
	else                     //read from txt file
	{
		referencefile.open(referencefilename.c_str(),std::fstream::in);
		if(!referencefile.is_open())
		{
			cout<<"Can't open input file correctly !"<<endl;
			return 0;
		}
		referencefile>>tmp1>>tmp2>>tmp3>>tmp4;
		referencefile>>ptnum;
		width = int(tmp3);
		height = int(tmp4);

		vector<cv::Point2f> pts;
		pts.resize(ptnum);
		cv::Mat img = cv::Mat::zeros(height,width,CV_8U);

		for (unsigned int i = 0 ; i < ptnum ; i++)
		{
			referencefile>>pts[i].x>>pts[i].y;
			cv::circle(img, pts[i], ptsize, cv::Scalar(255), -1);
		}

		referencefile.close();


		//cout<<img<<endl;
//		img = 255 - img;
		imwrite(imagefilename, img);
		exit(0);
	}

	assert(width > 0);
	assert(height > 0);
	assert(ptnum > 0);
	vector<cv::Point2f> pts;

	if (!drawingmode)
	{
		//Generate the points in 3*3 subregions
		srand (time(NULL));
		int subregion_width = (width-margin*2)/3;
		int subregion_height = (height-margin*2)/3;
		int sub_ptnum = ptnum/9;

		cv::Point subregionPos;
		pts.clear();
		generateAndPushBack(ptnum, cv::Point2f(margin,margin), (width-margin*2), (height-margin*2), ptsize*8, pts);
//		for (int r = 0 ; r < 3 ; r++)
//			for (int c = 0 ; c < 3 ; c++)
//			{
//				subregionPos.x = c*subregion_width + margin;
//				subregionPos.y = r*subregion_height + margin;
//
//				generateAndPushBack(sub_ptnum, subregionPos, subregion_width, subregion_height, ptsize, pts);
//			}
//		cout<<pts.size()<<endl;

		//Get the final missing points
//		cv::Point2f pt;
//		for (unsigned int i = pts.size() ; i < ptnum ; i++)
//		{
//			pt.x = rand() % (subregion_width*3) + margin;
//			pt.y = rand() % (subregion_height*3) + margin;
//			pts.push_back(pt);
//		}

	}
	else
	{
		pts.resize(ptnum);
	}

	//Read or record the points, and draw them
	cv::Mat img = cv::Mat::zeros(height,width,CV_8U);
	int rx,ry;

	//Top left as origne
	int halfNum = ptnum/2;
	for (unsigned int i = 0 ; i < ptnum ; i++)
	{
		if (!drawingmode)
		{
			referencefile<<pts[i].x<<" "<<pts[i].y<<endl;
		}
		else
		{
			referencefile>>pts[i].x>>pts[i].y;
		}

//		if (i >= halfNum)  //board pattern is the first half
		{

			for (rx = pts[i].x - ptsize ; rx <= pts[i].x + ptsize ; rx++)
				for (ry = pts[i].y - ptsize ; ry <= pts[i].y + ptsize ; ry++)
					if ( (rx - pts[i].x)*(rx - pts[i].x)+(ry - pts[i].y)*(ry - pts[i].y)<=ptsize*ptsize)
						{
	//						img.at<unsigned char>(height - ry,rx) = 255;  //Left low corner is the orignal
							img.at<unsigned char>(ry,rx) = 255;
						}
		}
	}

	referencefile.close();
	//cout<<img<<endl;
	img = 255 - img;
	imwrite(imagefilename, img);
}
