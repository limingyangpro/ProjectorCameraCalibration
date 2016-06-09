/*
 * skeleton.cpp
 *
 *  Created on: Apr 10, 2015
 *      Author: liming
 */

#include "skeleton.hpp"
const int s_lut[256] =
		{0,0,0,1,0,0,1,3,0,0,3,1,1,0,1,3,0,0,0,0,0,0,0,0,2,0,2,0,3,0,3,3,
		   0,0,0,0,0,0,0,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,0,0,3,0,2,2,
		   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		   2,0,0,0,0,0,0,0,2,0,0,0,2,0,0,0,3,0,0,0,0,0,0,0,3,0,0,0,3,0,2,0,
		   0,0,3,1,0,0,1,3,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
		   3,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		   2,3,1,3,0,0,1,3,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		   2,3,0,1,0,0,0,1,0,0,0,0,0,0,0,0,3,3,0,1,0,0,0,0,2,2,0,0,2,0,0,0};

int thinning(cv::Mat &roadnetwork, int iter)
{
	int rownum = roadnetwork.rows;
	int colnum = roadnetwork.cols;
	int pixelsRemoved = 0;
	int l,c;

	cv::Mat orignal = roadnetwork.clone();
   for (l = 1; l < rownum - 1; l++)
   {
      for (c = 1; c <colnum - 1; c++)
      {
         if (orignal.at<uchar>(l,c) == 255) // White
         {
            // Get neighbours
            unsigned char index = 0;
            if (orignal.at<uchar>(l-1,c-1) == 255) index |= 1; // p1
            if (orignal.at<uchar>(l-1,c  ) == 255) index |= 2; // p2
            if (orignal.at<uchar>(l-1,c+1) == 255) index |= 4; // p3
            if (orignal.at<uchar>(l  ,c+1) == 255) index |= 8; // p6
            if (orignal.at<uchar>(l+1,c+1) == 255) index |= 16; // p9
            if (orignal.at<uchar>(l+1,c  ) == 255) index |= 32; // p8
            if (orignal.at<uchar>(l+1,c-1) == 255) index |= 64; // p7
            if (orignal.at<uchar>(l  ,c-1) == 255) index |= 128; // p4

            int code = s_lut[index];
            if (iter == 1) // odd pass
            {
               if (code == 2 || code == 3)
               {
            	   roadnetwork.at<uchar>(l,c) = 0;
            	   ++pixelsRemoved;
               }
            }
            else // even pass
            {
               if (code == 1 || code == 3)
               {
            	   roadnetwork.at<uchar>(l,c) = 0;
            	   ++pixelsRemoved;
               }
            }
         }
      }
   }

   return pixelsRemoved;
}

void skeletonize(cv::Mat& inputoutput)
{

	cv::Mat roadnetwork = inputoutput.clone();
	int rownum = roadnetwork.rows;
	int colnum = roadnetwork.cols;

	// Border blackout
	cv::rectangle(roadnetwork, cv::Point(0,0), cv::Point(colnum-1,rownum-1), cvScalarAll(0));

	int pixelsRemoved;

	do
	{
		pixelsRemoved = thinning(roadnetwork, 0);
		pixelsRemoved += thinning(roadnetwork, 1);
	} while (pixelsRemoved > 0);

	inputoutput = roadnetwork.clone();
}

