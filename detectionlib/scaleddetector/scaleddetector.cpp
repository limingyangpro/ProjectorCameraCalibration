/*
 * scaleddetector.cpp
 *
 *  Created on: Jun 7, 2015
 *      Author: liming
 */

#include "scaleddetector.hpp"
#include <list>

ImgPatch::ImgPatch(cv::Mat_<uchar> patch)
{
	getCleanedPatch(patch);
}

void ImgPatch::setPatch(cv::Mat_<uchar> patch)
{
	getCleanedPatch(patch);
}

void ImgPatch::getCleanedPatch(cv::Mat_<uchar> patch)
{
	half_patch_size = (patch.cols - 1)/2;
	uchar fillValue = 1;
	cv::Mat mask = cv::Mat::zeros(patch.cols+2, patch.rows+2, CV_8UC1);
	cv::floodFill(patch, mask, cv::Point2f(half_patch_size,half_patch_size), cv::Scalar(255) ,0, cv::Scalar(), cv::Scalar(), 8 | cv::FLOODFILL_MASK_ONLY | (fillValue << 8));
	cleanedPatch = mask(cv::Rect(1,1,patch.cols,patch.rows)).clone();
//	cout<<cleanedPatch<<endl;
}

float ImgPatch::cornerValue()
{
	cv::Mat dx, dy;
	cv::Sobel(cleanedPatch, dx, CV_32F, 1, 0, 1);
	cv::Sobel(cleanedPatch, dy, CV_32F, 0, 1, 1);

	cv::Mat removeBorderDx = dx(cv::Rect(1, 1, dx.cols-2, dx.rows-2)).clone();
	cv::Mat removeBorderDy = dy(cv::Rect(1, 1, dy.cols-2, dy.rows-2)).clone();

	cv::Mat IX2 = removeBorderDx.mul(removeBorderDx);
	cv::Mat IY2 = removeBorderDy.mul(removeBorderDy);
	cv::Mat IXY = removeBorderDx.mul(removeBorderDy);
	cv::Mat_<float> A = cv::Mat::zeros(2,2,CV_32F);
	A[0][0] = sum(IX2).val[0];
	A[0][1] = sum(IXY).val[0];
	A[1][0] = sum(IXY).val[0];
	A[1][1] = sum(IY2).val[0];

	cv::Mat eigenA;
	eigen(A, eigenA);

	//For half_patch_size = 4, we get the number "6"
	if ((eigenA.at<float>(1,0) >= 6)&&(eigenA.at<float>(1,0)/eigenA.at<float>(0,0)>0.5))
	{
		return eigenA.at<float>(1,0);
	}
	else
	{
		return 0.0f;
	}
}

float ImgPatch::intersectionValue()
{
	int neibghours = (cleanedPatch.at<uchar>(half_patch_size-1,     half_patch_size-1))
						+ (cleanedPatch.at<uchar>(half_patch_size-1,    half_patch_size+0  ))
						+ (cleanedPatch.at<uchar>(half_patch_size-1,    half_patch_size+1))
						+ (cleanedPatch.at<uchar>(half_patch_size+0,  half_patch_size-1))
						+ (cleanedPatch.at<uchar>(half_patch_size+0,  half_patch_size+1))
						+ (cleanedPatch.at<uchar>(half_patch_size+1,  half_patch_size-1))
						+ (cleanedPatch.at<uchar>(half_patch_size+1,  half_patch_size+0  ))
						+ (cleanedPatch.at<uchar>(half_patch_size+1,  half_patch_size+1));

	if (neibghours < 3)
	{
		return false;
	}

	//collect all white points along a rectangle
	list<cv::Point2f> whitepoints;
	list<cv::Point2f>::iterator itlist;
	cv::Point2f pt1,pt2;
	float dotvalue;
	int x,y;
	/**
	 * 0 1 2 3 4 5
	 *           6
	 *           7
	 *           8
	 *           9
	 */
    unsigned char currentvalue,lastvalue = cleanedPatch[1][0];
	for (x = 0 ; x < cleanedPatch.cols - 1 ; x++)
	{
		currentvalue = cleanedPatch[0][x];
		if ((currentvalue == 1)&&(lastvalue == 0)) whitepoints.push_back(cv::Point2f(x,0));
		lastvalue = currentvalue;
	}

	for (y = 0 ; y < cleanedPatch.rows - 1 ; y++)
	{
		currentvalue = cleanedPatch[y][cleanedPatch.cols - 1];
		if ((currentvalue == 1)&&(lastvalue == 0)) whitepoints.push_back(cv::Point2f(cleanedPatch.cols - 1,y));
		lastvalue = currentvalue;
	}

	for (x = cleanedPatch.cols - 1 ; x > 0 ; x--)
	{
		currentvalue = cleanedPatch[cleanedPatch.rows-1][x];
		if ((currentvalue == 1)&&(lastvalue == 0)) whitepoints.push_back(cv::Point2f(x,cleanedPatch.rows-1));
		lastvalue = currentvalue;
	}

	for (y = cleanedPatch.rows - 1 ; y > 0 ; y--)
	{
		currentvalue = cleanedPatch[y][0];
		if ((currentvalue == 1)&&(lastvalue == 0)) whitepoints.push_back(cv::Point2f(0,y));
		lastvalue = currentvalue;
	}

	if (whitepoints.size() < 3)
	{
		return 0.0f;
	}
	else
	{
		return 100.0f;
	}
}

float ImgPatch::cornerOrIntersectionValue()
{
	float v = intersectionValue();
	if (v > 1)
	{
		return v;
	}
	else cornerValue();
}

float ImgPatch::onlyCornerValue()
{
	float v = intersectionValue();
	if (v > 1)
	{
		return 0.0f;
	}
	else cornerValue();
}

bool ImgPatch::selfTest()
{
	//Test 1 intersection:
	cv::Mat intersection_test = (cv::Mat_<uchar>(9,9) << 1, 0, 0, 0, 0, 0, 0, 0, 1,
							0, 1, 0, 0, 0, 0, 0, 1, 0,
							0, 0, 1, 0, 0, 0, 1, 0, 0,
							0, 0, 0, 1, 0, 1, 0, 0, 0,
							0, 0, 0, 0, 1, 0, 0, 0, 0,
							0, 0, 0, 0, 1, 0, 0, 0, 0,
							0, 0, 0, 0, 1, 0, 0, 0, 0,
							0, 0, 0, 0, 1, 0, 0, 0, 0,
							0, 0, 0, 0, 1, 0, 0, 0, 0);
	setPatch(intersection_test);
	if (intersectionValue() < 1) return false;

	//Test 2 Single line test (horizontal)
	cv::Mat line_test_1 = (cv::Mat_<uchar>(9,9) << 0, 0, 0, 0, 1, 0, 0, 0, 0,
							0, 0, 0, 0, 1, 0, 0, 0, 0,
							0, 0, 0, 0, 1, 0, 0, 0, 0,
							0, 0, 0, 0, 1, 0, 0, 0, 0,
							0, 0, 0, 0, 1, 0, 0, 0, 0,
							0, 0, 0, 0, 1, 0, 0, 0, 0,
							0, 0, 0, 0, 1, 0, 0, 0, 0,
							0, 0, 0, 0, 1, 0, 0, 0, 0,
							0, 0, 0, 0, 1, 0, 0, 0, 0);
	setPatch(line_test_1);
	if (intersectionValue() + cornerValue() > 1.0) return false;

	//Test 3 Single line test (diagnol):
	cv::Mat line_test_2 = (cv::Mat_<uchar>(9,9) << 1, 0, 0, 0, 0, 0, 0, 0, 0,
							0, 1, 0, 0, 0, 0, 0, 0, 0,
							0, 0, 1, 0, 0, 0, 0, 0, 0,
							0, 0, 0, 1, 0, 0, 0, 0, 0,
							0, 0, 0, 0, 1, 0, 0, 0, 0,
							0, 0, 0, 0, 0, 1, 0, 0, 0,
							0, 0, 0, 0, 0, 0, 1, 0, 0,
							0, 0, 0, 0, 0, 0, 0, 1, 0,
							0, 0, 0, 0, 0, 0, 0, 0, 1);

	setPatch(line_test_2);
	if (intersectionValue() + cornerValue() > 1.0) return false;

	//Test 4 Single corner test (90 degree):
	cv::Mat corner = (cv::Mat_<uchar>(9,9) << 0, 0, 0, 0, 0, 0, 0, 0, 0,
							0, 0, 0, 0, 0, 0, 0, 0, 0,
							0, 0, 0, 0, 0, 0, 0, 0, 0,
							0, 0, 0, 0, 0, 0, 0, 0, 0,
							0, 0, 0, 0, 1, 1, 1, 1, 1,
							0, 0, 0, 0, 1, 0, 0, 0, 0,
							0, 0, 0, 0, 1, 0, 0, 0, 0,
							0, 0, 0, 0, 1, 0, 0, 0, 0,
							0, 0, 0, 0, 1, 0, 0, 0, 0);

	setPatch(corner);
	if (intersectionValue() > 1) return false;
	if (cornerValue() < 1) return false;
}

ScaledDetector::ScaledDetector(cv::Mat binaryImg, int half_patch_size, int num_of_scale)
: nextScaleDetector(NULL), halfPatchSize(half_patch_size)
{
	if (num_of_scale > 1)
	{
		cv::Mat next_scale_binaryImg;
		cv::pyrDown(binaryImg, next_scale_binaryImg);
		threshold(next_scale_binaryImg,next_scale_binaryImg,100,255,cv::THRESH_BINARY);
		nextScaleDetector = new ScaledDetector(next_scale_binaryImg, halfPatchSize, num_of_scale - 1);
	}

	currentSkeleton = binaryImg.clone();
	skeletonize(currentSkeleton);
	reponseMap = cv::Mat::zeros(currentSkeleton.rows,currentSkeleton.cols, CV_32F);

//	cv::imshow("binary", binaryImg);
//	cv::imshow("skeletonize", currentSkeleton);
//	cv::waitKey(-1);
}

ScaledDetector::~ScaledDetector()
{
	if (nextScaleDetector != NULL)
	{
		delete nextScaleDetector;
	}
}

void ScaledDetector::detectCornerInCurrentScale()
{
	int l,c;
	int rownum = currentSkeleton.rows;
	int colnum = currentSkeleton.cols;
	ImgPatch patchInProcess;

	for (l = halfPatchSize ; l < rownum - halfPatchSize ; ++l)
	{
		for (c = halfPatchSize ; c < colnum - halfPatchSize ; ++c)
	    {
			if (currentSkeleton[l][c] != 0)
			{
				patchInProcess.setPatch(currentSkeleton(cv::Rect(c-halfPatchSize, l-halfPatchSize, halfPatchSize*2+1, halfPatchSize*2+1)));
				reponseMap[l][c] += patchInProcess.onlyCornerValue();
			}
	    }
	}

	//Only keep response on the skeleton
	cv::Mat temp = reponseMap.clone();
	reponseMap.setTo(0);
	temp.copyTo(reponseMap, currentSkeleton);
}



void ScaledDetector::detectIntersectionInCurrentScale()
{
	int l,c;
	int rownum = currentSkeleton.rows;
	int colnum = currentSkeleton.cols;
	float intersectionV;
	cv::Mat_<uchar> intersectionArea = cv::Mat::zeros(rownum, colnum, CV_8U);

	ImgPatch patchInProcess;

	for (l = halfPatchSize ; l < rownum - halfPatchSize ; ++l)
	{
		for (c = halfPatchSize ; c < colnum - halfPatchSize ; ++c)
	    {
			if (currentSkeleton[l][c] != 0)
			{
				patchInProcess.setPatch(currentSkeleton(cv::Rect(c-halfPatchSize, l-halfPatchSize, halfPatchSize*2+1, halfPatchSize*2+1)));
				intersectionV = patchInProcess.intersectionValue();
				if (intersectionV > 1)
				{
					reponseMap[l][c] += intersectionV;
					intersectionArea(cv::Rect(c-1,l-1,3,3)).setTo(255);
				}
			}
	    }
	}

//	imshow("before", currentSkeleton);
	currentSkeleton = currentSkeleton - intersectionArea;
//	imshow("after", currentSkeleton);
//	cv::waitKey(-1);
}


void ScaledDetector::detectAllInCurrentScale()
{
	int l,c;
	int rownum = currentSkeleton.rows;
	int colnum = currentSkeleton.cols;
	ImgPatch patchInProcess;

	for (l = halfPatchSize ; l < rownum - halfPatchSize ; ++l)
	{
		for (c = halfPatchSize ; c < colnum - halfPatchSize ; ++c)
	    {
			patchInProcess.setPatch(currentSkeleton(cv::Rect(c-halfPatchSize, l-halfPatchSize, halfPatchSize*2+1, halfPatchSize*2+1)));
			reponseMap[l][c] += patchInProcess.cornerOrIntersectionValue();
	    }
	}
}

/**
 * This function results in a response map with only response on skeletons
 */
void ScaledDetector::detectCorner()
{
	if (nextScaleDetector != NULL)
	{
		nextScaleDetector->detectCorner();

		//Find the local maximum in "next scale"
		//Find local maximum of the reponse map
		cv::Mat element = getStructuringElement( cv::MORPH_RECT, cv::Size( 7, 7 ), cv::Point( -1, -1 ) );

		//Apply the dilation operation
		cv::Mat_<float> dilated_reponseMap;
		dilate( nextScaleDetector->reponseMap, dilated_reponseMap, element );
		cv::Mat_<uchar> local_maximum = ((nextScaleDetector->reponseMap - dilated_reponseMap) == 0);

		reponseMap = cv::Mat::zeros(nextScaleDetector->reponseMap.rows*2, nextScaleDetector->reponseMap.cols*2, CV_32F);
		for (int i = 0 ; i < nextScaleDetector->reponseMap.rows ; i++)
		{
			for (int j = 0 ;j < nextScaleDetector->reponseMap.cols ; j++)
				if ((nextScaleDetector->reponseMap[i][j] > 1)&&(local_maximum[i][j] != 0))
				{
					reponseMap[2*i][2*j] = nextScaleDetector->reponseMap[i][j];
					reponseMap[2*i+1][2*j] = nextScaleDetector->reponseMap[i][j];
					reponseMap[2*i][2*j+1] = nextScaleDetector->reponseMap[i][j];
					reponseMap[2*i+1][2*j+1] = nextScaleDetector->reponseMap[i][j];
				}
		}
		reponseMap = reponseMap(cv::Rect(0,0,currentSkeleton.cols, currentSkeleton.rows));
	}
	else
	{
		reponseMap = cv::Mat::zeros(currentSkeleton.rows, currentSkeleton.cols, CV_32F);
	}
	detectCornerInCurrentScale();

//	cout<<reponseMap<<endl;
}

void ScaledDetector::detectAll()
{
	detectCorner();
	detectIntersectionInCurrentScale();
}

void ScaledDetector::findKeypointsInCurrentScale(vector<cv::Point2f> &keypoints)
{
	//Find local maximum
	cv::Mat element = getStructuringElement( cv::MORPH_RECT, cv::Size( 7, 7 ), cv::Point( -1, -1 ) );

	//Apply the dilation operation
	cv::Mat_<float> dilated_reponseMap;
	dilate( reponseMap, dilated_reponseMap, element );
	cv::Mat_<uchar> positveReponse = ((reponseMap - dilated_reponseMap) == 0);

	keypoints.clear();
	cv::Rect comp;
	cv::Point2f tmppoint;
	int rownum = currentSkeleton.rows;
	int colnum = currentSkeleton.cols;
	for (int l = halfPatchSize ; l < rownum - halfPatchSize ; ++l)
	{
		for (int c = halfPatchSize ; c < colnum - halfPatchSize ; ++c)
	    {
			if ((positveReponse[l][c] != 0)&&(reponseMap[l][c] > 1))
			{
				cv::floodFill(positveReponse, cvPoint(c, l), cvScalarAll(0), &comp, 8);          //clean the points from intersectionMap
				tmppoint = cv::Point2f(comp.x + (float)(comp.width-1) / 2.0, comp.y + (float)(comp.height-1) / 2.0);
				keypoints.push_back(cv::Point2f(tmppoint.x, tmppoint.y));
			}
	    }
	}

}

void ScaledDetector::drawKeypoints()
{
	if (nextScaleDetector != NULL)
	{
		nextScaleDetector->drawKeypoints();
	}

	vector<cv::Point2f> keypoints;
	findKeypointsInCurrentScale(keypoints);

	vector<cv::Mat_<uchar> > tomerge(3);
	tomerge[0] = currentSkeleton.clone();
	tomerge[1] = cv::Mat::zeros(currentSkeleton.rows, currentSkeleton.cols, CV_8U);
	tomerge[2] = cv::Mat::zeros(currentSkeleton.rows, currentSkeleton.cols, CV_8U);
	cv::Mat drawResult;
	cv::merge(tomerge,drawResult);

	for (int i = 0 ; i < keypoints.size() ; i++)
	{
		circle( drawResult, keypoints[i], 4, cv::Scalar( 0, 255, 0 ), -1, 8, 0 );
	}
	imshow("result", drawResult);

	currentSkeleton = currentSkeleton/255;
	for (int i = 0 ; i < keypoints.size() ; i++)
	{
		cout<<keypoints[i]<<endl;
		currentSkeleton[(int)keypoints[i].y][(int)keypoints[i].x] = 8;
	}
//	cout<<currentSkeleton<<endl;
	cv::waitKey(-1);
}
