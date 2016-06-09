/*
 * detector.cpp
 *
 *  Created on: Apr 10, 2015
 *      Author: liming
 */


#include "detector.hpp"
#include "../lgc/include/chronometer.h"

void removeSmallBlobs(cv::Mat& im, double size)
{

}

bool cleanup(vector<cv::Point2d>& inputoutputpoints, float thres)    //Clean up interest points
{
	std::cout << "Before cleaning intersections -- Size = " << inputoutputpoints.size() << std::endl;

	// Distances
	float d;
	cv::Point2d tmppt;
	int counter;

	for (unsigned int i = 0; i < inputoutputpoints.size(); ++i)
	{
		// Removing points too close from one another
		tmppt = inputoutputpoints[i];
		counter = 1;
		for(unsigned int j=i+1; j < inputoutputpoints.size();)
		{
			d = cv::norm(inputoutputpoints[i]-inputoutputpoints[j]);

	        if(d < thres)
	        {
	        	tmppt += inputoutputpoints[j];
	        	counter++;
	        	inputoutputpoints.erase(inputoutputpoints.begin()+j);
	        }
	        else
	            j++;
	     }
	  }

	  std::cout << "After cleaning intersections -- Size = " << inputoutputpoints.size() << std::endl;
}

void revert_y(vector<cv::Point2d>& ioKPoints, int rownum)
{
	for (unsigned int i = 0 ; i < ioKPoints.size() ; i++)
	{
		ioKPoints[i].y = rownum - ioKPoints[i].y;
	}
}

void adjustContrast(cv::Mat_<uchar> &inputouput)
{
	cv::Mat_<float> whitefloat, adjustedfloat;
	cv::Mat element = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 11, 11 ));

	cv::Mat_<float> grayfloat = inputouput;
	cv::morphologyEx(grayfloat, whitefloat, cv::MORPH_CLOSE, element);

	cv::divide(grayfloat, whitefloat, adjustedfloat);

	cv::normalize(adjustedfloat,adjustedfloat,0,255,cv::NORM_MINMAX);
	inputouput = adjustedfloat;
}

bool AbstractKPDetector::detect(const cv::Mat_<uchar> &iSrc_gray, vector<cv::Point2d>& oKPoints, vector<float> &oDiameters) const
{
	Chronometer tm;
	cv::Mat_<uchar> grayimg = iSrc_gray.clone();
	tm.tic();
	adjustContrast(grayimg);
//	cv::imshow("after_contrust", grayimg);

//	cout<<"Adjusting contrast used "<<tm.tac()<<" ms"<<endl;
	threshold(grayimg,grayimg,180,255,cv::THRESH_BINARY);

//	cv::imshow("thres_img", grayimg);
//	cv::waitKey(-1);

	cv::Mat element = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 3, 3 ));
	cv::morphologyEx(grayimg, grayimg, cv::MORPH_CLOSE, element);

	runAlgo(grayimg, oKPoints, oDiameters);

//	for (unsigned int i = 0 ; i < oKPoints.size() ; i++)
//	{
//		circle(grayimg, oKPoints[i], 5,  cv::Scalar(125), 3, 8, 0 );
//	}
//
//	cv::imshow("binary_img", grayimg);
//	cv::imshow("source_img", iSrc_gray);
//	cv::waitKey(-1);

//	cout<<"Do not use abstract method"<<endl;
	return true;
};

SimplePointDetector::SimplePointDetector()
{
	detectorparams.filterByColor = true;        //Find black points
	detectorparams.blobColor = 0;

	detectorparams.minDistBetweenBlobs = 0.0;  // minimum 10 pixels between blobs

	detectorparams.filterByArea = true;         // filter  by area of blob
	detectorparams.minArea = 5.0;              // min 5 pixels squared
	detectorparams.maxArea = 200.0;             // max 200 pixels squared

	//Without specifying "minConvexity, some floued points can be missed
	detectorparams.filterByConvexity = true;         // filter  by area of blob
	detectorparams.minConvexity = 0.5;              // min 5 pixels squared

	blobDetector = cv::SimpleBlobDetector::create(detectorparams);

//	blobDetector = new cv::SimpleBlobDetector(detectorparams);
}

SimplePointDetector::SimplePointDetector(int blobcolor, int minArea, int maxArea)
{
	detectorparams.filterByColor = true;        //Find black points
	detectorparams.blobColor = blobcolor;

	detectorparams.minDistBetweenBlobs = 0.0;  // minimum 10 pixels between blobs

	detectorparams.filterByArea = true;         // filter  by area of blob
	detectorparams.minArea = minArea;              // min 5 pixels squared
	detectorparams.maxArea = maxArea;             // max 200 pixels squared

	//Without specifying "minConvexity, some floued points can be missed
	detectorparams.filterByConvexity = true;         // filter  by area of blob
	detectorparams.minConvexity = 0.5;              // min 5 pixels squared

	blobDetector = cv::SimpleBlobDetector::create(detectorparams);

//	blobDetector = new cv::SimpleBlobDetector(detectorparams);
}

SimplePointDetector::~SimplePointDetector()
{
}

// Todo: clean up some big black area to reduce total blob num ?
void SimplePointDetector::runAlgo(const cv::Mat_<uchar> &iSrc_gray, vector<cv::Point2d>& oKPoints, vector<float> &oDiameters) const
{
	vector<cv::KeyPoint> detectedBlobs;

	blobDetector->detect(iSrc_gray, detectedBlobs);

	//Transfer the detected points to pointarray
	int i,num;
	num = detectedBlobs.size();
	oKPoints.resize(num);
	oDiameters.resize(num);
	for(i = 0; i < num ; i++ )
	{
		oKPoints[i].x = detectedBlobs[i].pt.x;
		oKPoints[i].y = detectedBlobs[i].pt.y;
		oDiameters[i] = detectedBlobs[i].size;
	}
}

ContoursPointDetector::ContoursPointDetector() : ratio(5.0f)
{

}

/**
 * Parameterized constructor
 * @param blobcolor [Input] Color of the blob to be detected (0 means to extract black blobs, 1 means to extract white blobs)
 * @param minArea [Input] Each extracted blob's area should be larger than minArea.
 * @param maxArea [Input] Each extracted blob's area should be smaller than maxArea.
 */
ContoursPointDetector::ContoursPointDetector(float r) : ratio(r)
{

}

ContoursPointDetector::~ContoursPointDetector()
{

}

void ContoursPointDetector::runAlgo(const cv::Mat_<uchar> &iSrc_gray, vector<cv::Point2d>& oKPoints, vector<float> &oDiameters) const
{
    // Only accept CV_8UC1
    if (iSrc_gray.channels() != 1 || iSrc_gray.type() != CV_8U)
        return;

    // Find all contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(iSrc_gray.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    // Find enclosing circles
    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
    vector<cv::Point2f> center( contours.size() );
    vector<float> radius( contours.size() );
    vector<double> areaRatios( contours.size() );
    for( int i = 0; i < contours.size(); i++ )
    {
    	approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
        minEnclosingCircle( (cv::Mat)contours_poly[i], center[i], radius[i] );
        areaRatios[i] = CV_PI*radius[i]*radius[i]/cv::contourArea(contours[i]);
    }

    oKPoints.clear();
    oDiameters.clear();
    for (int i = 0; i < contours.size(); i++)
    {
    	if (areaRatios[i] < ratio)
    	{
    		oKPoints.push_back(center[i]);
    		oDiameters.push_back(2*radius[i]);
    	}
    }
}

/**
 *====================================MSER Detector=====================================
 */

MSERDetector::MSERDetector() :
m_mserMaxRadiusRatio( 3.0),
m_mserMinRadius( 5.0),
m_mserMearAreaFac( 2.0 ),
area_sigma( 2.0 ),
detectBlackBlob( true )
{

}

bool MSERDetector::detect(const cv::Mat_<uchar> &iSrc, vector<cv::Point2d>& oKPoints, vector<float> &oDiameters) const
{
	runAlgo(iSrc, oKPoints, oDiameters);
	return true;
}

void MSERDetector::draw( const cv::Mat_<uchar> &grayImg,  const std::vector<cv::RotatedRect>& ellipses ) const
{
	cv::Mat imgshow;
	cv::cvtColor(grayImg, imgshow, CV_GRAY2RGB);
	for( size_t i=0; i<ellipses.size(); i++ )
		ellipse(imgshow, ellipses[i], cv::Scalar(0,0,255),3);
	cv::pyrDown(imgshow, imgshow);
//	cv::pyrDown(imgshow, imgshow);
	imshow("dtector", imgshow);
	cv::waitKey(-1);
}

void MSERDetector::printEllipses(const std::vector<cv::RotatedRect>& ellipses) const
{
    for( size_t i=0; i<ellipses.size(); i++ )
    {
    	cout<<ellipses[i].center<<endl;
    }
}

cv::Rect getROI(const cv::Mat &source, const cv::RotatedRect &ellipse)
{
	int size = ellipse.size.width > ellipse.size.height ? ellipse.size.width : ellipse.size.height;
	int minx = ellipse.center.x - size/2 > 0 ? ellipse.center.x - size/2 : 0;
	int miny = ellipse.center.y - size/2 > 0 ? ellipse.center.y - size/2 : 0;
	int maxx = ellipse.center.x + size/2 < source.cols ? ellipse.center.x + size/2 : source.cols;
	int maxy = ellipse.center.y + size/2 < source.rows ? ellipse.center.y + size/2 : source.rows;

	return cv::Rect(minx, miny, maxx-minx, maxy-miny);
}

void MSERDetector::runAlgo( const cv::Mat_<uchar> &iSrc_gray, vector<cv::Point2d>& oKPoints, vector<float> &oDiameters ) const
{
    // init stuff
    std::vector<std::vector<cv::Point> > contours;


//    cv::Mat mask( iSrc_gray.rows, iSrc_gray.cols, iSrc_gray.type(), 255 );
    cv::GaussianBlur( iSrc_gray, iSrc_gray, cv::Size(3, 3), 2, 2 );

    // detect blobs
    cv::Ptr< cv::MSER > mserdetector = cv::MSER::create(
    		5, //delta
			60, //_min_areaint
			14400, //_max_area
            0.25, //_max_variation
			.2,  //_min_diversity
            200,  //_max_evolution
			1.01,  //_area_threshold
            0.003,  //_min_margin
			5 );  //_edge_blur_size


    vector<cv::Rect> mserBbox;
    mserdetector->detectRegions(iSrc_gray, contours, mserBbox);

//    mser( iSrc_gray, contours, mask );


    std::vector<cv::RotatedRect> ellipses;

    // fit ellipses
    for( size_t i=0; i<contours.size(); i++ )
    {
    	ellipses.push_back( cv::fitEllipse( contours[i] ) );
    }

//    draw(iSrc_gray, ellipses);

    // filter detected ellipses
    ellipses = filterEllipses( ellipses );
//    draw(iSrc_gray, ellipses);

    // remove all self intersecting elipses
//    std::vector<cv::RotatedRect> oldellipses(0);
//    while (oldellipses.size() != ellipses.size())
//    {
//    	oldellipses = ellipses;
    	ellipses = removeIntersectingEllipses( ellipses );
//    	draw(iSrc_gray, ellipses);
//    }

    // remove two interesetcting elipses
    ellipses = removeIntersectingEllipses2( ellipses );
//    draw(iSrc_gray, ellipses);

    // Color test
    ellipses = filterColors(iSrc_gray, ellipses);

    //add centers
    for( size_t e=0; e<ellipses.size(); e++ )
    {
    	oKPoints.push_back( cv::Point2d( ellipses[e].center.x, ellipses[e].center.y ) );
    	oDiameters.push_back(sqrt(ellipses[e].size.height*ellipses[e].size.width));
    }

}

std::vector<cv::RotatedRect> MSERDetector::filterColors( const cv::Mat_<uchar> &grayImg, const std::vector<cv::RotatedRect>& ellipses ) const
{
	std::vector<cv::RotatedRect> result;
    cv::Mat labels = cv::Mat::zeros(grayImg.rows, grayImg.cols, CV_32S);
    cv::RotatedRect bigger;

    for( size_t i=0; i<ellipses.size(); i++ )
    {
        bigger = ellipses[i];
        bigger.size.height = 2*bigger.size.height;
        bigger.size.width = 2*bigger.size.width;
        cv::ellipse(labels, bigger, cv::Scalar(2*i+1), CV_FILLED);
    	cv::ellipse(labels, ellipses[i], cv::Scalar(2*i+2), CV_FILLED);

    	cv::Rect roi = getROI(labels, ellipses[i]);
        cv::Scalar mean = cv::mean(grayImg(roi), labels(roi) == 2*i+2);

        cv::Rect largerroi = getROI(labels, bigger);
        cv::Scalar outermean = cv::mean(grayImg(largerroi), labels(largerroi) == 2*i+1);

//        cout<<labels(roi)<<endl;
//        cout<<labels(largerroi)<<endl;
//        cout<< mean[0]<< "   "<< outermean[0] << endl;
//        imshow("roi", grayImg(roi));
//        imshow("largerroi", grayImg(largerroi));
//        cv::waitKey(-1);

        if (int (mean[0] < outermean[0]) + (int)detectBlackBlob != 1)
        {
        	result.push_back( ellipses[i] );
        }
    }

    return result;
}

std::vector<cv::RotatedRect> MSERDetector::filterAreas( const cv::Mat_<uchar> &grayImg, const std::vector<cv::RotatedRect>& ellipses ) const
{
	std::vector<cv::RotatedRect> result;
    std::vector<float> areas;
    float pi = std::atan(1.0f)*4.0f;
    // compute the mean area
    for( size_t e = 0; e < ellipses.size(); e++ )
    {
        areas.push_back( pi * ellipses[e].size.width * ellipses[e].size.height );
    }

    double sumArea = std::accumulate(areas.begin(), areas.end(), 0.0);
    float mean_area =  sumArea / areas.size();

	double sq_sum = std::inner_product(areas.begin(), areas.end(), areas.begin(), 0.0);
	float stdev_area = std::sqrt(sq_sum / areas.size() - mean_area * mean_area);

    for( size_t e = 0; e < ellipses.size(); e++ )
    {
        // if the elllipse not too big nor too small, keep it
        if( fabs( areas[e] - mean_area ) < area_sigma*stdev_area )
        	result.push_back( result[e] );
    }

    return result;
}

std::vector<cv::RotatedRect> MSERDetector::filterEllipses( const std::vector<cv::RotatedRect>& ellipses ) const
{
    // init stuff
    std::vector<cv::RotatedRect> result;
    float pi = std::atan(1.0f)*4.0f;
    std::vector<float> areas;

    // compute the mean area
    for( size_t e = 0; e < ellipses.size(); e++ )
        areas.push_back( pi * ellipses[e].size.width * ellipses[e].size.height );

    double sumArea = std::accumulate(areas.begin(), areas.end(), 0.0);
    float mean_area =  sumArea / areas.size();

    // filter ellipses
    for( size_t e = 0; e < ellipses.size(); e++ )
    {
        // fit the elipse
        bool ok = true;
        cv::RotatedRect ell = ellipses[e];
        double rMax = std::max( ell.size.width, ell.size.height );
        double rMin = std::min( ell.size.width, ell.size.height );

        // filter the elipses
        ok = ok & ((rMax/rMin) <= m_mserMaxRadiusRatio);     //long axis/short axis
        ok = ok & (rMin > m_mserMinRadius);
        ok = ok & (fabs( areas[e] - mean_area ) < m_mserMearAreaFac*mean_area);

        // if all is well keep it
        if( ok )
            result.push_back( ell );
    }

    return result;
}

/**
 * Operation flag shows if any ellipse is removed in current function
 */
std::vector<cv::RotatedRect> MSERDetector::removeIntersectingEllipses( const std::vector<cv::RotatedRect>& ellipses) const
{
    // init stuff
    std::vector<cv::RotatedRect> result;

    // init flann
    cv::Mat_<float> centersCV( static_cast<int>(ellipses.size()), 2 );
    for( size_t i=0; i<ellipses.size(); i++ )
    {
        centersCV( i, 0 ) = ellipses[i].center.x;
        centersCV( i, 1 ) = ellipses[i].center.y;
    }


    cv::flann::Index flann( centersCV, cv::flann::KDTreeIndexParams(5));

    // run over all points and look at the neighbors
    int expectedClusteredNum = 10;
    for( size_t i=0; i<ellipses.size(); i++ )
    {
        // get the 5 nearest neighbors of this elipse' center
        cv::Mat_<int> neighbors( 1, expectedClusteredNum+1 );
        cv::Mat_<float> dists( 1, expectedClusteredNum+1 );
        flann.knnSearch( centersCV.row(i).clone(), neighbors, dists, expectedClusteredNum+1, cv::flann::SearchParams(128) );
        bool skip = false;

        // have a look at the neighbors
        // if not first point in a "dense cluster", skip
        double rMax = std::max( ellipses[i].size.width, ellipses[i].size.height );
        for( size_t n=1; n<=expectedClusteredNum; n++ )
            if( dists(n) < rMax/2 && neighbors(n) < i )
                skip = true;

        // if all went well, add the ellipse
        if( !skip )
            result.push_back( ellipses[i] );
    }

    // return

    return result;
}

std::vector<cv::RotatedRect> MSERDetector::removeIntersectingEllipses2( const std::vector<cv::RotatedRect>& ellipses ) const
{
    // init stuff
    std::vector<cv::RotatedRect> result;
    std::vector<bool> skip(ellipses.size(), false);

    for( size_t i=0; i<ellipses.size(); i++ )
    	if (!skip[i])
		{
			double dMaxi = std::max( ellipses[i].size.width, ellipses[i].size.height );
			for (size_t j=0; j < i; j++ )
			{
				if (!skip[j])
				{
					double dMaxj = std::max( ellipses[j].size.width, ellipses[j].size.height );
					double dMax = ( dMaxi + dMaxj )/2;
					double dist = cv::norm(ellipses[i].center - ellipses[j].center);
					if (dist < 0.5*dMax)
					{
//						if (fabs(ellipses[i].size.width + ellipses[i].size.height - ellipses[j].size.width - ellipses[j].size.height) < 0.5*dMax) //dense cluster
						{
//							cout<<"cluster ones : "<<endl;
//							cout<< ellipses[i].size<<" "<<ellipses[j].size<<endl;
							skip[i] = true;
						}
					}
					else if( dist < 1.2*dMax)   //two ecllipses intersect
					{
//						cout<<"intersecting ones : "<<endl;
//						cout<< ellipses[i].size<<" "<<ellipses[j].size<<endl;
						skip[i] = true;
						skip[j] = true;
						break;
					}
				}
			}
		}


	// if all went well, add the ellipse
    for( size_t i=0; i<ellipses.size(); i++ )
    	if( !skip[i] )
    		result.push_back( ellipses[i] );

    // return
    return result;
}
