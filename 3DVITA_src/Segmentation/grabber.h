//#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
//#include <crtdbg.h>

#include <stdio.h>

#include <iostream>

/* PCL headers */
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>

/* Math Headers */
#define _USE_MATH_DEFINES
#include <math.h>    /* for sqrtf() */

/* OpenCV headers */
#include <cv.h>  
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/* Segmentation */
#include "segmentation.h"

class Grabber
{
public:
	Grabber();
	~Grabber();
	void run();

private:
	// OpenCV images
	cv::Mat frameRGB;
	cv::Mat frameBGR;
	cv::Mat frameD;
	cv::Mat frameHSV;
	cv::Mat frameS;
	bool newImages;
	bool converting;

protected:
	void retrieveImages();


	void rgbd_cb_ ( const boost::shared_ptr<openni_wrapper::Image>      &image_in, 
		const boost::shared_ptr<openni_wrapper::DepthImage> &depth_in, 
		float constant_in);




};