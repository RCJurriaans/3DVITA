#include "grabber.h"

Grabber::Grabber(){
	cv::namedWindow("input", CV_WINDOW_AUTOSIZE);

	// Relevant images
	frameRGB = cv::Mat(480, 640, CV_8UC3);
	frameBGR = cv::Mat(480, 640, CV_8UC3);;
	frameD = cv::Mat(480, 640, CV_32F);
	frameHSV = cv::Mat(480, 640, CV_8UC3);
}

Grabber::~Grabber(){
	cv::destroyAllWindows();
}

void Grabber::run ()
{
	// Create the interface
	pcl::Grabber* interface = new pcl::OpenNIGrabber();

	// Connect rgbd callback
	boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&, const boost::shared_ptr<openni_wrapper::DepthImage>&, float constant)> rgbd =\
		boost::bind (&Grabber::rgbd_cb_, this, _1,_2,_3);
	boost::signals2::connection c = interface->registerCallback (rgbd);

	// Start the interface
	interface->start();

	IplImage ipl_img;
		CvScalar color = CV_RGB(255, 0, 0);
int num_segs;
std::vector<ncmec::Segment> segs;

	// Thread runs forever until stop
	int k = -1;
	while (k==-1)
	{	
		//boost::this_thread::sleep (boost::posix_time::seconds (2));
		ipl_img = frameBGR;

		ncmec::SuperPixel* sp = new  ncmec::SuperPixel(&ipl_img);

		//cvReleaseImage(&img);
		sp->SegmentNumber(128, 20);
		//  sp->SegmentSTEP(STEP, m);
		segs = sp->segments();
		num_segs = sp->num_segments();

		//cv::imshow("input", frameBGR );

		sp->DrawContours(color);
		std::cout << num_segs << std::endl;
		//cv::imshow("input", frameD );
		sp->~SuperPixel();
		k = cv::waitKey(33);
	}

	interface->stop ();
}

void Grabber::rgbd_cb_ ( const boost::shared_ptr<openni_wrapper::Image>      &image_in, 
	const boost::shared_ptr<openni_wrapper::DepthImage> &depth_in, 
	float constant_in) 
{
		// Retrieve color image
		image_in->fillRGB(frameRGB.cols,frameRGB.rows,frameRGB.data,frameRGB.step);
		cv::cvtColor(frameRGB, frameBGR, CV_RGB2BGR);
//		cv::cvtColor(frameRGB, frameHSV, CV_RGB2HSV);

		// Retrieve depth image
//		depth_in->fillDepthImage(frameD.cols, frameD.rows, (float*) frameD.data, frameD.step);
}

int main ()
{
	Grabber v;
	v.run ();
	return (0);
}