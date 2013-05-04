#include <stdlib.h>
#include <stdio.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>

#include <cv.h>  
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define _USE_MATH_DEFINES
#include <math.h>    /* for sqrtf() */


class SimpleOpenNIViewer
{
public:
	cv::Mat frameRGB;
	cv::Mat frameBGR;
	cv::Mat segments[128];
	float huevalues[128];
	float depthvalues[128];
	float tonefreqs[128];
	float notes[9];

	// C E G C* E G C E G
	

	SimpleOpenNIViewer(){
		cv::namedWindow("input", CV_WINDOW_AUTOSIZE);
		frameRGB = cv::Mat(480, 640, CV_8UC3);
		frameBGR = cv::Mat(480, 640, CV_8UC3);;
		notes[0] = 28;
		notes[1] = 32;
		notes[2] = 35;
		notes[3] = 40;
		notes[4] = 44;
		notes[5] = 47;
		notes[6] = 52;
		notes[7] = 56;
		notes[8] = 59;
	}

	void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
	{

	}

	void rgbd_cb_ ( const boost::shared_ptr<openni_wrapper::Image>      &image_in, 
		const boost::shared_ptr<openni_wrapper::DepthImage> &depth_in, 
		float constant_in) 
	{
		
		image_in->fillRGB(frameRGB.cols,frameRGB.rows,frameRGB.data,frameRGB.step);

		cv::cvtColor(frameRGB, frameBGR, CV_RGB2BGR);

		cv::Mat frameHSV;
		cv::cvtColor(frameRGB, frameHSV, CV_RGB2HSV);

		cv::Mat frameD = cv::Mat(depth_in->getHeight(), depth_in->getWidth(), CV_32F);
		depth_in->fillDepthImage(frameD.cols, frameD.rows, (float*) frameD.data, frameD.step);
		

		int index;
		float note;
		for(int i=0 ; i<16; i++)
		{
			for(int j=0 ; j<8; j++)
			{
				index = i+(j*16);
				segments[index] = frameHSV(cv::Range(j*60, j*60+59), cv::Range(i*40,i*40+39 ));
				
				huevalues[index] =  cv::mean(segments[i+(j*16)])[0];
				int noteindex = (int) ((huevalues[index]/360.0)*8)+0.5;
				note = notes[noteindex];
				tonefreqs[index] = 440.f * pow(2, (( note-49) /12));

				depthvalues[index] = frameD.at<float>(j*60+30, i*40+20);
			}
		}
	}

	void run ()
	{
		pcl::Grabber* interface = new pcl::OpenNIGrabber();

		boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&, const boost::shared_ptr<openni_wrapper::DepthImage>&, float constant)> rgbd =\
			boost::bind (&SimpleOpenNIViewer::rgbd_cb_, this, _1,_2,_3);

		boost::signals2::connection c = interface->registerCallback (rgbd);

		interface->start();

		int k = -1;
		while (k==-1)
		{
			//boost::this_thread::sleep (boost::posix_time::seconds (1));
			cv::imshow("input", frameBGR );
			k = cv::waitKey(33);
		}

		cv::destroyAllWindows();
		interface->stop ();
	}


};


int main ()
{
	SimpleOpenNIViewer v;
	v.run ();
	return (0);
}