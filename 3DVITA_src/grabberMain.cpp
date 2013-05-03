#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>

#include <cv.h>  
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class SimpleOpenNIViewer
{
public:
	cv::Mat frameRGB;

	cv::Mat segments[128];
	float huevalues[128];
	float depthvalues[128];

	SimpleOpenNIViewer(){
		cv::namedWindow("input", CV_WINDOW_AUTOSIZE);
		frameRGB = cv::Mat(480, 640, CV_8UC3);
	}

	void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
	{

	}

	void rgbd_cb_ ( const boost::shared_ptr<openni_wrapper::Image>      &image_in, 
		const boost::shared_ptr<openni_wrapper::DepthImage> &depth_in, 
		float constant_in) 
	{

		image_in->fillRGB(frameRGB.cols,frameRGB.rows,frameRGB.data,frameRGB.step);
		cv::Mat frameHSV;
		cv::cvtColor(frameRGB, frameHSV, CV_RGB2HSV);

		cv::Mat frameD = cv::Mat(depth_in->getHeight(), depth_in->getWidth(), CV_32F);
		depth_in->fillDepthImage(frameD.cols, frameD.rows, (float*) frameD.data, frameD.step);
		
		for(int i=0 ; i<16; i++)
		{
			for(int j=0 ; j<8; j++)
			{
				segments[i+(j*16)] = frameHSV(cv::Range(j*60, j*60+59), cv::Range(i*40,i*40+39 ));
				
				huevalues[i+(j*16)] =  cv::mean(segments[i+(j*16)])[0];
				depthvalues[i+(j*16)] = frameD.at<float>(j*60+30, i*40+20);
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
			cv::imshow("input", frameRGB );
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