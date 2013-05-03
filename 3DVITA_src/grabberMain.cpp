#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class SimpleOpenNIViewer
{
public:
	cv::Mat frameRGB;

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

		interface->stop ();
	}


};


int main ()
{
	SimpleOpenNIViewer v;
	v.run ();
	return (0);
}