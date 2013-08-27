#include "grabber.h"


Grabber::Grabber(){
	cv::namedWindow("input", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);

	AC = new audioComponents();
	AC->createSoundEngine();

	//for(int i = 0; i<128; i++){
		objects = new audioObject[128];
	//}

	// Relevant images
	frameRGB = cv::Mat(480, 640, CV_8UC3);
	frameBGR = cv::Mat(480, 640, CV_8UC3);;
	frameD   = cv::Mat(480, 640, CV_32F);
	frameHSV = cv::Mat(480, 640, CV_8UC3);
	frameS   = cv::Mat(480, 640, CV_8U);
	newImages = true;

}

Grabber::~Grabber(){
	AC->destroySoundEngine();
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
	ipl_img = frameBGR;
	ncmec::SuperPixel* sp = new  ncmec::SuperPixel(&ipl_img);
	const int* seg_map;
	seg_map = new int[640*480];


	int clusters = 128;
	int m = 20;

	int drawing = 1;
	bool running = true;

	// Thread runs forever until stop
	char k = ' ';
	while (running)
	{	
		if(newImages){
			converting = true;
			ipl_img = frameBGR;
			sp->updateImage(&ipl_img);		
			sp->SegmentNumber(clusters, m);
			segs = sp->segments();
			num_segs = sp->num_segments();
			
			seg_map = sp->segmentation_map();
			// Move segmentation map into a Mat structure
			for(int i=0; i<480*640; i++){
				frameS.data[i] = seg_map[i];
			}

			cv::Mat M;
			// Calculate mean HSV for each segment
			frameBGR.release();
			frameBGR = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0,0));

			int notes[12] = {30,32,33,35,37,39,40,42,44,45,47,49};

			for(int i=0; i<std::min(128,sp->num_segments()); i++){
				cv::Scalar hsv = mean(frameHSV, frameS==i);
				M = cv::Mat(480, 640, CV_8UC3, hsv);//cv::Scalar(hsv[0],255,255,125));

				// Create Sound
				float depth = frameD.at<float>(segs[i].center);
				if(depth!=depth){
										objects[i].location[2] = depth;
				} else{
					float location[3] = {(int)(segs[i].center.x)*depth* constant_c, segs[i].center.y*depth*constant_c, depth};
					//printf("location: %f\t%f\t%f\t", location[0], location[1], location[2]);
					float overtones[5] = {1, 0.8, 0.6, 0.4, 0.2};
					objects[i].setParams(overtones);
					//printf("hue: %f\t", hsv[0]);
					int noteind = (int)((hsv[0]/180.0f)*12);
					//printf("%i\t", notes[noteind]);
					float freq = 440.f * pow(2.0f, (((float)notes[noteind]-49.0f) /12.0f));
					//printf("%f\n", freq);
					objects[i].params.attack = hsv[1]/255.0f;
					AC->createSound(objects[i].audioData, freq);

					objects[i].updateSound(location, objects[i].audioData);
				}

				M.copyTo(frameBGR, frameS==i);
				M.release();
			}
			printf("playing sounds\n");
			
			cv::cvtColor(frameBGR, frameBGR, CV_HSV2BGR);

			for(int i=0; i<std::min(128,sp->num_segments()); i++){
				//printf("%i: %f\t", i, objects[i].location[2]);
				printf("%i, %i, %f\n", segs[i].center.x, segs[i].center.y, objects[i].location[2]); 
				if(objects[i].location[2]!=objects[i].location[2]){
					cv::circle(frameBGR, segs[i].center, 5, cv::Scalar( 0, 0, 0 ), -1 , 8);
					continue;
				}
				
				cv::circle(frameBGR, segs[i].center, (10.0f/objects[i].location[2]), cv::Scalar( 0, 0, 255 ), -1 , 8);
				objects[i].playSound();
				//cv::imshow("input", frameBGR );
				//cv::waitKey(250);
			}

			if (!drawing)
				sp->DrawContours(color);
			else
				cv::imshow("input", frameBGR );

			converting=false;
			newImages=false;		
			k = cv::waitKey(330);

		}

		switch (k) {
		case 'q':
			running = false;
			break;
		case 'c':
			std::cout << "Input number of clusters:" << std::endl;
			std::cin >> clusters;
			break;
		case 'm':
			std::cout << "Input weight m:" << std::endl;
			std::cin >> m;
			break;
		case 'd':
			if (drawing){
				drawing = 0;
				std::cout << "Stop drawing contours" << std::endl;
			}else{
				drawing = 1;
				std::cout << "Start drawing contours" << std::endl;
			}
			break;
		case '1':
			clusters -= 1;
			std::cout << "new clusters: " << clusters << std::endl;
			break;
		case '2':
			clusters += 1;
			std::cout << "new clusters: " << clusters << std::endl;
			break;
		case '3':
			m -= 1;
			std::cout << "new m: " << m << std::endl;
			break;
		case '4':
			m += 1;
			std::cout << "new m: " << m << std::endl;
			break;
		case 'h':
			std::cout << "q: to exit program" << std::endl;
			std::cout << "h: displays this text" << std::endl;
			std::cout << "c: to change input cluster count" << std::endl;
			std::cout << "m: to change weight m" << std::endl;
			std::cout << "r: resets to default values" << std::endl;
			std::cout << "d: starts/stops drawing contours" << std::endl;
			break;
		case 'r':
			std::cout << "Resetting " << std::endl;
			drawing = 1;
			clusters = 128;
			m = 20;
			break;

		}


	}

	sp->~SuperPixel();

	interface->stop ();
}

void Grabber::rgbd_cb_ ( const boost::shared_ptr<openni_wrapper::Image>      &image_in, 
	const boost::shared_ptr<openni_wrapper::DepthImage> &depth_in, 
	float constant_in) 
{
	constant_c = constant_in;
	// Retrieve color image
	image_in->fillRGB(frameRGB.cols,frameRGB.rows,frameRGB.data,frameRGB.step);
	if(!converting){
		cv::cvtColor(frameRGB, frameBGR, CV_RGB2BGR);
		cv::cvtColor(frameRGB, frameHSV, CV_RGB2HSV);
		newImages = true;
	}

	// Retrieve depth image
	depth_in->fillDepthImage(frameD.cols, frameD.rows, (float*) frameD.data, frameD.step);
}

int main ()
{


	cv::namedWindow("input", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);



	Grabber v;
	v.run ();

	/*
	audioComponents *AC = new audioComponents();

	int notes[8] = {28,32,35,40,44,47,52,56};
	float freq = 480;
	short *data = new short[44100];
	AC->defaultSound(data, freq);

	int noteind = 0;
	int note= 0;

	audioObject *sound;

	while(noteind<64){

	note = notes[noteind%8];
	freq = 440.f * pow(2.0f, (( note-49) /12.0f));
	AC->defaultSound(data, freq);
	sound = new audioObject(data);
	sound->playSound();
	noteind = (noteind+1);
	cv::waitKey(330);
	}




	*/
	_CrtDumpMemoryLeaks();
	return (0);
}