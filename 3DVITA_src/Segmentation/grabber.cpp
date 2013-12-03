#include "grabber.h"

Grabber::Grabber(){
	_CrtSetReportMode( _CRT_WARN, _CRTDBG_MODE_DEBUG );
	cv::namedWindow("input", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);

	// Relevant images
	frameRGB = cv::Mat(480, 640, CV_8UC3);
	frameBGR = cv::Mat(480, 640, CV_8UC3);;
	frameD   = cv::Mat(480, 640, CV_32F);
	frameHSV = cv::Mat(480, 640, CV_8UC3);
	frameS   = cv::Mat(480, 640, CV_8U);
	newImages = true;

}

Grabber::~Grabber(){
	//	AC->destroySoundEngine();
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

			//frameBGR = cv::imread("C:\\Users\\Robrecht\\3DVITA\\3DVITA_src\\Segmentation\\miro.jpg", CV_LOAD_IMAGE_COLOR);
			//cv::resize(frameBGR, frameBGR, cv::Size(640,480));
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

			std::vector<std::vector<int> > slices;
			slices.resize(33);

			for(int i=0; i<std::min(128,sp->num_segments()); i++){
				cv::Scalar hsv = mean(frameHSV, frameS==i);
				M = cv::Mat(480, 640, CV_8UC3,  hsv);//cv::Scalar(hsv[0],255,255,125));

				// Create Sound
				float depth = frameD.at<float>(segs[i].center);
				if(depth!=depth){
					//						objects[i].location[2] = depth;
				} else{
					float location[3] = {(int)(segs[i].center.x)*depth* constant_c, segs[i].center.y*depth*constant_c, depth};
					audio::audioParams params;
					///* Hue Names: http://www.procato.com/rgb+index/
					// *   0 = Red		392
					// *  30 = Orange	440
					// *  60 = Yellow	466.16
					// * 120 = Green	523.25
					// * 180 = Cyan		554.37
					// * 240 = Blue		587.33
					// * 300 = Magenta	698.46
					// * 360 = Red
					// * Frequency adjusted to scale based on A4=440.f
					// */
					float tones[7] = {392, 440, 466.16, 523.25, 554.37, 587.33, 698.46};
					params.freq = tones[(int)((hsv[0]/255)*7)];

					slices[std::min(19, (int)(depth*3.3))].push_back(AE.sounds.size());
					AE.createSound(params, location);
				}

				M.copyTo(frameBGR, frameS==i);
				M.release();
			}
			printf("playing sounds\n");

			cv::cvtColor(frameBGR, frameBGR, CV_HSV2BGR);

			for(int j=0; j<slices.size(); j++){
				for(int k =0; k<slices[j].size(); k++){
					int i = slices[j][k];
						if(AE.sounds[i].location[2]!=AE.sounds[i].location[2]){
							cv::circle(frameBGR, segs[i].center, 5, cv::Scalar( 0, 0, 0 ), -1 , 8);
						}else {
							cv::circle(frameBGR, segs[i].center, (10.0f/ AE.sounds[i].location[2]), cv::Scalar( 0, 0, 255 ), -1 , 8);
							alSourcePlay( AE.sounds[i].source );
						//cv::imshow("input", frameBGR );
						//cv::waitKey(110);
						}
				}
				cv::imshow("input", frameBGR );
				//std::cout << slices[j].size() << std::endl;
				cv::waitKey(1000 * ((slices[j].size())/(float)sp->num_segments() )+1);
				slices[j].clear();
			}
			slices.clear();
			for(int i = 0; i<AE.sounds.size(); i++){
				alDeleteSources( 1, &AE.sounds[i].source );
				alDeleteBuffers( 1, &AE.sounds[i].buffer );
			}
			AE.sounds.clear();

			if (drawing)
				sp->DrawContours(color);
			else
				cv::imshow("input", frameBGR );


			converting=false;
			newImages=false;		
			k = cv::waitKey(1);

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
		segs.clear();

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

	//_CrtDumpMemoryLeaks();
	return (0);
	//int tst;
	//std::cin >> tst;

	//audio::audioParams params;
	//params.freq = 440.f;
	//params.attack = 0.1;
	//params.decay = 0.1;
	//params.release = 0.1;
	//params.sustain = 0.1;
	//params.texture = 0.1;
	//params.dampThres = 0.5;
	//params.overtone_amps.push_back(0.9);
	//params.overtone_amps.push_back(0.3);
	//params.overtone_amps.push_back(0.9);
	//params.overtone_amps.push_back(0.3);

	///* Hue Names: http://www.procato.com/rgb+index/
	// *   0 = Red		392
	// *  30 = Orange	440
	// *  60 = Yellow	466.16
	// * 120 = Green	523.25
	// * 180 = Cyan		554.37
	// * 240 = Blue		587.33
	// * 300 = Magenta	698.46
	// * 360 = Red
	// * Frequency adjusted to scale based on A4=440.f
	// */
	//std::map<int, int> colorBins;
	//

	//for(int i =0; i<tst; i++)
	//		AE.createSound(params);
	//AE.playSounds();
}