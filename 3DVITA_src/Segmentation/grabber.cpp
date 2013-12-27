#include "grabber.h"

Grabber::Grabber(){
	//_CrtSetReportMode( _CRT_WARN, _CRTDBG_MODE_DEBUG );
	updateImage = false;
	keys = new char[256];
	oldkeys = new char[256];
	for (int x = 0; x < 256; x++){
		keys[x] = 0;
		oldkeys[x] = 0;
	}

	screenNo = 0;

	// Relevant images
	frameRGB = cv::Mat(480, 640, CV_8UC3);
	frameBGR = cv::Mat(480, 640, CV_8UC3);;
	frameD   = cv::Mat(480, 640, CV_32F);
	frameHSV = cv::Mat(480, 640, CV_8UC3);
	frameS   = cv::Mat(480, 640, CV_8U);

	// Control all the booleans
	newImages = false;
	converting = false;
	playing = false;

	// Control Program
	control.running = true;
	control.drawImage = true;
	control.music = false;
	control.verbose = false;

	// Control Features;
	control.color = true;
	control.saturation = true;
	control.texture = true;
	control.depth = true;

	control.screenshot = false;

	printf("Created Grabber, starting frame\n");
	boost::thread(&Grabber::updateFrame, this);


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
	Sleep(500);
	printf("Starting Interface\n");
	interface->start();
	printf("Started Interface\n");
	while(!newImages){
		Sleep(10);
	}
	printf("There is an incoming image\n");

	// Connect keyListener callback
	boost::thread keyListen(&Grabber::keyListener, this);

	IplImage ipl_img;
	CvScalar color = CV_RGB(255, 0, 0);
	int num_segs;
	std::vector<ncmec::Segment> segs;
	ipl_img = frameBGR;
	ncmec::SuperPixel* sp = new  ncmec::SuperPixel(&ipl_img);
	const int* seg_map;
	seg_map = new int[640*480];

	int drawing = 1;
	// Thread runs forever until stop
	while (control.running)
	{

		if(newImages){
			boost::progress_timer timer;
			converting = true;

			//frameBGR = cv::imread("C:\\Users\\Robrecht\\3DVITA\\3DVITA_src\\Segmentation\\malevich.jpg", CV_LOAD_IMAGE_COLOR);
			//cv::resize(frameBGR, frameBGR, cv::Size(640,480));

			// Calculate texturedness by comparing to local area
			cv::Mat edges;
			cv::Mat frameGrey, frameGreyBlur;
			cv::cvtColor(frameBGR, frameGrey, CV_BGR2GRAY);
			cv::blur(frameGrey, frameGreyBlur, cv::Size(9,9));
			edges = abs(frameGrey-frameGreyBlur);
			//	imshow("TEST", edges);
			//	cv::waitKey(33);
			frameGrey.release();
			frameGreyBlur.release();

			ipl_img = frameBGR;

			sp->updateImage(&ipl_img);	



			int clusters = 64;
			if(tmp_clusters>31)
				clusters = tmp_clusters;
			else
				clusters = 32;
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
			slices.resize(20);

			int num_seg = sp->num_segments();
			std::vector<cv::Mat> hsv_planes;
			cv::split(frameHSV, hsv_planes);
#ifdef FREQHISTO
			//cv::Mat histograms =  cv::Mat::zeros(num_seg, 6, CV_32S);
			cv::Mat histograms =  cv::Mat::zeros(num_seg, 6, CV_32F);
			for(int i=0; i<480*640; i++){
				int x = hsv_planes[0].data[i];
				int segNo = frameS.data[i];
				/*
				if(x<15)
				histograms.at<int>(segNo,0)++;
				else if(x<45)
				histograms.at<int>(segNo,1)++;
				else if(x<90)
				histograms.at<int>(segNo,2)++;
				else if(x<180)
				histograms.at<int>(segNo,3)++;
				else if(x<270)
				histograms.at<int>(segNo,4)++;
				else if(x<315)
				histograms.at<int>(segNo,5)++;
				else
				histograms.at<int>(segNo,0)++;
				*/
				if(x>245)
					std::cout << x << std::endl;
				x = (x/255.f)*360;
				if(x<11 || x >355){
					// RED G4
					histograms.at<float>(segNo,0)++;
				}else if(x<51){
					float alpha = 1-((x-11)/(50.f-11.f));
					histograms.at<float>(segNo,0) += alpha;
					histograms.at<float>(segNo,1) += (1-alpha);
				}else if(x<61){
					// Yellow Bb4
					histograms.at<float>(segNo,1)++;
				}else if(x<81){
					float alpha = 1-((x-61)/(80.f-61.f));
					histograms.at<float>(segNo,1) += alpha;
					histograms.at<float>(segNo,2) += (1-alpha);
				}else if(x<141){
					// Green C5
					histograms.at<float>(segNo,2)++;
				}else if(x<221){
					float alpha = 1-((x-141)/(220.f-141.f));
					histograms.at<float>(segNo,2) += alpha;
					histograms.at<float>(segNo,3) += (1-alpha);
				}else if(x<241){
					// Blue D#5
					histograms.at<float>(segNo,3)++;
				}else if(x<331){
					float alpha = 1-((x-241)/(330.f-241.f));
					histograms.at<float>(segNo,3) += alpha;
					histograms.at<float>(segNo,5) += (1-alpha);
				}else if(x<346){
					// Pink F5
					histograms.at<float>(segNo,5)++;
				}else{
					float alpha = 1-((x-346)/(355.f-346.f));
					histograms.at<float>(segNo,5) += alpha;
					histograms.at<float>(segNo,0) += (1-alpha);
				}


				// *   0 = Red			392		G4
				// *  30 = Orange		440		A4
				// *  60 = Yellow		466.16 Bb4
				// *  90 = Chartreuse	494		B4
				// * 120 = Green		523.25	C5
				// * 150 = Green		523.25
				// * 180 = Cyan			554.37	Db5
				// * 210 = Blue			587.33	D5
				// * 240 = Blue			587.33
				// * 270 = Violet		659.25	E5
				// * 300 = Magenta		698.46	F5
				// * 330 = Red

			}

#else
			std::vector<float> cosSum  (num_seg, 0.0);
			std::vector<float> sinSum  (num_seg, 0.0);
			std::vector<float> hueMean (num_seg, 0.0);
			for(int i=0; i<480*640; i++){
				//printf("%i -> hsv_val: %i, %f, %f, %f\n", frameS.data[i], hsv_planes[0].data[i], hsv_planes[0].data[i]/255.f, hsv_planes[0].data[i]/255.f, ((float)hsv_planes[0].data[i]/255.f)*360.f);
				//cosSum[frameS.data[i]] += cos((((float)hsv_planes[0].data[i]/255.f)*360.f)*M_PI/180.f);
				//sinSum[frameS.data[i]] += sin((((float)hsv_planes[0].data[i]/255.f)*360.f)*M_PI/180.f);
				cosSum[frameS.data[i]] += cos((float)hsv_planes[0].data[i]*M_PI/180.0);
				sinSum[frameS.data[i]] += sin((float)hsv_planes[0].data[i]*M_PI/180.0);
			}

			for(int i =0; i<hueMean.size(); i++){
				hueMean[i] = atan2(sinSum[i], cosSum[i])*180.0/M_PI;
			}
			cosSum.clear();
			sinSum.clear();
#endif
			hsv_planes.clear();
			int maxTex = 0;

			for(int i=0; i<std::min(clusters, sp->num_segments()); i++){
				cv::Scalar hsv = mean(frameHSV, frameS==i);

				//cv::Mat tmpEdges;
				//edges.copyTo(tmpEdges, frameS==i);
				//cv::Scalar texture = cv::sum(tmpEdges);
				cv::Scalar texture = mean(edges, frameS==i);
				//tmpEdges.release();
				//	std::cout << "cluster " << i << " has sum of " << texture[0] << " and Max is " << maxTex << std::endl;

#ifndef FREQHISTO
				M = cv::Mat(480, 640, CV_8UC3,  cv::Scalar(hueMean[i], hsv[1], hsv[2]));
#else
				M = cv::Mat(480, 640, CV_8UC3,  hsv);
#endif
				// Create Sound
				float depth = frameD.at<float>(segs[i].center);
				if(!control.depth)
					depth = 2.f+ (sqrt( (pow(segs[i].center.x-320.f, 2) + pow(segs[i].center.y-240.f,2)))/100.f);
				if(depth<0.3 || depth!=depth){
					//						objects[i].location[2] = depth;
				} else{
					depth = std::max(0.1f, depth);
					float location[3] = {(int)(segs[i].center.x)*depth* constant_c, segs[i].center.y*depth*constant_c, depth};
					audio::audioParams params;
					///* Hue Names: http://www.procato.com/rgb+index/
					// *   0 = Red			392		G4
					// *  30 = Orange		440		A4
					// *  60 = Yellow		466.16 Bb4
					// *  90 = Chartreuse	494		B4
					// * 120 = Green		523.25	C5
					// * 150 = Green		523.25
					// * 180 = Cyan			554.37	Db5
					// * 210 = Blue			587.33	D5
					// * 240 = Blue			587.33
					// * 270 = Violet		659.25	E5
					// * 300 = Magenta		698.46	F5
					// * 330 = Red
					// * 360 = Red
					// * Frequency adjusted to scale based on A4=440.f
					// */
					//float tones[13] = {392, 440, 466.16, 493.88, 523.25, 523.25, 554.37, 587.33, 587.33, 659.25, 698.46, 392, 392};
					float tones[6] = {392, 466.16, 523.25, 554.37, 587.33, 698.46};
					//params.freq = tones[(int)((hsv[0]/255)*7)];
					if(control.verbose){
#ifndef FREQHISTO
						printf("%i,\t %f,\t %i,\t %f\n", i, hueMean[i], (int)(hueMean[i]/360.0)*6, tones[(int)(((hueMean[i]/360.0))*6)]);				
#endif
					}
					float brightness = 1.0f;
					if(((int)((hsv[2]/255.f)*2))==0)
						brightness = 0.5f;
					else if(((int)((hsv[2]/255.f)*2))==2)
						brightness = 2.0f;
					params.brightness = brightness;
#ifndef FREQHISTO
					params.freq = tones[(int)(((hueMean[i]/360.0))*6)]*brightness;
#else
					histograms.row(i).copyTo(params.freqMat);
#endif
					params.dampThres = hsv[1]/255;
					//params.freqMat = histograms;
					params.texture = texture[0]/50.f;
					params.attack	= 0.05;
					params.decay	= 0.4;
					params.release	= 0.45;
					params.sustain	= 0.9;

					depth = std::min(std::max(depth, 0.5f), 8.0f);
					slices[std::min(19, (int)(depth*2))].push_back(AE.sounds.size());
					AE.createSound(params, location);
				}

				M.copyTo(frameBGR, frameS==i);
				M.release();
			}
			if(control.verbose)
				printf("playing sounds\n");

			cv::cvtColor(frameBGR, frameBGR, CV_HSV2BGR);

			// While playing sounds, wait
			while(playing && control.running){
				Sleep(10);
			}
			//boost::progress_timer timer2;
			// Copy image, slices, sounds, segments into player
			tmpFrame = frameBGR.clone();

			std::vector<std::vector<int> > tmpSlices;
			tmpSlices.resize(slices.size());
			for(int i = 0; i<slices.size(); i++)
				for(int j = 0; j<slices[i].size(); j++)
					tmpSlices[i].push_back(slices[i][j]);

			std::vector<cv::Point> tmpCenters;
			for(int i=0; i<segs.size(); i++)
				tmpCenters.push_back(cv::Point(segs[i].center.x, segs[i].center.y));

			std::vector<audio::audioObject> tmpSounds;
			for(int i = 0; i<AE.sounds.size(); i++){
				audio::audioObject tmpSound;
				// Create a source	
				alGenSources( 1, &tmpSound.source );
				alSourcef(  tmpSound.source, AL_PITCH, 1. );
				alSourcef(  tmpSound.source, AL_GAIN, 1. );
				alSource3f( tmpSound.source, AL_POSITION, AE.sounds[i].location[0], AE.sounds[i].location[1], AE.sounds[i].location[2] );
				alSource3f( tmpSound.source, AL_VELOCITY, 0.,0.,0. );
				alSourcei(  tmpSound.source, AL_LOOPING, AL_FALSE );

				alGenBuffers( 1, &tmpSound.buffer );
				//audioData = data;

				tmpSound.location[0] = AE.sounds[i].location[0];
				tmpSound.location[1] = AE.sounds[i].location[1];
				tmpSound.location[2] = AE.sounds[i].location[2];

				tmpSound.data = new short[44100];
				for(int j = 0; j<44100 ; j++)
					tmpSound.data[j] =  AE.sounds[i].data[j];

				alBufferData(tmpSound.buffer, AL_FORMAT_MONO16, tmpSound.data, 44100, 44100 );
				alSourcei( tmpSound.source, AL_BUFFER,  tmpSound.buffer );	
				tmpSounds.push_back(tmpSound);
				//tmpSounds.push_back(AE.sounds[i]);
			}

			updateImage = true;
			boost::thread(&Grabber::playSounds, this, tmpSlices, tmpSounds, tmpCenters, sp->num_segments());

			// Remove stuff
			for(int i = 0; i<AE.sounds.size(); i++){
				alDeleteSources( 1, &AE.sounds[i].source );
				alDeleteBuffers( 1, &AE.sounds[i].buffer );
				delete [] AE.sounds[i].data;
			}
			AE.sounds.clear();	

			slices.clear();
#ifndef FREQHISTO
			hueMean.clear();
#endif
			converting=false;
			newImages=false;		
			cv::waitKey(1);

		}
		segs.clear();
	}

	sp->~SuperPixel();
	interface->stop ();
}

void Grabber::updateFrame(){
	printf("Starting Frame Display");
	cv::namedWindow("Input", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
	// Create Trackbars
	char TrackbarName[50];
	char TrackbarName2[50];
	sprintf( TrackbarName, "M: %d", 100 );
	m = 20;
	cv::createTrackbar( TrackbarName, "Input", &m, 200 );
	tmp_clusters=64;
	sprintf( TrackbarName2, "C: %d", 256 );
	cv::createTrackbar( TrackbarName2, "Input", &tmp_clusters, 512 );

	while(true){
		if(updateImage){
			cv::imshow("Input", tmpFrame);
			updateImage=false;
		}
		cv::waitKey(110);
	}

}

void Grabber::playSounds(std::vector<std::vector<int> > tmpSlices, std::vector<audio::audioObject> tmpSounds, std::vector<cv::Point> tmpCenters, int numSeg){
	playing = true;
	printf("Playing %i sounds over %i slices for %f ms\n", tmpSounds.size(), tmpSlices.size(), (float)(1000/(float)(tmpSlices.size())));
	for(int j=0; j<tmpSlices.size(); j++){
		for(int k =0; k<tmpSlices[j].size(); k++){
			int i = tmpSlices[j][k];
			cv::circle(tmpFrame, tmpCenters[i], (int)ceil(12.f-(std::min(10.f, tmpSounds[i].location[2]))), cv::Scalar( 0, 0, 255 ), -1 , 8);
			alSourcePlay( tmpSounds[i].source );
			if(control.music){
				if(control.drawImage)
					updateImage = true;
				cv::waitKey(250);
			}
		}
		//if(control.drawImage)

		updateImage=true;
		cv::waitKey(50);
		tmpSlices[j].clear();
	}

	tmpSlices.clear();
	for(int i = 0; i<tmpSounds.size(); i++){
		alDeleteSources( 1, &tmpSounds[i].source );
		alDeleteBuffers( 1, &tmpSounds[i].buffer );
		delete [] tmpSounds[i].data;
	}
	//tmpFrame.release();
	tmpSounds.clear();
	tmpCenters.clear();
	playing = false;
}

void Grabber::rgbd_cb_ ( const boost::shared_ptr<openni_wrapper::Image> &image_in, 
	const boost::shared_ptr<openni_wrapper::DepthImage> &depth_in, float constant_in) 
{
	constant_c = constant_in;
	// Retrieve color image
	image_in->fillRGB(frameRGB.cols, frameRGB.rows, frameRGB.data, frameRGB.step);
	// Retrieve depth image
	depth_in->fillDepthImage(frameD.cols, frameD.rows, (float*) frameD.data, frameD.step);

	if(!converting && control.running){
		printf("CONVERTING\n");
		//cv::blur(frameRGB, frameRGB, cv::Size(5,5));
		cv::cvtColor(frameRGB, frameBGR, CV_RGB2BGR);	
		cv::cvtColor(frameRGB, frameHSV, CV_RGB2HSV);


		std::cout << "PATCH NANS\n" << std::endl;
		cv::patchNaNs(frameD, 20.0);
		cv::Mat tmpD;
		frameD.copyTo(tmpD);
		//tmpD -= minVal;
		tmpD *= (1/(20.f));
		tmpD = 1-tmpD;
		//tmpD *= 255;

		tmpD.convertTo(tmpD, CV_32F);
		frameD.convertTo(frameD, CV_32F);

		std::cout << "BILATERAL FILTER on " << tmpD.type() << "\n" << std::endl;
		//cv::blur(tmpD, tmpD, cv::Size(7,7));
		cv::bilateralFilter(tmpD, frameD, 5, 50.0, 50.0);

		newImages = true;

	}

	if(control.screenshot){
		printf("Take ScreenShot\n");

		// RGB Image
		cv::Mat frameSave;
		cv::cvtColor(frameRGB, frameSave, CV_RGB2BGR);
		std::ostringstream oss; 
		oss << "C:/Users/Robrecht/3DVITA/3DVITA_src/Segmentation/user/image_" << screenNo << "_RGB.png";
		imwrite(oss.str(), frameSave);
		frameSave.release();

		// Depth Image
		std::ostringstream oss2; 
		oss2 << "C:/Users/Robrecht/3DVITA/3DVITA_src/Segmentation/user/image_" << screenNo << "_D.png";
		cv::Mat frameDSave;
		frameDSave = frameD*255;
		frameDSave.convertTo(frameDSave, CV_8U);
		imwrite(oss2.str(), frameDSave);
		frameDSave.release();

		screenNo++;
		control.screenshot = false;
		if(screenNo==30)
			control.running = false;

	}
	Sleep(33);
}

int main ()
{

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