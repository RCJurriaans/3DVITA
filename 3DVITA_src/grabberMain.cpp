#include "grabberMain.h"

SimpleOpenNIViewer::SimpleOpenNIViewer()
{

	write_allowed = true;
	write_done = false;

	/* initialize OpenAL context, asking for 44.1kHz to match HRIR data */
	ALCint contextAttr[] = {ALC_FREQUENCY,44100,0};
	device = alcOpenDevice( NULL );
	context = alcCreateContext( device, contextAttr );
	alcMakeContextCurrent( context );

	const char *defname = alcGetString(NULL, ALC_DEFAULT_DEVICE_SPECIFIER);
	std::cout << "Default device: " << defname << std::endl;


	/* listener at origin, facing down -z (ears at 20cm below camera) */
	alListener3f( AL_POSITION, 0., -0.2, 0. );
	alListener3f( AL_VELOCITY, 0., 0., 0. );
	float ori[6] = {0., 0., -1. , 0., 1., 0.};
	alListenerfv( AL_ORIENTATION, ori );

	cv::namedWindow("input", CV_WINDOW_AUTOSIZE);
	frameRGB = cv::Mat(480, 640, CV_8UC3);
	frameBGR = cv::Mat(480, 640, CV_8UC3);;
	frameD = cv::Mat(480, 640, CV_32F);
	frameHSV = cv::Mat(480, 640, CV_8UC3);

	for(int i=0; i<128; i++){
		sources.push_back(new AudioSource());
	}


	// C E G C* E G C E G
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

SimpleOpenNIViewer::~SimpleOpenNIViewer()
{
	alcDestroyContext( context );
	alcCloseDevice( device );
	sources.clear();

}

// Function to retrieve rgb- and depth images, constant_in is the focal_length
void SimpleOpenNIViewer::rgbd_cb_ ( const boost::shared_ptr<openni_wrapper::Image>      &image_in, 
	const boost::shared_ptr<openni_wrapper::DepthImage> &depth_in, 
	float constant_in) 
{
	if(write_allowed){
		std::cout << "Writing files and segmentation" << std::endl;
		image_in->fillRGB(frameRGB.cols,frameRGB.rows,frameRGB.data,frameRGB.step);

		cv::cvtColor(frameRGB, frameBGR, CV_RGB2BGR);
		cv::cvtColor(frameRGB, frameHSV, CV_RGB2HSV);

		depth_in->fillDepthImage(frameD.cols, frameD.rows, (float*) frameD.data, frameD.step);

		int index;
		float note;
		for(int i=0 ; i<16; i++)
		{
			for(int j=0 ; j<8; j++)
			{
				index = i+(j*16);
				// Get 128 cells from the image
				segments[index] = frameHSV(cv::Range(j*60, j*60+59), cv::Range(i*40,i*40+39 ));

				// Get mean hue from each cell
				huevalues[index] =  cv::mean(segments[i+(j*16)])[0];
				satvalues[index] =  cv::mean(segments[i+(j*16)])[1];
				valvalues[index] =  cv::mean(segments[i+(j*16)])[2];

				/*
				// Convert the hue into a note and then into a frequency
				int noteindex = (int) ((huevalues[index]/360.0)*8)+0.5;
				note = notes[noteindex];
				tonefreqs[index] = 440.f * pow(2, (( note-49) /12));
				*/

				// More elaborate method with hue and value
				hue =  huevalues[index];
				sat =  satvalues[index];
				val =  valvalues[index];

				tone = 0;
				float chord[] = {0, 2, 4, 5, 7, 9, 11};
				float octave[] = {28, 40, 52, 64, 78};
				chordindex = (int) (((hue/360.0)*7)+0.5);
				octaveindex = (int) (((val/360.0)*5)+0.5);
				note = octave[octaveindex]+chord[chordindex];
				tonefreqs[index] = 440.f * pow(2, (( note-49) /12));

				// Get the depthvalue
				depthvalues[index] = frameD.at<float>(j*60+30, i*40+20);

				//depthvalues[index] = 3;//cv::mean(frameD(cv::Range(j*60, j*60+59), cv::Range(i*40, i*40+39)))[0];

				// Convert depthvalue to 3d position and generate sound
				location[0] = (float)(i*40+20)*depthvalues[index] * constant_in;
				location[1] = (float)(j*60+30)*depthvalues[index] * constant_in;
				location[2] = depthvalues[index];

				sources.at(index)->updateSound(location, tonefreqs[index]);
				//sources[index]->updateSound(location, tonefreqs[index]);
			}

		}
		write_allowed = false;
		write_done = true;
		std::cout << "Writing done" << std::endl;
	} 
}


void SimpleOpenNIViewer::playSounds()
{
	while(true){
		if( write_done ){
			std::cout << "Playing Sounds" << std::endl;
			int counter = 0;

			/*
			for(int i=0; i<128; i++)
			{
			if(!(depthvalues[i] != depthvalues[i])){
			sources.at(i)->play();
			counter++;
			}
			}*/


			/*			for(int i=4; i<12; i++)
			{
			for(int j=1; j<7; j++)
			{
			int index = i+(j*16);
			if(!(depthvalues[index] != depthvalues[index])){
			cv::Mat tmp = cv::Mat(1,1,CV_8UC3);
			cv::rectangle( frameBGR,
			cv::Point(i*40, j*60 ),
			cv::Point( i*40+40, j*60+60),
			cv::Scalar( 0, 0, 0 ),
			3,
			8 );

			sources.at(index)->play();
			Sleep(330);
			counter++;
			}
			}
			}*/

			/*
			std::vector<int> indexes ;
			std::vector<float> vecdepth;
			for(int i = 0; i<sizeof(depthvalues)/sizeof(depthvalues[0]); i++){
			if(!(depthvalues[i] != depthvalues[i])){
			vecdepth.push_back(depthvalues[i]);
			indexes.push_back(i);
			}
			}

			sort (indexes.begin (), indexes.end (), compare_index (vecdepth));
			for(std::vector<int>::size_type it = 0; it != 16; it++) {
			int index = indexes[it];
			int i = index%16;
			int j = index/16;


			//std::cout << index << " with " << it << " = " << i << ", " << j << " => " << vecdepth[index] << std::endl;
			cv::Mat tmp = cv::Mat(1,1,CV_8UC3);
			cv::rectangle( frameBGR,
			cv::Point(i*40, j*60 ),
			cv::Point( i*40+40, j*60+60),
			cv::Scalar( 0, 0, 0 ),
			3,
			8 );

			sources.at(index)->play();
			//Sleep(330);
			counter++;


			}*/

			for(int i=4; i<12; i++)
			{
				for(int j=1; j<7; j++)
				{
					int index = i+(j*16);
					if(!(depthvalues[index] != depthvalues[index])){
						cv::Mat tmp = cv::Mat(1,1,CV_8UC3);
						cv::rectangle( frameBGR,
							cv::Point(i*40, j*60 ),
							cv::Point( i*40+40, j*60+60),
							cv::Scalar( 0, 0, 0 ),
							3,
							8 );

						sources.at(index)->play();
						Sleep(330);
						counter++;
					}
				}
			}

			Sleep(990);

			std::cout << "Played: " << counter << std::endl;

			write_allowed = true;
			write_done = false;


		}
	}
}


void SimpleOpenNIViewer::run ()
{
	// Test tone
	AudioSource* test = new AudioSource();
	test->play();
	Sleep(1000);
	std::cout << "Test tone played" << std::endl;

	pcl::Grabber* interface = new pcl::OpenNIGrabber();

	boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&, const boost::shared_ptr<openni_wrapper::DepthImage>&, float constant)> rgbd =\
		boost::bind (&SimpleOpenNIViewer::rgbd_cb_, this, _1,_2,_3);

	boost::signals2::connection c = interface->registerCallback (rgbd);

	interface->start();

	boost::thread sound_thread(&SimpleOpenNIViewer::playSounds, this);

	int k = -1;
	while (k==-1)
	{	
		for(int i=1; i<16; i++){
			cv::line(frameBGR, cv::Point(i*40,0), cv::Point(i*40, 480), cv::Scalar( 0, 0, 0 ),1, 8, 0);
		}
		for(int j=1; j<8; j++){
			cv::line(frameBGR, cv::Point(0,j*60), cv::Point(640, j*60), cv::Scalar( 0, 0, 0 ),1, 8, 0);
		}
		//boost::this_thread::sleep (boost::posix_time::seconds (2));
		cv::imshow("input", frameBGR );
		//cv::imshow("input", frameD );
		_CrtDumpMemoryLeaks();
		k = cv::waitKey(330);

	}

	sound_thread.detach();

	cv::destroyAllWindows();
	interface->stop ();
}





int main ()
{
	SimpleOpenNIViewer v;
	v.run ();
	return (0);
}