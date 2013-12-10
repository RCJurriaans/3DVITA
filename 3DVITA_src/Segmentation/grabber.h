#ifndef GRABBER_H
#define GRABBER_H

//#define _CRTDBG_MAP_ALLOC
//#include <crtdbg.h>



#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include "vector"

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

#include <boost\thread.hpp>

/* Segmentation */
#include "segmentation.h"

#include "audioEngine.h"

/* Sound */
//#include "audiogenerator.h"
//#include "audioObject.h"

struct controls{
	// Program Controls
	bool running;
	bool drawImage;
	bool music;
	bool verbose;

	// Feature Controls
	bool depth;
	bool color;
	bool saturation;
	bool texture;

	// ScreenShot
	bool screenshot;
};

class Grabber
{
public:
	Grabber();
	~Grabber();
	void run();

private:
	char* keys;
	char* oldkeys;
	void keyListener(){
		int keyNo[10] = {27, 81, 68, 77, 86, 49, 50, 51, 52, 32};
		printf("Start Key Listener\n");
		while(true){
			for (int y = 0; y < 10; y++){
				int x = keyNo[y];
				keys[x] = (char)(GetAsyncKeyState(x) >> 8);
				if((int)keys[x]!=(int)oldkeys[x]){
					if(keys[x]){
						switch(x){
						case 27: // ESC or q pressed
						case 81:
							if(control.verbose)
								printf("Quit program\n");
							control.running = false;
							break;
						case 68: // d pressed
							if(control.verbose)
								printf("Drawing turned on/off\n");
							control.drawImage = !control.drawImage;
							break;
						case 77: // m pressed
							printf("Music turned on/off\n");
							control.music = !control.music;
							break;
						case 86: // v pressed
							printf("Verbose turned on/off\n");
							control.verbose = !control.verbose;
							break;
						case 49: // 1 pressed
							if(control.verbose)
								printf("Depth turned on/off\n");
							control.depth = !control.depth;
							break;
						case 50: // 2 pressed
							if(control.verbose)
								printf("Color turned on/off\n");
							control.color = !control.color;
							break;
						case 51: // 3 pressed
							if(control.verbose)
								printf("Texture turned on/off\n");
							control.texture = !control.texture;
							break;	
						case 52: // 4 pressed
							if(control.verbose)
								printf("Saturation turned on/off\n");
							control.saturation = !control.saturation;
							break;
						case 32: // Space pressed
							control.screenshot = true;
							break;
						default:
							break;
						}

					}
				}
				oldkeys[x] = keys[x];
			}

			/*for (int x = 0; x < 256; x++){
				keys[x] = (char)(GetAsyncKeyState(x) >> 8);
				if((int)keys[x]!=(int)oldkeys[x]){
					std::cout << x << ":\t " << (bool)keys[x] << std::endl;
				}
				oldkeys[x] = keys[x];
			}*/
			Sleep(10);
		}
	}

	audio::audioEngine AE;
	controls control;

	// OpenCV images
	cv::Mat frameRGB;
	cv::Mat frameBGR;
	cv::Mat frameD;
	cv::Mat frameHSV;
	cv::Mat frameS;
	float constant_c;
	
	bool newImages;
	bool converting;

protected:
	void retrieveImages();
	int screenNo;

	void rgbd_cb_ ( const boost::shared_ptr<openni_wrapper::Image>      &image_in, 
		const boost::shared_ptr<openni_wrapper::DepthImage> &depth_in, 
		float constant_in);

};

#endif