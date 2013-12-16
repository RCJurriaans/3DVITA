#ifndef AUDIOENGINE_H
#define AUDIOENGINE_H

#include <stdio.h>
#include <iostream>

/* Math Headers */
#define _USE_MATH_DEFINES
#include <math.h>    /* for sqrtf() */

#define FREQHISTO

/* OpenAL headers */
#include <al.h>
#include <alc.h>
#include <alext.h>

/* OpenCV headers */
#include <cv.h>  
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>

namespace audio{

	struct audioParams{
#ifdef FREQHISTO
		cv::Mat freqMat;
#else
		float freq;
#endif
		float attack;
		float decay;
		float release;
		float sustain;
		float dampThres;
		float brightness;
		float texture;
		std::vector<float> overtone_amps; 
	};

	struct audioObject{
		ALuint source;
		ALuint buffer;
		float location[3];
	};


	class audioEngine{
	public:
		audioEngine();
		~audioEngine();

		void createSound(audioParams params, float loc[3]); 
		void playSounds();
		std::vector<audioObject> sounds;
	private:
		// OpenAL variables
		ALCdevice* device;
		ALCcontext* context;



	protected:




	};

}




#endif