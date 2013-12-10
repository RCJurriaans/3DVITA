#include "audioEngine.h"

namespace audio{

	audioEngine::audioEngine(){
		/* initialize OpenAL context, asking for 44.1kHz to match HRIR data */
		ALCint contextAttr[] = {ALC_FREQUENCY, 44100, 0};
		device = alcOpenDevice( NULL );
		context = alcCreateContext( device, contextAttr );
		alcMakeContextCurrent( context );

		const char *defname = alcGetString(NULL, ALC_DEFAULT_DEVICE_SPECIFIER);
		std::cout << "Default device: " << defname << std::endl;

		/* listener at origin, facing down -z (ears at 20cm below camera) */
		alListener3f( AL_POSITION, 0.0f, -0.2f, 0.0f );
		alListener3f( AL_VELOCITY, 0., 0., 0. );
		float ori[6] = {0., 0., -1. , 0., 1., 0.};
		alListenerfv( AL_ORIENTATION, ori);
	}

	audioEngine::~audioEngine(){
		alcDestroyContext( context );
		alcCloseDevice( device );
	}

	void audioEngine::createSound(audioParams params, float loc[3]){
		audioObject tmp;

		short* audioData = new short[44100];
		//float location[3] = {0,0,3};
		tmp.location[0] = loc[0];
		tmp.location[1] = loc[1];
		tmp.location[2] = loc[2];

		//std::cout << params.freqMat << std::endl;
		//std::cout << sum(params.freqMat)[0] << std::endl;
#ifdef FREQHISTO
		cv::Mat tmpFreqs = cv::Mat(1, 6, CV_32F);
		params.freqMat.convertTo(tmpFreqs, CV_32F);
		tmpFreqs = tmpFreqs / (float)(sum(params.freqMat)[0]);
#endif
		float compress = (params.dampThres*0.6)+0.4;

		// Create the sound :O
		for(int i = 0; i<44100 ; i++){
			// Sine Wave
#ifndef FREQHISTO
			//audioData[i] = (5000 * sin( (2*M_PI*params.freq)/44100 * (float)i));
#endif
			// Overtones
#ifdef FREQHISTO
			audioData[i] = 0;
			if(tmpFreqs.at<float>(0,0)>0.2)
				audioData[i] += 5000* sin((2*M_PI*392.f)/44100.f * (float)i);
			if(tmpFreqs.at<float>(0,1)>0.2)
				audioData[i] += 5000* sin((2*M_PI*440.f)/44100.f * (float)i);
			if(tmpFreqs.at<float>(0,2)>0.2)
				audioData[i] += 5000* sin((2*M_PI*466.f)/44100.f * (float)i);
			if(tmpFreqs.at<float>(0,3)>0.2)
				audioData[i] += 5000* sin((2*M_PI*523.f)/44100.f * (float)i);
			if(tmpFreqs.at<float>(0,4)>0.2)
				audioData[i] += 5000* sin((2*M_PI*587.f)/44100.f * (float)i);
			if(tmpFreqs.at<float>(0,5)>0.2)
				audioData[i] += 5000* sin((2*M_PI*698.f)/44100.f * (float)i);
#endif
			// Textured

			// Cut-off
#ifdef FREQHISTO
			int maxVal = (tmpFreqs.at<float>(0,0)>0.2)+(tmpFreqs.at<float>(0,1)>0.2)+(tmpFreqs.at<float>(0,2)>0.2)+
				(tmpFreqs.at<float>(0,3)>0.2)+(tmpFreqs.at<float>(0,4)>0.2)+(tmpFreqs.at<float>(0,5)>0.2);
			maxVal *= 5000;
#else
			int maxVal = 5000;
#endif

			//if(audioData[i] > maxVal-((maxVal/3)))
		//		audioData[i] = maxVal+((maxVal/3)*log(compress));
			


		}
#ifdef FREQHISTO
		tmpFreqs.release();
#endif
		// Create a source	
		alGenSources( 1, &tmp.source );
		alSourcef(  tmp.source, AL_PITCH, 1. );
		alSourcef(  tmp.source, AL_GAIN, 1. );
		alSource3f( tmp.source, AL_POSITION, tmp.location[0], tmp.location[1], tmp.location[2] );
		alSource3f( tmp.source, AL_VELOCITY, 0.,0.,0. );
		alSourcei(  tmp.source, AL_LOOPING, AL_FALSE );

		alGenBuffers( 1, &tmp.buffer );
		//audioData = data;
		
		alBufferData( tmp.buffer, AL_FORMAT_MONO16, audioData, 44100, 44100 );
		alSourcei( tmp.source, AL_BUFFER,  tmp.buffer );	
		sounds.push_back(tmp);
		delete [] audioData;
	}

	void audioEngine::playSounds(){
		std::vector<audioObject>::iterator it = sounds.begin();
		for( ; it!=sounds.end(); it++){
			alSourcePlay( it->source );
		}
		cv::waitKey(330);
		sounds.clear();
	}











}