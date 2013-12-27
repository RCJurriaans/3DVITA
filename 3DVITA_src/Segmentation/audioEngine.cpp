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

		tmp.data = new short[44100];

		int amplitude = 9000;

		//short* audioData = new short[44100];
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

		// Threshold notes
		cv::threshold( tmpFreqs, tmpFreqs, 0.25, 1, 3 );
		tmpFreqs = tmpFreqs / (float)(sum(params.freqMat)[0]);

		double minVal, maxVal;
		cv::minMaxLoc(tmpFreqs, &minVal, &maxVal);
		tmpFreqs = tmpFreqs/maxVal;

#endif
		int j = 0;
		// Create the sound :O
		for(int i = 0; i<44100 ; i++){
			// Sine Wave
			float bright = params.brightness;
#ifndef FREQHISTO
			//audioData[i] = (amplitude * sin( (2*M_PI* bright*params.freq)/44100 * (float)i));
#endif
			// Overtones
#ifdef FREQHISTO
			tmp.data[i] = 0;
			//if(tmpFreqs.at<float>(0,0)>0.26)
				tmp.data[i] += (tmpFreqs.at<float>(0,0)*amplitude)* sin((2*M_PI*(bright*392.f))/44100.f * (float)i);
			//if(tmpFreqs.at<float>(0,1)>0.26)
				tmp.data[i] += (tmpFreqs.at<float>(0,1)*amplitude)* sin((2*M_PI*(bright*440.f))/44100.f * (float)i);
			//if(tmpFreqs.at<float>(0,2)>0.26)
				tmp.data[i] += (tmpFreqs.at<float>(0,2)*amplitude)* sin((2*M_PI*(bright*466.f))/44100.f * (float)i);
			//if(tmpFreqs.at<float>(0,3)>0.26)
				tmp.data[i] += (tmpFreqs.at<float>(0,3)*amplitude)* sin((2*M_PI*(bright*523.f))/44100.f * (float)i);
			//if(tmpFreqs.at<float>(0,4)>0.26)
				tmp.data[i] += (tmpFreqs.at<float>(0,4)*amplitude)* sin((2*M_PI*(bright*587.f))/44100.f * (float)i);
			//if(tmpFreqs.at<float>(0,5)>0.26)
				tmp.data[i] += (tmpFreqs.at<float>(0,5)*amplitude)* sin((2*M_PI*(bright*698.f))/44100.f * (float)i);
#endif
			// Textured
			float tst = (float)(rand()%10000)-5000.f;
			//std::cout << params.texture << " " << tst << std::endl;
			tmp.data[i] += (params.texture * tst);
			// Cut-off
#ifdef FREQHISTO
			int maxVal = (tmpFreqs.at<float>(0,0)>0.2)+(tmpFreqs.at<float>(0,1)>0.2)+(tmpFreqs.at<float>(0,2)>0.2)+
				(tmpFreqs.at<float>(0,3)>0.2)+(tmpFreqs.at<float>(0,4)>0.2)+(tmpFreqs.at<float>(0,5)>0.2);
			maxVal *= amplitude;

			maxVal = amplitude;
#else
			int maxVal = amplitude;
#endif
			// Low Pass Filter
			float RC = ((1-params.dampThres)*150)/44100.f;
			float dt = 1/44100.f;
			float alpha = dt/(RC+dt);
			//	printf("RC = %f \t dt = %f \t alpha = %f\n", RC, dt, alpha);
			if(i>0){
				tmp.data[i] = alpha*tmp.data[i] + (1-alpha)*tmp.data[i-1];
			}

			// ADRS
			// Multiply sinewave with ADRS function
			float ADRS;
			if(i<44100*params.attack){
				ADRS = 1 - (pow((44100*params.attack) - i, 2)/pow( 44100*params.attack, 2));
			}
			else if(i<44100*params.decay){
				j = i-44100*params.attack;
				ADRS = params.sustain + (1-(params.sustain)) *(( pow( 44100*params.decay-j, 2)/pow(44100*params.decay,2) ));
			}
			else if(i<44100*params.release){
				j = i-44100*params.decay;
				ADRS = params.sustain * (pow( 44100*params.release-j, 2)/pow(44100*params.release,2) );
			}
			else{
				ADRS=0;
			}
			tmp.data[i] *= ADRS;
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

		alBufferData( tmp.buffer, AL_FORMAT_MONO16, tmp.data, 44100, 44100 );
		alSourcei( tmp.source, AL_BUFFER,  tmp.buffer );	
		//tmp.data = audioData;
		sounds.push_back(tmp);
		//delete [] audioData;
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