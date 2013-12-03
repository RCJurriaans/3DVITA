#include "audiogenerator.h"

	audioComponents::audioComponents(){
		samplerate = 44100;
		amplitude = 1000;
	}

	audioComponents::~audioComponents(){

	}
	
	void audioComponents::destroySoundEngine(){
		alcDestroyContext( context );
		alcCloseDevice( device );
	}

	void audioComponents::createSoundEngine(){
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

	}

	void audioComponents::defaultSound(short *src){
		generateSine(src);
		addOvertones(src, 440.f);
		adrs(src);
	}

	void audioComponents::defaultSound(short *src, float frequency){
		generateSine(src, frequency, 1.0f);
		addOvertones(src, frequency);
		addNoise(src, 1000);
		adrs(src);
	}

	void audioComponents::generateSine(short *src, float frequency, float targetlength){
		float length = 44100;//(int)(samplerate*targetlength);
		for(int i =0; i<length; i++)
			src[i] = (amplitude * sin( (2*M_PI*frequency)/samplerate * (float)i));

	}
	void audioComponents::generateSquare(short *src, float frequency, float targetlength){
		float length = 44100;//(int)(samplerate*targetlength);
		for(int i =0; i<length; i++)
			src[i] = (amplitude * ( -1 + 2*(sin((2*M_PI*frequency)/samplerate * (float)i)>0) ));

	}
	void audioComponents::generateSaw(short *src, float frequency, float targetlength){
		float length = 44100;//(int)(samplerate*targetlength);

	}
	void audioComponents::generateNoise(short *src, float strength, float targetlength){
		float length = 44100;//(int)(samplerate*targetlength);
		float randN = 0;
		for(int i =0; i<length; i++){
			randN = ((double) rand() / (RAND_MAX));
			src[i] =  ( (2.0f*(randN) ) -1.0f) * strength;

		}
	}

	void audioComponents::addNoise(short *src, float strength){
		float frames = 44100;
		//short *noise = new short[44100];
		//generateNoise(noise, frames/samplerate, strength);
		//additive(src, noise, src);

		float randN = 0;
		for(int i =0; i<frames; i++){
			randN = ((double) rand() / (RAND_MAX));
			src[i] +=  ( (2.0f*(randN) ) -1.0f) * strength;
		}

		// Remove noise array
		//delete noise;
		//noise = NULL;
	}

	void audioComponents::addOvertones(short *src, float fundamental_frequency){
		int src_size = 44100;//sizeof(src)/sizeof(short);
		float overtone_amp[7] = {1, 0.8, 0.9, 0.01, 0.1, 0.05, 0.01};
		for(int overtone=2; overtone<5; overtone++){
			//short *overtone_src = new short[src_size];
			for(int i =0; i < src_size; i++)
				//overtone_src[i] = (amplitude*(overtone_amp[overtone]) * sin( (2*M_PI*(fundamental_frequency*overtone))/samplerate * (float)i));
				src[i] = src[i] + (amplitude*(overtone_amp[overtone]) * sin( (2*M_PI*(fundamental_frequency*overtone))/samplerate * (float)i));
				//				overtone_src[i] = (amplitude*(pow(0.55,overtone-1)) * sin( (2*M_PI*(fundamental_frequency*overtone))/samplerate * (float)i));
			//additive(src, overtone_src, src);

			// Remove overtone array
			//delete [] overtone_src;
			//overtone_src = NULL;
		}

	}


	void audioComponents::adrs(short *src, float attack, float decay, float release, float sustain){
		float frames = 44100;//sizeof(src)/sizeof(short);
		float ADRS;
		int j;
		for(int i =0; i < frames; i++){
			if(i<frames*attack){
				ADRS = 1 - (pow((frames*attack) - i, 2)/pow( frames*attack, 2));
			}
			else if(i<frames*decay){
				j = i-frames*attack;
				ADRS = sustain + (1-(sustain)) *(( pow( frames*decay-j, 2)/pow(frames*decay,2) ));
			}
			else if(i<frames*release){
				j = i-frames*decay;
				ADRS = sustain * (pow( frames*release-j, 2)/pow(frames*release,2) );
			}
			else{
				ADRS=0;
			}
			src[i] = ADRS * src[i];
		}
	}

	void audioComponents::additive(short *input, short *add, short *output){
		int input_size = 44100;//sizeof(src)/sizeof(short);
		for(int i = 0; i<input_size; i++)
			output[i] = input[i]+add[i];
	}
	void audioComponents::subtractive(short *input, short *min, short *output){
		int input_size = sizeof(&input)/sizeof(short);
		for(int i = 0; i<input_size; i++)
			output[i] = input[i]-min[i];
	}

	void audioComponents::createSound(short *src, float freq, float attack, float decay, float release, float sustain, float strength){
		generateSine(src, freq, 1.0f);
		addOvertones(src, freq);
		addNoise(src, strength);
		adrs(src, attack, attack+0.2, attack+0.2, sustain);
	}


