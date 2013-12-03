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

		// Create the sound :O
		for(int i = 0; i<44100 ; i++){
			// Sine Wave
			audioData[i] = (5000 * sin( (2*M_PI*params.freq)/44100 * (float)i));
			// Overtones

			// Textured

			// Cut-off
		}

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