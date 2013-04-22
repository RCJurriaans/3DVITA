#include "AudioSource.h"

AudioSource::AudioSource(float location[3], float frequency)
{
	// Create a source	
	alGenSources( 1, &source );
	alSourcef( source, AL_PITCH, 1. );
	alSourcef( source, AL_GAIN, 1. );
	alSource3f( source, AL_POSITION, location[0],location[1],location[2] );
	alSource3f( source, AL_VELOCITY, 0.,0.,0. );
	alSourcei( source, AL_LOOPING, AL_TRUE );

	alGenBuffers( 1, &buffer );
	createBuffer(frequency);
	alBufferData( buffer, AL_FORMAT_MONO16, audioData, 44100, 44100 );
	
	alSourcei( source, AL_BUFFER, buffer );
}

AudioSource::~AudioSource()
{
	alDeleteSources( 1, &source );
	alDeleteBuffers( 1, &buffer );

}

void AudioSource::createBuffer(float frequency)
{
	AudioSource::frames = 44100;

	float B = (2.f * M_PI * frequency);

	for (int i=0; i<AudioSource::frames; i++)
	{
		
		//AudioSource::audioData[i] = 25000.f * sin( B/44100.f * (float)i);
		AudioSource::audioData[i] = 35000.f*sin( (frequency*M_PI)/44100.f * (float)i);
	}
	
}

void AudioSource::play()
{
	alSourcePlay( source );
}

	//public SineWave(double frequency, int seconds)
	//{
	//	this.buffer = AL.GenBuffer();
	//	this.source = AL.GenSource();
 //
	//	int frames = seconds * sampleRate;
	//	this.audioData = new short[frames];
	//	for (int i = 0; i < frames; i++)
	//	{
	//		this.audioData[i] = (short)(short.MaxValue * Math.Sin((2 * Math.PI * frequency) / sampleRate * i));
	//	}
	//}