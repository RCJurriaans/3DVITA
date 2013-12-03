#include "audioObject.h"

audioObject::audioObject(){

	audioData = new short[44100];
	audioComponents AC = audioComponents();

	float freq = 440.0f;
	short *data = new short[44100];
	AC.defaultSound(data, freq);

	float location[3] = {0,0,3};

	// Create a source	
	alGenSources( 1, &source );
	alSourcef( source, AL_PITCH, 1. );
	alSourcef( source, AL_GAIN, 1. );
	alSource3f( source, AL_POSITION, location[0],location[1],location[2] );
	alSource3f( source, AL_VELOCITY, 0.,0.,0. );
	alSourcei( source, AL_LOOPING, AL_FALSE );

	alGenBuffers( 1, &buffer );
	//audioData = data;
	std::copy(data, data+44100, audioData);
	//audioData = new short(*data);
	alBufferData( buffer, AL_FORMAT_MONO16, audioData, 44100, 44100 );
	alSourcei( source, AL_BUFFER, buffer );	

	delete [] data;
	data = NULL;

	AC.~audioComponents();
}

audioObject::audioObject(short *data)
{
	audioData = new short[44100];
	float location[3] = {0,0,3};

	// Create a source	
	alGenSources( 1, &source );
	alSourcef( source, AL_PITCH, 1. );
	alSourcef( source, AL_GAIN, 1. );
	alSource3f( source, AL_POSITION, location[0], location[1], location[2] );
	alSource3f( source, AL_VELOCITY, 0.,0.,0. );
	alSourcei( source, AL_LOOPING, AL_FALSE );

	alGenBuffers( 1, &buffer );
	//audioData = data;
	
	std::copy(data, data+44100, audioData);
	alBufferData( buffer, AL_FORMAT_MONO16, audioData, 44100, 44100 );
	alSourcei( source, AL_BUFFER, buffer );	

}

audioObject::~audioObject()
{
	alDeleteSources( 1, &source );
	alDeleteBuffers( 1, &buffer );
	delete [] audioData;
	audioData = NULL;
}

void audioObject::updateSound(float loc[3], short *data)
{
std::copy(data, data+44100, audioData);
	//audioData = data;
	location[0] = loc[0];
	location[1] = loc[1];
	location[2] = loc[2];
	alSource3f( source, AL_POSITION, loc[0],loc[1],loc[2] );
	alGenBuffers( 1, &buffer );
	alBufferData( buffer, AL_FORMAT_MONO16, data, 44100, 44100 );
	alSourcei( source, AL_BUFFER, buffer );	
}

void audioObject::playSound()
{
	alSourcePlay( source );
}


/*
audioObject::audioObject(short *data){

float location[3] = {0,0,3};

// Convert vector to short array
audioData = new short[sizeof(data)];
copy(data->begin(), data->end(), audioData);

// Create a source	
alGenSources( 1, &source );
alSourcef( source, AL_PITCH, 1. );
alSourcef( source, AL_GAIN, 1. );
alSource3f( source, AL_POSITION, location[0],location[1],location[2] );
alSource3f( source, AL_VELOCITY, 0.,0.,0. );
alSourcei( source, AL_LOOPING, AL_FALSE );

alGenBuffers( 1, &buffer );
alBufferData( buffer, AL_FORMAT_MONO16, &audioData, 44100, 44100 );
alSourcei( source, AL_BUFFER, buffer );	

}

void audioObject::playSound(){
alSourcePlay( source );
}

void audioObject::updateSound(){


}
*/