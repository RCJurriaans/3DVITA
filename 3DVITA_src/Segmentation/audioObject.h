#ifndef AUDIOOBJECT_H
#define AUDIOOBJECT_H

//#include <stdlib.h>
#include <stdio.h>
#include <iostream>

/* Math Headers */
#define _USE_MATH_DEFINES
#include <math.h>    /* for sqrtf() */

/* OpenAL headers */
#include <al.h>
#include <alc.h>
#include <alext.h>

#include "audiogenerator.h"

class audioObject 
{
public:
	struct Parameters {
		float overtone_amps[5]; 
		float freq;
		float targetlength;
		float strength;
		float attack;
		float decay;
		float release;
		float sustain;
		bool overtones;
		bool noise;
	};

	Parameters params;

	void setParams(	float overtone_amps[5], float freq=440.f, float targetlength=1.0, float strength=1,	
		float attack=0.2, float decay=0.5, float release=0.75, float sustain=0.5, bool overtones=true, bool noise=false){
			for (int i=0; i<5; i++){
				params.overtone_amps[i] = overtone_amps[i];
			}
			params.attack = attack;
			params.decay = decay;
			params.freq = freq;
			params.noise = noise;
			params.overtones = overtones;
			params.release = release;
			params.strength = strength;
			params.sustain = sustain;
			params.targetlength = targetlength;
	}

	int frames;
	short *audioData;
	float location[3];
	float frequency;

	ALuint source;

	// Allocate buffers
	ALuint buffer;

	audioObject();
	audioObject(short *audioData);
	//AudioSource(float loc[3], float frequency);
	~audioObject();

	void playSound();
	void updateSound(float loc[3], short *data);

private:


}; 

#endif