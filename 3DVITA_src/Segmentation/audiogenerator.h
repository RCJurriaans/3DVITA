#ifndef AUDIOGEN_H
#define AUDIOGEN_H

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

#include <vector>
//#include "audioObject.h"
//class audioObject;

class audioComponents{
private:
	// OpenAL variables
	ALCdevice* device;
	ALCcontext* context;

public:
	float samplerate;
	float amplitude;

	audioComponents();
	~audioComponents();

	void createSoundEngine();
	void destroySoundEngine();

	void defaultSound(short *src);
	void defaultSound(short *src, float frequency);

	void createSound(short *src, float freq);

	void generateSine	(short *src,	float frequency=440, float targetlength=1.0);
	void generateSquare	(short *src,	float frequency=440, float targetlength=1.0);
	void generateSaw	(short *src,	float frequency=440, float targetlength=1.0);
	void generateNoise	(short *src,	float strength,		 float targetlength=1.0);

	void addNoise(short *src, float strength=1);
	void addOvertones(short *src, float fundamental_frequency);

	void adrs(short *src, float attack=0.025, float decay=0.25, float release=0.45, float sustain=0.65);

	void additive(short *input, short *add, short *output);
	void subtractive(short *input, short *min, short *output);
};

#endif