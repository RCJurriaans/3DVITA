#include "SoundManager.h"

using namespace std;

SoundManager::SoundManager(void)
{
	/* initialize OpenAL context, asking for 44.1kHz to match HRIR data */
	ALCint contextAttr[] = {ALC_FREQUENCY,44100,0};
	device = alcOpenDevice( NULL );
	context = alcCreateContext( device, contextAttr );
	alcMakeContextCurrent( context );

	const char *defname = alcGetString(NULL, ALC_DEFAULT_DEVICE_SPECIFIER);
    cout << "Default device: " << defname << endl;


	/* listener at origin, facing down -z (ears at 1.5m height) */
	alListener3f( AL_POSITION, 0., 1.5, 0. );
	alListener3f( AL_VELOCITY, 0., 0., 0. );
	float ori[6] = {0., 0., -1. , 0., 1., 0.};
	alListenerfv( AL_ORIENTATION, ori );

}

SoundManager::~SoundManager(void)
{
	alcDestroyContext( context );
	alcCloseDevice( device );
}

void SoundManager::flushBuffer(void)
{


}