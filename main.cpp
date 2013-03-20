/* footsteps.c
*
* To compile:
*   gcc -o footsteps footsteps.c -lopenal
*
* Requires data "footsteps.raw", which is any signed-16bit
* mono audio data (no header!); assumed samplerate is 44.1kHz.
*
*/

#include <stdlib.h>
#include <stdio.h>
//#include <unistd.h>  /* for usleep() */
#include <math.h>    /* for sqrtf() */
#include <time.h>    /* for time(), to seed srand() */

/* OpenAL headers */
#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alext.h>

#include <windows.h>

using namespace std;

/* load a file into memory, returning the buffer and
* setting bufsize to the size-in-bytes */
void* load( char *fname, long *bufsize ){
	FILE* fp = fopen( fname, "rb" );
	fseek( fp, 0L, SEEK_END );
	long len = ftell( fp );
	rewind( fp );
	void *buf = malloc( len );
	fread( buf, 1, len, fp );
	fclose( fp );
	*bufsize = len;
	return buf;
}


int main( int argc, char *argv[] ){
	/* current position*/
	float curr[3] = {0.,0.,0.};

	/* initialize OpenAL context, asking for 44.1kHz to match HRIR data */
	ALCint contextAttr[] = {ALC_FREQUENCY,44100,0};
	ALCdevice* device = alcOpenDevice( NULL );
	ALCcontext* context = alcCreateContext( device, contextAttr );
	alcMakeContextCurrent( context );

	/* listener at origin, facing down -z (ears at 1.5m height) */
	alListener3f( AL_POSITION, 0., 1.5, 0. );
	alListener3f( AL_VELOCITY, 0., 0., 0. );
	float ori[6] = {0., 0., -1. , 0., 1., 0.};
	alListenerfv( AL_ORIENTATION, ori );

	/* this will be the source of ghostly footsteps... */

	ALuint source;
	alGenSources( 1, &source );
	alSourcef( source, AL_PITCH, 1. );
	alSourcef( source, AL_GAIN, 1. );
	alSource3f( source, AL_POSITION, curr[0],curr[1],curr[2] );
	alSource3f( source, AL_VELOCITY, 0.,0.,0. );
	alSourcei( source, AL_LOOPING, AL_TRUE );

	/* allocate an OpenAL buffer and fill it with monaural sample data */
	ALuint buffer;
	alGenBuffers( 1, &buffer );
	{
		long dataSize;
		const ALvoid* data = load( "footsteps.raw", &dataSize );
		/* for simplicity, assume raw file is signed-16b at 44.1kHz */
		alBufferData( buffer, AL_FORMAT_MONO16, data, dataSize, 44100 );
		free( (void*)data );
	}
	alSourcei( source, AL_BUFFER, buffer );

	/** BEGIN! **/
	alSourcePlay( source );

	/* loop forever... walking to random, adjacent, integer coordinates */
	float x = 0.0;
	curr[0] = 0.0;
	curr[1] = 0.;
	curr[2] = 0.0;
	float lastx=0., lasty=0., lastz=0.;
	float v[3] = {0., 0., 0.};
	printf("%s \n%s \n ",alGetString(AL_VERSION), alGetString(AL_RENDERER));
	fflush(stderr);

	for(int i =0 ; i<50; i++){
		x+=0.2;

		lastx = curr[0];
		lasty = curr[1];
		lastz = curr[2];

		curr[0] = cos(x)*3;
		curr[2] = sin(x)*3;
		
		v[0] = curr[0]-lastx;
		v[1] = curr[1]-lasty;
		v[2] = curr[2]-lastz;

		printf("Pos: %.1f, %.1f, %.1f\n",curr[0],curr[1],curr[2]);

		alSource3f( source, AL_POSITION, curr[0],curr[1],curr[2] );
		alSource3f( source, AL_VELOCITY, v[0], v[1], v[2] );

		Sleep( (int)(600) );

	}

	/* cleanup that should be done when you have a proper exit... ;) */
	alDeleteSources( 1, &source );
	alDeleteBuffers( 1, &buffer );
	alcDestroyContext( context );
	alcCloseDevice( device );

	return 0;
}