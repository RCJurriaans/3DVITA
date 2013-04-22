/* OpenAL headers */
#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alext.h>

class AudioSource
{
public:
	AudioSource(float location[3], float frequency);
	~AudioSource(void);
	void play(void);

private:
	int frames;


	short audioData[44100];

	float location[3];
	float velocity[3];

	ALuint source;

	// Allocate buffers
	ALuint buffer;
	void createBuffer(float frequency);



};