/* OpenAL headers */
#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alext.h>

#include <vector>
#include "AudioSource.h"

class SoundManager
{
public:
	SoundManager(void);
	~SoundManager(void);

private:
	// OpenAL variables
	ALCdevice* device;
	ALCcontext* context;
	std::vector<AudioSource> sources;

	// Create the context where all the sounds can be played
	void createContext(void);

	// Generate a sound based on a set of features
	void generateSound(void);

	// Remove all buffers
	void flushBuffer(void);


	
};