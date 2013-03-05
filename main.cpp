#include <cstdlib>
#include <iostream>
#include <windows.h>
#include <AL/al.h>
#include <AL/alc.h>

using namespace std;

int endWithError(char* msg, int error=0)
{
    //Display error message in console
    cout << msg << "\n";
    system("PAUSE");
    return error;
}

int main(int argc, char *argv[])
{  
    //Loading of the WAVE file
    FILE *fp = NULL;                                                            //Create FILE pointer for the WAVE file
    fp=fopen("WAVE/Sound.wav","rb");                                            //Open the WAVE file
    if (!fp) return endWithError("Failed to open file");                        //Could not open file
    
    //Variables to store info about the WAVE file (all of them is not needed for OpenAL)
    char type[4];
    DWORD size,chunkSize;
    short formatType,channels;
    DWORD sampleRate,avgBytesPerSec;
    short bytesPerSample,bitsPerSample;
    DWORD dataSize;
    
    //Check that the WAVE file is OK
    fread(type,sizeof(char),4,fp);                                              //Reads the first bytes in the file
    if(type[0]!='R' || type[1]!='I' || type[2]!='F' || type[3]!='F')            //Should be "RIFF"
    return endWithError ("No RIFF");                                            //Not RIFF

    fread(&size, sizeof(DWORD),1,fp);                                           //Continue to read the file
    fread(type, sizeof(char),4,fp);                                             //Continue to read the file
    if (type[0]!='W' || type[1]!='A' || type[2]!='V' || type[3]!='E')           //This part should be "WAVE"
    return endWithError("not WAVE");                                            //Not WAVE
    
    fread(type,sizeof(char),4,fp);                                              //Continue to read the file
    if (type[0]!='f' || type[1]!='m' || type[2]!='t' || type[3]!=' ')           //This part should be "fmt "
    return endWithError("not fmt ");                                            //Not fmt 
    
    //Now we know that the file is a acceptable WAVE file
    //Info about the WAVE data is now read and stored
    fread(&chunkSize,sizeof(DWORD),1,fp);
    fread(&formatType,sizeof(short),1,fp);
    fread(&channels,sizeof(short),1,fp);
    fread(&sampleRate,sizeof(DWORD),1,fp);
    fread(&avgBytesPerSec,sizeof(DWORD),1,fp);
    fread(&bytesPerSample,sizeof(short),1,fp);
    fread(&bitsPerSample,sizeof(short),1,fp);
    
    fread(type,sizeof(char),4,fp);
    if (type[0]!='d' || type[1]!='a' || type[2]!='t' || type[3]!='a')           //This part should be "data"
    return endWithError("Missing DATA");                                        //not data
    
    fread(&dataSize,sizeof(DWORD),1,fp);                                        //The size of the sound data is read
    
    //Display the info about the WAVE file
    cout << "Chunk Size: " << chunkSize << "\n";
    cout << "Format Type: " << formatType << "\n";
    cout << "Channels: " << channels << "\n";
    cout << "Sample Rate: " << sampleRate << "\n";
    cout << "Average Bytes Per Second: " << avgBytesPerSec << "\n";
    cout << "Bytes Per Sample: " << bytesPerSample << "\n";
    cout << "Bits Per Sample: " << bitsPerSample << "\n";
    cout << "Data Size: " << dataSize << "\n";
        
    unsigned char* buf= new unsigned char[dataSize];                            //Allocate memory for the sound data
    cout << fread(buf,sizeof(BYTE),dataSize,fp) << " bytes loaded\n";           //Read the sound data and display the 
                                                                                //number of bytes loaded.
                                                                                //Should be the same as the Data Size if OK
    

    //Now OpenAL needs to be initialized 
    ALCdevice *device;                                                          //Create an OpenAL Device
    ALCcontext *context;                                                        //And an OpenAL Context
    device = alcOpenDevice(NULL);                                               //Open the device
    if(!device) return endWithError("no sound device");                         //Error during device oening
    context = alcCreateContext(device, NULL);                                   //Give the device a context
    alcMakeContextCurrent(context);                                             //Make the context the current
    if(!context) return endWithError("no sound context");                       //Error during context handeling

    ALuint source;                                                              //Is the name of source (where the sound come from)
    ALuint buffer;                                                           //Stores the sound data
    ALuint frequency=sampleRate;;                                               //The Sample Rate of the WAVE file
    ALenum format=0;                                                            //The audio format (bits per sample, number of channels)
    
    alGenBuffers(1, &buffer);                                                    //Generate one OpenAL Buffer and link to "buffer"
    alGenSources(1, &source);                                                   //Generate one OpenAL Source and link to "source"
    if(alGetError() != AL_NO_ERROR) return endWithError("Error GenSource");     //Error during buffer/source generation
    
    //Figure out the format of the WAVE file
    if(bitsPerSample == 8)
    {
        if(channels == 1)
            format = AL_FORMAT_MONO8;
        else if(channels == 2)
            format = AL_FORMAT_STEREO8;
    }
    else if(bitsPerSample == 16)
    {
        if(channels == 1)
            format = AL_FORMAT_MONO16;
        else if(channels == 2)
            format = AL_FORMAT_STEREO16;
    }
    if(!format) return endWithError("Wrong BitPerSample");                      //Not valid format

    alBufferData(buffer, format, buf, dataSize, frequency);                    //Store the sound data in the OpenAL Buffer
    if(alGetError() != AL_NO_ERROR) 
    return endWithError("Error loading ALBuffer");                              //Error during buffer loading
  
    //Sound setting variables
    ALfloat SourcePos[] = { 0.0, 0.0, 0.0 };                                    //Position of the source sound
    ALfloat SourceVel[] = { 0.0, 0.0, 0.0 };                                    //Velocity of the source sound
    ALfloat ListenerPos[] = { 0.0, 0.0, 0.0 };                                  //Position of the listener
    ALfloat ListenerVel[] = { 0.0, 0.0, 0.0 };                                  //Velocity of the listener
    ALfloat ListenerOri[] = { 0.0, 0.0, -1.0,  0.0, 1.0, 0.0 };                 //Orientation of the listener
                                                                                //First direction vector, then vector pointing up) 
    //Listener                                                                               
    alListenerfv(AL_POSITION,    ListenerPos);                                  //Set position of the listener
    alListenerfv(AL_VELOCITY,    ListenerVel);                                  //Set velocity of the listener
    alListenerfv(AL_ORIENTATION, ListenerOri);                                  //Set orientation of the listener
    
    //Source
    alSourcei (source, AL_BUFFER,   buffer);                                 //Link the buffer to the source
    alSourcef (source, AL_PITCH,    1.0f     );                                 //Set the pitch of the source
    alSourcef (source, AL_GAIN,     1.0f     );                                 //Set the gain of the source
    alSourcefv(source, AL_POSITION, SourcePos);                                 //Set the position of the source
    alSourcefv(source, AL_VELOCITY, SourceVel);                                 //Set the velocity of the source
    alSourcei (source, AL_LOOPING,  AL_FALSE );                                 //Set if source is looping sound
    
    //PLAY 
    alSourcePlay(source);                                                       //Play the sound buffer linked to the source
    if(alGetError() != AL_NO_ERROR) return endWithError("Error playing sound"); //Error when playing sound
    system("PAUSE");                                                            //Pause to let the sound play
    
    //Clean-up
    fclose(fp);                                                                 //Close the WAVE file
    delete[] buf;                                                               //Delete the sound data buffer
    alDeleteSources(1, &source);                                                //Delete the OpenAL Source
    alDeleteBuffers(1, &buffer);                                                 //Delete the OpenAL Buffer
    alcMakeContextCurrent(NULL);                                                //Make no context current
    alcDestroyContext(context);                                                 //Destroy the OpenAL Context
    alcCloseDevice(device);                                                     //Close the OpenAL Device
    
    return EXIT_SUCCESS;                                                        
}


