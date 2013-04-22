#include "StdAfx.h"
#include "audioDemo\SoundManager.h"

using namespace std;

int main( int argc, char *argv[] )
{
	SoundManager Sound;

	float location[3] = {0., 0., 1.};
	float frequency;
	float note;
	float notes[8] = {40, 44, 47, 49, 50, 49,  47, 44};
	//float notes[8] = {40, 50, 49, 50, 44, 47,  44, 40};
	float blues[10] = {0, 0, 5, 5, 0, 0, 7, 5, 0, 0};
	float time;

	int i=0;
	int blue = 0;
	while(true){
		location[0] = rand()%5-2;
		location[2] = rand()%5-2;
		note = notes[i]+blues[blue];
		cout << "Note: " << note << endl;
		frequency = 440.f * pow(2, (( note-49) /12));
		
		AudioSource test = AudioSource(location, frequency);

		test.play();
		Sleep(150);
		

		i++;
		if(i>7){
			blue++;
			i=0;
		}
		if(blue>9)
			break;
	}

	return 0;
}