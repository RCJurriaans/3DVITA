#include "StdAfx.h"
#include "SoundManager.h"

using namespace std;



int main( int argc, char *argv[] )
{
	SoundManager Sound;

	float location[3] = {0., 0., 2.};
	float frequency;
	float r[3] = {0., 0., 0.};

	while(true){
		frequency = rand() %479 +261;
		r[1] = rand()%5 -2;
		//r[2] = rand()%5 -2;
		r[3] = rand()%5 -2;
		cout << r[1] << r[2] << r[3] << endl;
		AudioSource test = AudioSource(r, frequency);

		test.play();
		Sleep(150);
	}

	return 0;
}