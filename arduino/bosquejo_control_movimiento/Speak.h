/***************************************************
  Project R2D2 Sound Generator
  https://www.instructables.com/R2D2-Sound-Generator/
  - Bocina 8 Ohms
 ***************************************************
 *
 * Speak.h - Library for speaking utterances
 * @author blackzafiro
 */
#ifndef Speak_h
#define Speak_h

#include "Arduino.h"

class Voice
{
public:
	Voice(int sp, int lp);

	void begin();

	void whistle(int baseFreq, bool up, int timesFirst, int timesSecond);

	void phrase(bool up);

	void babble();

	void randomPhrase();
private:
	const int SPEAKER_PIN;
	const int LED_PIN;
};

#endif

