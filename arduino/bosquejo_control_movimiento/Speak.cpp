#include "Speak.h"

Voice::Voice(int sp, int lp) : SPEAKER_PIN(sp), LED_PIN(lp) {}

void Voice::begin()
{
	pinMode(SPEAKER_PIN, OUTPUT);
	pinMode(LED_PIN, OUTPUT);
}

void Voice::whistle(int baseFreq, bool up, int timesFirst, int timesSecond)
{
	int dir = (up) ? 1 : -1;
	digitalWrite(LED_PIN, HIGH);
	for (int i = 0; i <= timesFirst; i++){
		tone(SPEAKER_PIN, baseFreq + (-dir * i * 2));
		delay(random(.9, 2));
	}
	digitalWrite(LED_PIN, LOW);
	for (int i = 0; i <= timesSecond; i++){
		tone(SPEAKER_PIN, baseFreq + (dir * i * 10));
		delay(random(.9, 2));
	}
}

void Voice::phrase(bool up)
{
	int baseFreq;
	int timesFirst;
	int timesSecond;

	if (up) {
		baseFreq = random(700,1000);
		timesFirst = random(100,400);
		timesSecond = random(100,1000);
	} else {
		baseFreq = random(2000,3000);
		timesFirst = random(200,1000);;
		timesSecond = random(50,150);
	}
	whistle(baseFreq, up, timesFirst, timesSecond);
}

void Voice::babble()
{
	int K = 2000;
	for (int i = 0; i <= random(3, 9); i++){
		digitalWrite(LED_PIN, HIGH);
		tone(SPEAKER_PIN, K + random(-1700, 0));
		delay(random(70, 170));
		digitalWrite(LED_PIN, LOW);
		noTone(SPEAKER_PIN);
		delay(random(0, 30));     
	}
}

void Voice::randomPhrase()
{
	switch (random(1,7))
	{
		case 1:phrase(true); break;
		case 2:phrase(false); break;
		case 3:phrase(true); phrase(false); break;
		case 4:phrase(true); phrase(false); phrase(true);break;
		case 5:phrase(true); phrase(false); phrase(true); phrase(false); phrase(true);break;
		case 6:phrase(false); phrase(true); phrase(false); break;
	}
	babble();
	noTone(SPEAKER_PIN);
	delay(random(2000, 4000));
}

