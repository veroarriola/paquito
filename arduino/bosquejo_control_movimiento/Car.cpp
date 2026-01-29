#include "Arduino.h"
#include "Car.h"


// Wheel

Wheel::Wheel(unsigned int sp, unsigned int fp, unsigned int bp) : SPEED_PIN(sp), FORWARD_PIN(fp), BACKWARD_PIN(bp) {}

void Wheel::begin()
{
    pinMode(FORWARD_PIN, OUTPUT);
    pinMode(BACKWARD_PIN, OUTPUT);
    pinMode(SPEED_PIN, OUTPUT);

    stop();
}

void Wheel::moveForward(unsigned int speed)
{
    //printStatus("mF ");

    digitalWrite(FORWARD_PIN, HIGH);
    digitalWrite(BACKWARD_PIN, LOW);
    analogWrite(SPEED_PIN, speed);
}

void Wheel::moveBackward(unsigned int speed)
{
    //printStatus("mB");

    digitalWrite(FORWARD_PIN, LOW);
    digitalWrite(BACKWARD_PIN, HIGH);
    analogWrite(SPEED_PIN, speed);
}

void Wheel::setSignedSpeed(int16_t speed)
{
    if (speed > 0) {
        moveForward(constrain(speed, 0, 255));
    } else if (speed < 0) {
        moveBackward(constrain(-1 * speed, 0, 255));
    } else {
        stop();
    }
}

void Wheel::stop()
{
    //printStatus("S");
    analogWrite(SPEED_PIN, 0);
}

void Wheel::printStatus(String name)
{
    Serial.print(name);
    Serial.print(" ");
    Serial.print(FORWARD_PIN);
    Serial.print(" ");
    Serial.print(BACKWARD_PIN);
    Serial.print(" ");
    Serial.println(SPEED_PIN);
}

// Encoder
Encoder::Encoder(unsigned int s1, unsigned int s2) : S1_PIN(s1), S2_PIN(s2), _counter(0) {}

void Encoder::begin()
{
    pinMode (S1_PIN, INPUT);
    pinMode (S2_PIN, INPUT);

    reset_count();
}

void Encoder::reset_count()
{
    _counter = 0;
    _lastTime = millis();
    _s1LastState = digitalRead(S1_PIN);  // Estado incial del pin S1
}

void Encoder::read(int readings[2])
{
    readings[0] = digitalRead(S1_PIN);
    readings[1] = digitalRead(S2_PIN);
    return readings;
}

double Encoder::count()
{
    _s1State = digitalRead(S1_PIN);
    unsigned long time = millis();
    int delta = time - _lastTime;
    _lastTime = time;
    if(_s1State != _s1LastState)
    {
        _s2State = digitalRead(S2_PIN);
        // If S2 state is different to the S1 state, that means the encoder is rotating clockwise
        if (_s2State != _s1State)
        {
            _counter++;
        }
        else
        {
            _counter--;
        }
        _s1LastState = _s1State;  // Updates the previous S1 state with the current state
    }
    //return double(_counter)/double(time);
    return _counter;
}


// Car
Car::Car(const Wheel wheels[NUM_WHEELS], const Encoder encoders[NUM_WHEELS])
    : _wheels(wheels), _encoders(encoders){}

Car::Car(const Wheel wheels[NUM_WHEELS])
    : _wheels(wheels), _encoders(NULL){}

void Car::begin()
{
    for(int i = 0; i < NUM_WHEELS; i++)
    {
      // Inicializa llanta
      _wheels[i].begin();

      if(_encoders != NULL) {
        // Inicializa odómetro
        _encoders[i].begin();
      }
    }
 }

  // Mueve cada llanta una vez hacia adelante y hacia atrás.
void Car::testWheels()
{
    int delTime = 1000;
    int speed = 70;
    for(int i = 0; i < NUM_WHEELS; i++)
    {
        _wheels[i].moveForward(speed);
        delay(delTime);
        _wheels[i].stop();
        delay(delTime);
        _wheels[i].moveBackward(speed);
        delay(delTime);
        _wheels[i].stop();
        delay(delTime);
    }
}

void Car::moveForward(WheelId id, int speed)
{
    _wheels[id].moveForward(speed);
}

void Car::stop()
{
    for(int i = 0; i < NUM_WHEELS; i++)
    {
      _wheels[i].stop();
    }
}

void Car::stop(WheelId id)
{
    _wheels[id].stop();
}

void Car::moveForward(unsigned int speed)
{
    for(int i = 0; i < NUM_WHEELS; i++)
    {
        _wheels[i].moveForward(speed);
    }
}

void Car::moveBackward(unsigned int speed)
{
    for(int i = 0; i < NUM_WHEELS; i++)
    {
        _wheels[i].moveBackward(speed);
    }
}

void Car::moveLeft(unsigned int speed)
{
    _wheels[FR].moveForward(speed);
    _wheels[BL].moveForward(speed);
    _wheels[FL].moveBackward(speed);
    _wheels[BR].moveBackward(speed);
}

void Car::moveRight(unsigned int speed)
{
    _wheels[FL].moveForward(speed);
    _wheels[BR].moveForward(speed);
    _wheels[FR].moveBackward(speed);
    _wheels[BL].moveBackward(speed);
}

void Car::moveNW(unsigned int speed)
{
    _wheels[FR].moveForward(speed);
    _wheels[BR].moveForward(speed);
    _wheels[FL].stop();
    _wheels[BL].stop();
}

void Car::moveNE(unsigned int speed)
{
    _wheels[FL].moveForward(speed);
    _wheels[BL].moveForward(speed);
    _wheels[FR].stop();
    _wheels[BR].stop();
}

void Car::moveSE(unsigned int speed)
{
    _wheels[FL].moveBackward(speed);
    _wheels[BL].moveBackward(speed);
    _wheels[FR].stop();
    _wheels[BR].stop();
}

void Car::moveSW(unsigned int speed)
{
    _wheels[FR].moveBackward(speed);
    _wheels[BR].moveBackward(speed);
    _wheels[FL].stop();
    _wheels[BL].stop();
}

void Car::rotateClockwise(unsigned int speed)
{
    _wheels[FL].moveForward(speed);
    _wheels[BL].moveForward(speed);
    _wheels[FR].moveBackward(speed);
    _wheels[BR].moveBackward(speed);
}

void Car::rotateCounterClockwise(unsigned int speed)
{
    _wheels[FR].moveForward(speed);
    _wheels[BR].moveForward(speed);
    _wheels[FL].moveBackward(speed);
    _wheels[BL].moveBackward(speed);
}

void Car::setSignedSpeeds(int16_t signedSpeeds[NUM_WHEELS])
{
    for(int i = 0; i < NUM_WHEELS; i++)
    {
        int16_t speed = signedSpeeds[i];
        if (speed > 0) {
            _wheels[i].moveForward(constrain(speed, 0, 255));
        } else if (speed < 0) {
            _wheels[i].moveBackward(constrain(-1 * speed, 0, 255));
        } else {
            _wheels[i].stop();
        }
    }
    // Esta función sigue un orden diferente para el orden de las llantas
    /*
    _wheels[FL].setSignedSpeed(signedSpeeds[1]);
    _wheels[FR].setSignedSpeed(signedSpeeds[0]);
    _wheels[BR].setSignedSpeed(signedSpeeds[3]);
    _wheels[BL].setSignedSpeed(signedSpeeds[2]);
    */
    /*
    _wheels[FL].setSignedSpeed(signedSpeeds[0]);
    _wheels[BL].setSignedSpeed(signedSpeeds[1]);
    _wheels[FR].setSignedSpeed(signedSpeeds[2]);
    _wheels[BR].setSignedSpeed(signedSpeeds[3]);
    */
}

double Car::count(WheelId id)
{
    return _encoders[id].count();
}
