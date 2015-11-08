#include <SPI.h>
#include <AMIS30543.h>

const uint8_t stepPin = 9;
const uint8_t ss = 10;

AMIS30543 stepper;

void setup() {
  SPI.begin();
  stepper.init(ss);

  digitalWrite(stepPin, LOW);
  pinMode(stepPin, OUTPUT);

  delay(1);
  stepper.resetSettings();
  stepper.setCurrentMilliamps(1500);
  stepper.setStepMode(4); //1,2,4,8,16,32,64,128
  stepper.enableDriver();
}

void loop() {
  int stepSpeed = 250;
  // Step in the default direction
  setDirection(0);
  for (unsigned int x = 0; x < 2400; x++)
  {
    step(stepSpeed);
  }
  // Wait for 300 ms.
  delay(300);
  // Step in the other direction
  setDirection(1);
  for (unsigned int x = 0; x < 2400; x++)
  {
    step(stepSpeed);
  }
  // Wait for 300 ms.
  delay(300);
}

// Sends a pulse on the NXT/STEP pin to tell the driver to take
// one step, and also delays to control the speed of the motor.
void step(int speedDelay)
{
  // The NXT/STEP minimum high pulse width is 2 microseconds.
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(3);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(3);

  // The delay here controls the stepper motor's speed.  You can
  // increase the delay to make the stepper motor go slower.  If
  // you decrease the delay, the stepper motor will go fast, but
  // there is a limit to how fast it can go before it starts
  // missing steps.
  delayMicroseconds(speedDelay);
}

// Writes a high or low value to the direction pin to specify
// what direction to turn the motor.
void setDirection(bool dir)
{
  // The NXT/STEP pin must not change for at least 0.5
  // microseconds before and after changing the DIR pin.
  delayMicroseconds(1);
  stepper.setDirection(dir);
  delayMicroseconds(1);
}
