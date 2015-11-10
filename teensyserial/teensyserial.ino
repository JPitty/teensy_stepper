//Ard. 1.0.5, 1.6.5 11/9/15 JP
//This version doesn't use Accel, too many missed steps
//send an cmd over the serial cnxn, motor should do what it's told.
//Uses Teensy 3.2 to control an AMIS-30543 stepper motor controller using a Pololu dev board #2970.
//Teensy 3.2 hw SPI pins: 
// 11= Data Out
// 12= Data In
// 13= Clock

//#include <Metro.h>
#include <SPI.h>
#include <AMIS30543.h>

//Metro sysTimer = Metro(1);// milliseconds
const uint8_t stepPin = 9;  //pulse per step to AMIS
const uint8_t ss = 10;  //slave select

AMIS30543 stepper;
int debug = 1;

int stepSpeed = 350; //delay in microsec.
int stepSpeedt = stepSpeed;
int stepRampup = 0;
int totSteps = 0;
int stepCount = 0;
int runto=0;
int newcmd=0;
int runtime=0;
int steps=0;
boolean setdir, dir, setspd, setrt, setmode, setcont;
unsigned long time, dur;
int stepRampup = (400-stepSpeed)/100;

enum stepsetting {
1,2,4,8,16,32,64,128
};  //usage: stepsetting variable = <value>

void setup() {
  SPI.begin();
  stepper.init(ss);
  digitalWrite(stepPin, LOW);
  pinMode(stepPin, OUTPUT);
  delay(1);
  
  stepper.resetSettings();
  stepper.setCurrentMilliamps(1500);
  stepper.setStepMode(4); //1,2,4,8,16,32,64,128... ENUM?
  stepper.enableDriver();
  
  Serial.begin(9600);
  Serial.println("Hello Teensy 3.1 Serial");

  //sysTimer.reset();
}

void loop() {  
  //read the new Serial command
  if (Serial.available()) {
    runto = 0;
    setspd = false;
    setrt = false;
    setdir = false;
    setmode = false;
	//setpwr = false;  ADD BELOW to set motor current with stepper.setCurrentMilliamps()
    while (Serial.available()) {
      char c = Serial.read();
      if (c == 'd') { //set direction
        setdir = true;
      } else if (c == 's') { //start msg w/ 's' to set speed
        setspd = true;
      } else if (c == 'g') { //'g' to set runtime in millis
        setrt = true;
      } else if (c == 'm') { //'m' to set microsteps
        setmode = true;
      } else if (c == 'x') { //'x' to set cont run
        setcont = true;
      } else {
        if (c >= '0' && c <= '9') {
          runto = (10 * runto) + (c - '0') ; // convert string of digits to a number
        }
    }}
    
    if (setspd){
      stepSpeed = runto;
      newcmd=4;
    } else if (setrt) {
      runtime = runto;
      newcmd=2;
    } else if (setdir) {
      if (runto == 0) {
        dir = false;
      } else {
        dir = true;
      }
	  delayMicroseconds(1);
      stepper.setDirection(dir);
      delayMicroseconds(1);
      newcmd=3;
    } else if (setmode) {
        if (runto > 0 ) {  //TODO: check for valid setting, should this be an ENUM?
          stepper.setStepMode(runto);
        } else {
          stepper.setStepMode(4); //default
        }
      newcmd=5;
    } else if (setcont) {
      newcmd=6;
    } else {
      totSteps = runto;
      newcmd=1;
    }
  }
  
  //move number of steps
  if (newcmd==1) { 
    Serial.print("moving: ");
    Serial.println(runto);
    digitalWrite(11,HIGH);
    digitalWrite(10,LOW);
    stepCount = 0;
    stepSpeedt=400;
	
    //ramp-up
    while (stepCount < totSteps){
      if (stepSpeed < 350 && stepSpeedt > stepSpeed){ 
        stepSpeedt=400-stepCount*stepRampup;
      } else {
        stepSpeedt = stepSpeed;
      }
	  
      step(stepSpeedt);
	  stepCount++;
      if (Serial.available()) {
        Serial.flush();
        Serial.println("interrupted by serial line");
        break;
      }
    }
    delay(50);
    Serial.print("new pos: ");
    Serial.println(totSteps);
	
    //move for a duration
  } else if (newcmd==2) {  
    digitalWrite(10,LOW);
    time = millis();
    Serial.print("Time: ");
    Serial.println(time);
    dur = time + runtime;
    stepCount = 0;
    stepSpeedt=400;
    while (time < dur) {
      if (stepSpeed < 350 && stepSpeedt > stepSpeed){ //ramp-up
        stepSpeedt=400-stepCount*stepRampup;
        stepCount++;
      } else {
        stepSpeedt = stepSpeed;
      }
      digitalWrite(0,HIGH);
      delayMicroseconds(stepSpeedt);
      digitalWrite(0,LOW);
      delayMicroseconds(stepSpeedt); 
      time = millis();
      if (Serial.available()) {
        Serial.flush();
        Serial.println("interrupted by serial line");
        break;
      }
    }
    Serial.print("Time: ");
    time = millis();
    Serial.println(time);
    delay(250);
    digitalWrite(10,HIGH);
    
  } else if (newcmd==3) { //set direction
      digitalWrite(1,dir);
      Serial.print("direction: ");
      Serial.println(dir);
  } else if (newcmd==4) {
      Serial.print("new speed: ");
      Serial.println(stepSpeed);
  } else if (newcmd==5) {
      Serial.print("micro step div: ");
      Serial.println(steps);
  } else if (newcmd==6) {
      Serial.println("running continous");
      digitalWrite(11,HIGH);
      digitalWrite(10,LOW);
      while (1) {
        if (stepSpeed < 350 && stepSpeedt > stepSpeed){ //ramp-up
          stepSpeedt=400-stepCount*stepRampup;
          stepCount++;
        } else {
          stepSpeedt = stepSpeed;
        }
        digitalWrite(0,HIGH);
        delay(50);
        digitalWrite(0,LOW);
        delay(50);
        if (Serial.available()) {
          Serial.flush();
          Serial.println("interrupted by serial line");
          setcont = false;
          digitalWrite(10,HIGH);
          digitalWrite(11,LOW);
          break;
        }
      }
  }
  newcmd=0;
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
  //REPLACE so this isn't blocking:
  delayMicroseconds(speedDelay);
}