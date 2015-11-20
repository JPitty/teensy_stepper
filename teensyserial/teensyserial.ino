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
int debug = 0;

//these should be global?
int stepSpeed = 350; //delay in microsec.
int stepSpeedt = stepSpeed;
int stepRampup = 4; //dwells on this many steps before accel to next step speed
int totSteps = 0;
int stepCount = 0;
int runto=0;
int newcmd=0;
int runtime=0;
int steps=8;
boolean setdir, dir, setspd, setrt, setmode, setcont, setpwr;
unsigned long time, dur;

//Should this be used?
//enum stepsetting {
//1,2,4,8,16,32,64,128  //}  //usage: stepsetting variable = <value>

void setup() {
  SPI.begin();
  stepper.init(ss);
  digitalWrite(stepPin, LOW);
  pinMode(stepPin, OUTPUT);
  delay(1);
  
  stepper.resetSettings();
  stepper.setCurrentMilliamps(2000);
  stepper.setStepMode(steps); //ENUM?
  stepper.setDirection(1);
  stepper.enableDriver();  //should enable and disable as needed
  
  Serial.begin(9600);
  Serial.println("Hello Teensy 3.1 Serial");
  //sysTimer.reset();
}

void loop() {  
  //read the new Serial command
  if (Serial.available()) {
    //reset values for a fresh cmd
    runto = 0;
    setspd = false;
    setrt = false;
    setdir = false;
    setmode = false;
	  setpwr = false;
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
      } else if (c == 'p') { //'p' to set mA
        setpwr = true;  
      } else {
        if (c >= '0' && c <= '9') {
          runto = (10 * runto) + (c - '0') ; // convert string of digits to a number
        }}}
    
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
        steps=runto;
        if (steps > 0 ) {  //TODO: check for valid setting, should this be an ENUM?
          stepper.setStepMode(steps);
        } else {
          stepper.setStepMode(8); //default
        }
      newcmd=5;
    } else if (setcont) {
      newcmd=6;
    } else if (setpwr) {
      if (runto > 200 && runto < 2500) {
        delayMicroseconds(1);
        stepper.setCurrentMilliamps(runto);
        delayMicroseconds(1);  
      }
      newcmd = 7;
    } else {
      totSteps = runto;
      newcmd=1;
    }
  } //end of Serial.Available

  if (newcmd>0) { //skip if no new cmd
    //move number of steps
    if (newcmd==1) { 
      time = millis();
      Serial.print("moving #steps: ");
      Serial.println(runto);
      stepCount = 0;
      stepSpeedt=400;
  	  stepper.enableDriver();
     
      //ramp-up FIX: this should be moved to step fxn, values aren't right
      while (stepCount < totSteps){
  	    stepSpeedt = step(stepSpeedt, stepCount);
  	    stepCount++;
        //check serial to interupt
        if (Serial.available()) {
          Serial.flush();
          Serial.println("interrupted by serial line");
          break;
        }
      }
      int dT = millis()-time;
      float rpm = (60.0*(float)totSteps/(float)(400*steps))/((float)(dT/1000.0));
      Serial.print("dTime: ");
      Serial.println(dT);
      Serial.print("rpm: ");
      Serial.println(rpm);
      
      delay(50);
      stepper.disableDriver();
      Serial.print("new pos: ");
      Serial.println(totSteps);
  	
      //move for a duration
    } else if (newcmd==2) {  
      time = millis();
      Serial.print("Time: ");
      Serial.println(time);
      dur = time + runtime;
      stepCount = 0;
      stepSpeedt=400;
      stepper.enableDriver();
      while (time < dur) {
        stepSpeedt = step(stepSpeedt,stepCount);
        stepCount++;
        time = millis();
        //check serial to interupt
        if (Serial.available()) {
          Serial.flush();
          Serial.println("interrupted by serial line");
          break;
        }
      }
      stepper.disableDriver();
      Serial.print("Time: ");
      time = millis();
      Serial.println(time);
      delay(50);
      
    } else if (newcmd==3) {
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
        stepper.enableDriver();
        while (1) {
          stepSpeedt = step(stepSpeedt, stepCount);
          stepCount++;
          if (Serial.available()) {
            Serial.flush();
            Serial.println("interrupted by serial line");
            setcont = false;
            break;
          }
        }
        stepper.disableDriver();
    }}
  newcmd=0; //resets the serial cmd
} //end of loop

// Sends a pulse on the NXT/STEP pin to tell the driver to take
// one step, and also delays to control the speed of the motor.
int step(int stepSpeedt, int stepCount) {
  // The NXT/STEP minimum high pulse width is 2 microseconds.
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(3);
  digitalWrite(stepPin, LOW);

  //control the ramp rate
  if (debug) {Serial.println(stepSpeedt);} //to debug accel
  if (stepSpeed < 350 && stepSpeedt > stepSpeed){ //FIX: these constants
    stepSpeedt=400-stepCount/stepRampup;
  } else {
    stepSpeedt = stepSpeed;
  }

  // This delay controls the stepper motor's speed.  increase = stepper motor goes slower,
  //decrease = faster, but there is a limit to how fast it can go before it starts missing steps.
  delayMicroseconds(stepSpeedt);
  return stepSpeedt;
}
