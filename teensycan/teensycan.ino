//Arduino 1.6.5, 11/9/15 JP, working well on 1/28/16
//Uses Teensy 3.2 CAN bus, intended to be mounted on CAN_Node v0.2 or eq.
//to control an AMIS-30543 stepper motor controller using a Pololu dev board #2970.
//Teensy 3.2 hw SPI pins: 
// 11= Data Out
// 12= Data In
// 13= Clock

//#include <Metro.h>
#include <FlexCAN.h>
#include <SPI.h>
#include <AMIS30543.h>

//Metro sysTimer = Metro(1);// milliseconds
const uint8_t stepPin = 9;  //pulse per step pin to AMIS
const uint8_t ss = 10;  //slave select

AMIS30543 stepper;
FlexCAN CANbus(500000);
static CAN_message_t rxmsg; //msg  rec'd from linux: 0-256
static CAN_filter_t myMask, myFilter_0;

//int txCount,rxCount;
//unsigned int txTimer,rxTimer;

//on-board RGB LED indicator:
const int redPin =  23;
const int greenPin =  21;
const int bluePin =  22;

//stepper defaults
int stepSpeed = 350; //delay in microsec.
int stepSpeedt = stepSpeed;
int stepRampup = 4; //dwells on this many steps before accel to next step speed

int i, debug = 1;

void setup(void)
{
  //CAN addressing
  myMask.rtr = 0;
  myMask.ext = 0;
  myMask.id = 0xfff;  //masks everything except exact filter.id
  CANbus.begin(myMask);
  myFilter_0.id = 0x007; //this should be read from hw?
  myFilter_0.ext = 0;
  myFilter_0.rtr = 0;
  CANbus.setFilter(myFilter_0,0);
  CANbus.setFilter(myFilter_0,1);
  CANbus.setFilter(myFilter_0,2);
  CANbus.setFilter(myFilter_0,3);
  CANbus.setFilter(myFilter_0,4);
  CANbus.setFilter(myFilter_0,5);
  CANbus.setFilter(myFilter_0,6);
  CANbus.setFilter(myFilter_0,7);

  SPI.begin();
  stepper.init(ss);
  digitalWrite(stepPin, LOW);
  pinMode(stepPin, OUTPUT);
  delay(1);
  
  stepper.resetSettings();
  stepper.setCurrentMilliamps(2000);
  stepper.setDirection(1);
  stepper.setStepMode(4); //1,2,4,8,16,32,64,128
  stepper.enableDriver();
  stepper.disableDriver(); //only enable when needed
    
  Serial.begin(9600);
  delay(100);
  Serial.println(F("Hello Teensy 3.2 CAN."));

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  ledWrite(30, 10, 20);
  
  delay(500);

  //sysTimer.reset();
} //end setup

void loop(void)
{ 
//  msg.len = 8;
//  msg.id = 0x222;
//  for( int idx=0; idx<8; ++idx ) {
//    msg.buf[idx] = '0'+idx+1;
//  }
  
  if ( CANbus.available() ){  //get the msg
    CANbus.read(rxmsg);

    if ( rxmsg.buf[0] == 0 ){  //all stop
      stepper.disableDriver();
      ledWrite(0, 0, 0);
    }
    else if ( rxmsg.buf[0] == 1 ){  //1=led cmd, must have 3 values after
      if (rxmsg.len >= 4) {
        ledWrite(rxmsg.buf[1], rxmsg.buf[2], rxmsg.buf[3]);
      }
    }
    else if ( rxmsg.buf[0] == 2 ){
      motorControl(rxmsg);
    }
    else if ( rxmsg.buf[0] == 3 ){
      motorSet(rxmsg);
    }
    
    if ( debug > 0 ) {
      Serial.print("sender id: ");
      Serial.println(rxmsg.id);
      //for loop through data buf
      for (i=0; i<rxmsg.len; i++) {
        Serial.print(i);
        Serial.print(": ");
        Serial.println(rxmsg.buf[i]);
      }
      Serial.println();
    }
  }

/*test: uncomment to run the motor back and forth continuously
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
  delay(300);
end test*/
} //end loop

// Sends a pulse on the NXT/STEP pin to tell the driver to take
// one step, and also delays to control the speed of the motor.
int step(int stepSpeedt, int stepCount) {
  // The NXT/STEP minimum high pulse width is 2 microseconds.
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(3);
  digitalWrite(stepPin, LOW);

  //control the ramp rate
  //if (debug) {Serial.println(stepSpeedt);} //to debug accel
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

// Writes a high or low value to specify what direction to turn the motor.
void setDirection(bool dir)
{
  // The NXT/STEP pin must not change for at least 0.5
  // microseconds before and after changing the DIR pin.
  //?? Is this true for sw dir change with SPI?
  delayMicroseconds(1);
  stepper.setDirection(dir);
  delayMicroseconds(1);
}

// Writes PWM pins to set led
void ledWrite(int r, int g, int b){
  analogWrite(redPin, r);
  analogWrite(bluePin, g);
  analogWrite(greenPin, b);
}

// Run the stepper
// 1 = continuous
// 2 = #steps
// 3 = 
void motorControl(CAN_message_t rxmsg){
  int stepCount = 0;
  if ( rxmsg.buf[1] == 1 ){
    stepper.enableDriver();
    while ( !CANbus.available()){
      step(stepSpeedt, stepCount);
      stepCount++;
    }
    stepper.disableDriver();
  }
}

// Writes motor settings where buf[1]=
// 1 = speed; sets stepSpeed which is delay between steps (smaller = faster)
// 2 = direction
// 3 = power, mA
// 4 = microstepping 1,2,4,8,16,32,64,128
void motorSet(CAN_message_t rxmsg){
  String msg1 = "";
  int msg2 = 0;
  boolean dir;
  
  if ( rxmsg.buf[1] == 1 ){
    stepSpeed = rxmsg.buf[2] * rxmsg.buf[3];
    msg1 = "new speed: ";
    msg2 = stepSpeed;
  }
    
  if ( rxmsg.buf[1] == 2 ){
    if ( rxmsg.buf[2] == 0 ){
      dir = false;
      setDirection(dir);
      msg1 = "new direction: ";
      msg2 = rxmsg.buf[2];
    }
    else if ( rxmsg.buf[2] == 1 ){
      dir = true;
      setDirection(dir);
      msg1 = "new direction: ";
      msg2 = rxmsg.buf[2]; 
    }
    else {
      msg1 = "failed direction: ";
      msg2 = 666;  
    }
  }

  if ( rxmsg.buf[1] == 3 ){
    int power = rxmsg.buf[2] * rxmsg.buf[3];
    msg1 = "new power: ";
    msg2 = power;
  }

  if ( rxmsg.buf[1] == 4 ){
    //TODO: check modulo of buf[2] first
    stepper.setStepMode(rxmsg.buf[2]);
    msg1 = "new usteps: ";
    msg2 = rxmsg.buf[2];
  }
    
  if ( debug > 0 ) {
    Serial.print(msg1);
    Serial.println(msg2);
  }
}

/*static void hexDump(uint8_t dumpLen, uint8_t *bytePtr)
{
  uint8_t working;
  while( dumpLen-- ) {
    working = *bytePtr++;
    Serial.write( hex[ working>>4 ] );
    Serial.write( hex[ working&15 ] );
  }
  Serial.write('\r');
  Serial.write('\n');
}*/
