//Arduino 1.6.5, 11/9/15 JP
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
const uint8_t stepPin = 9;  //pulse per step to AMIS
const uint8_t ss = 10;  //slave select

AMIS30543 stepper;
FlexCAN CANbus(500000);
static CAN_message_t msg,rxmsg;
//static uint8_t hex[17] = "0123456789abcdef";
static CAN_filter_t myMask, myFilter_0;

//int txCount,rxCount;
//unsigned int txTimer,rxTimer;

//on-board RGB LED indicator:
const int redPin =  23;
const int greenPin =  21;
const int bluePin =  22;
int i;
int debug = 1;

void setup(void)
{
  //CAN addressing
  myMask.rtr = 0;
  myMask.ext = 0;
  myMask.id = 0xfff;  //masks everything except exact filter.id
  CANbus.begin(myMask);
  myFilter_0.id = 0x003; //this should be read from hw: dips, etc
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
  stepper.setCurrentMilliamps(1500);
  stepper.setStepMode(4); //1,2,4,8,16,32,64,128
  stepper.enableDriver();
  
  Serial.begin(9600);
  delay(100);
  Serial.println(F("Hello Teensy 3.1 CAN."));

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  analogWrite(redPin, 30);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 20);
  delay(500);

  //sysTimer.reset();
}

void loop(void)
{ 
//  msg.len = 8;
//  msg.id = 0x222;
//  for( int idx=0; idx<8; ++idx ) {
//    msg.buf[idx] = '0'+idx+1;
//  }

  if ( CANbus.available() ){
    CANbus.read(rxmsg);
    
    if (rxmsg.len >= 3) {
      analogWrite(redPin, rxmsg.buf[0]);
      analogWrite(bluePin, rxmsg.buf[1]);
      analogWrite(greenPin, rxmsg.buf[2]);
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
    }
  }

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
