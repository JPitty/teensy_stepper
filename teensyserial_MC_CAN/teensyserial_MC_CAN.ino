//Arduino 1.6.5, 5/6/16 JP,
//control of AMC DPRALTE-015B200 digital motor controller via CANbus
//uses FastCRC for xmodem crc calc on serial msgs

//#include <util/crc16.h>
#include <FastCRC.h>
#include <FlexCAN.h>

FlexCAN CANbus(500000);
static CAN_message_t rxmsg; //msg  rec'd from linux: 0-256
static CAN_filter_t myMask, myFilter_0;

#define Ser Serial1 //pins: RX1=0, TX1=1
#define BUFSIZE 16384
FastCRC16 CRC16;
uint32_t crc;

//AMC message structure:
// Header section:
uint8_t SOF = 0xA5; //start of frame
uint8_t Addr = 0x3F; //drive address, 3Fh=factory default
uint8_t Ctrl; //2bit=res/4bit=seq#/2bit=cmd(LSB)
uint8_t Idx; //8bit
uint8_t Offset; //8bit
uint8_t DataW; //8bits
uint16_t CrcH; //16bits, MSB,
//Data Section
uint8_t DataF; //255word max, LSB  FIX type!
uint16_t CrcD; //16bits

//Temp:
uint8_t buf[BUFSIZE];
uint8_t bufAt[8] = {SOF, Addr, 0x01, 0x02, 0x04, 0x01, 0x0F, 0x0F};
  //recd #bytes: A5 FF 2 1 0 1 32 FF 43 0 58 9F
//uint8_t bufAt[8] = {SOF, Addr, 0x01, 0x45, 0x02, 0x02, 0x0D, 0xF7};

//example: drive status-At Cmd:
//send: A5|3F|01|02|04|01|0F|0F
//reply: A5|FF|02|01|00|01|32|FF|C7|00|8F|C3 (ex. only, 00C7=Data)
//end Temp

//on-board RGB LED indicators:
const int redPin =  23;
const int greenPin =  21;
const int bluePin =  22;
int ledPin = 13;

int i, debug = 1;

void setup() {
  pinMode(ledPin, OUTPUT);
  Ser.begin(115200, SERIAL_8N1_RXINV_TXINV); //to MC, inverted pins for direct TTL cnxn
  Serial.begin(115200); //to USB

  //CAN addressing
  myMask.rtr = 0;
  myMask.ext = 0;
  myMask.id = 0xfff;  //masks everything except exact filter.id
  CANbus.begin(myMask);
  myFilter_0.id = 0x011;
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

  /*//Fill array with data
  for (int i=0; i<BUFSIZE; i++) {
    buf[i] = (i+1) & 0xff;
  }*/

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  ledWrite(30, 10, 20);
}

void loop() {
  int count = 0;

  //send to USB
  /*if (count > 0){
    Serial.print("recd #bytes: ");
    for (int i=0; i<count; i++) {
      Serial.print(String(buf[i], HEX));
      Serial.print(" ");
    }
    Serial.println();
  }*/

  if ( CANbus.available() ){  //get the msg
    CANbus.read(rxmsg);

    if ( rxmsg.buf[0] == 0 ){  //all stop
      //send stop msg;
      ledWrite(0, 0, 0);
    }
    else if ( rxmsg.buf[0] == 1 ){  //1=led cmd, must have 3 values after
      if (rxmsg.len >= 4) {
        ledWrite(rxmsg.buf[1], rxmsg.buf[2], rxmsg.buf[3]);
      }
    }
    else if ( rxmsg.buf[0] == 2 ){ //MC set commands
      //pop rxmsg.buf[0]
      //crc=crcIt(buf)
      //form msg: msg = msg + crc;
      //sendToMC(msg);
    }
    else if ( rxmsg.buf[0] == 3 ){ //MC get commands
      //do something;
    }
    else if ( rxmsg.buf[0] == 9 ){ //MC run commands
      //do something;
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
  } // end CAN bus.available

  //on-board led heartbeat (Teensy): TODO: get rid of delays
  digitalWrite(ledPin, HIGH);
  //delay(500);
  digitalWrite(ledPin, LOW);
  //delay(500);

} //end loop

// Writes PWM pins to set led
void ledWrite(int r, int g, int b){
  analogWrite(redPin, r);
  analogWrite(bluePin, g);
  analogWrite(greenPin, b);
}

buf crcIt(buf){
  crc = CRC16.xmodem(buf, sizeof(buf));
  return crc;
}

buf sendToMC(sendmsg){
  //send msg to MC
  Ser.write(sendmsg,sizeof(sendmsg));
  Ser.flush();
  if ( debug > 0 ) { Serial.println("sent bufAt"); }

  //get reply
  delay(100);
  while (Ser.available()) {  // receive all bytes into "buf"
      buf[count++] = Ser.read();
  }
  return buf
}
