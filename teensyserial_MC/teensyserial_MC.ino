//Arduino 1.6.5, 4/30/16 JP,
//early control of AMC DPRALTE-015B200 RS232 digital motor controller
//uses FastCRC for xmodem crc calc

//#include <util/crc16.h>
#include <FastCRC.h>

#define Ser Serial1 //pins: RX1=0 w/ 10kOhm, TX1=1 w/ 220Ohm
#define BUFSIZE 16384
FastCRC16 CRC16;
int ledPin = 13;
//uint32_t crc;

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

void setup() {
  pinMode(ledPin, OUTPUT);
  Ser.begin(115200, SERIAL_8N1_RXINV_TXINV); //to MC
  Serial.begin(115200); //to USB
  
  /*//Fill array with data
  for (int i=0; i<BUFSIZE; i++) {
    buf[i] = (i+1) & 0xff;
  }*/
  
  //crc = CRC16.xmodem(buf, sizeof(buf));
  //Ser.printf("\r\nXMODEM: %s",(crc==0x31c3) ? "OK":"FALSE!!!!"); for ascii '123456789'
  
  /*Ser.println("\r\nXMODEM 16-Bit CRC:");
  Ser.flush();
  crc = CRC16.xmodem(buf, BUFSIZE);
  Ser.printf("FastCRC: Value 0x%X\r\n", crc);
  Ser.flush(); */
}

void loop() {
  int count = 0;
  //send msg to MC
  Ser.write(bufAt,sizeof(bufAt));
  Ser.flush();
  Serial.println("sent bufAt");
  //get reply
  delay(100);
  while (Ser.available()) {  // receive all bytes into "buf"
      buf[count++] = Ser.read();
  }
  //send to USB
  if (count > 0){
    Serial.print("recd #bytes: ");
    for (int i=0; i<count; i++) {
      Serial.print(String(buf[i], HEX));
      Serial.print(" ");
    }
    Serial.println();
  }
  digitalWrite(ledPin, HIGH);
  delay(500);
  digitalWrite(ledPin, LOW);
  delay(500);
}
