//Omkar Suralkar 1 + Comp RRL, [21 - 05 - 2023 11:19]
// RR
#include <XBOXONE.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
USB Usb;
XBOXONE Xbox(&Usb);
RF24 radio(2, 4);
const byte address[6] = "10101";
float timer1, timer2;
struct record
{
  int rx;
  int ry;
  int lx;
  int ly;
  int up;
  int down;
  int right;
  int left;
  int green;
  int blue;
  int yellow;
  int red;
  int r1;
  int l1;
  int l2;
  int r2;
  int r3;
  int l3;
  int start;
  int back;
  int xbox;
  int sync;


};
record Record;
int rx , ry , lx , ly , up, down, right, left, green, blue, yellow, red, l1, r1, l2, r2, l3, r3, start, back, xbox, sync;
int flag = 0 ;
void setup() {
  Serial.begin(115200);
  while (!Serial);
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nXBOX ONE USB Library Started"));
  radio.begin();
  radio.openWritingPipe(address);
  radio.setAutoAck(false);
  radio.setDataRate( RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}
void loop() {
  //  timer1=millis();{
  Usb.Task();
  if (Xbox.XboxOneConnected) {
    radio.write(&Record, sizeof(Record));
    rx = Xbox.getAnalogHat(RightHatX) ;
    ry = Xbox.getAnalogHat(RightHatY) ;
    rx = map( rx , -32768 , 31690 , -127 , 127 );
    ry = map( ry , -30712 , 32767 , -127 , 127 );
    rx = constrain( rx , -127 , 127 );
    ry = constrain( ry , -127, 127 );
    if (( rx > -20 && rx < 20 ) && ( ry > -20 && ry < 20 )) {
      rx = 0 ;
      ry = 0 ;
    }
    lx = Xbox.getAnalogHat(LeftHatX) ;
    ly = Xbox.getAnalogHat(LeftHatY) ;
    lx = map( lx , -32768 , 31690 , -127 , 127 );
    ly = map( ly , -30712 , 32767 , -127 , 127 );
    lx = constrain( lx , -127 , 127 );
    ly = constrain( ly , -127, 127 );
    if (( lx > -20 && lx < 20 ) && ( ly > -20 && ly < 20 )) {
      lx = 0 ;
      ly = 0 ;
    }

    //    ****************************  //
    //    if ( Xbox.getButtonClick(UP)) {
    //      up++;
    //    }
    //
    //    if ( Xbox.getButtonClick(DOWN)) {
    //      down++;
    //    }
    //
    //    if ( Xbox.getButtonClick(LEFT)) {
    //      left++;
    //    }
    //
    //    if ( Xbox.getButtonClick(RIGHT)) {
    //      right++;
    //    }
    up = Xbox.getButtonPress(UP);
    down = Xbox.getButtonPress(DOWN);
    right = Xbox.getButtonPress(RIGHT);
    left = Xbox.getButtonPress(LEFT);
    l1 = Xbox.getButtonPress(L1);
    r1 = Xbox.getButtonPress(R1);
    green = Xbox.getButtonPress(A);
    red = Xbox.getButtonPress(B);
    blue = Xbox.getButtonPress(X);
    yellow = Xbox.getButtonPress(Y);
    l2 = Xbox.getButtonPress(L2);
    r2 = Xbox.getButtonPress(R2);
    if (l2 > 100 ) {
      l2 = 1;
    }
    else {
      l2 = 0;
    }
    if (r2 > 100)r2 = 1;
    else {
      r2 = 0;
    }

     if (l2 > 0 || r2 > 0)
        Xbox.setRumbleOn(l2, r2, l2, r2);
      else
        Xbox.setRumbleOff();
    //    if ( Xbox.getButtonClick(A) ) {
    //      green++;
    //    }
    //
    //
    //    if ( Xbox.getButtonClick(B) ) {
    //      red++;
    //    }
    //
    //
    //    if ( Xbox.getButtonClick(X) ) {
    //      blue++;
    //    }
    //
    //
    //    if ( Xbox.getButtonClick(Y) ) {
    //      yellow++;
    //    }

    //    if ( Xbox.getButtonClick(L1) ) {
    //      l1++;
    //    }
    //
    //    if (  Xbox.getButtonClick(R1) ) {
    //      r1++;
    //    }

    if ( Xbox.getButtonClick(L3) ) {
      l3++;
    }

    if (  Xbox.getButtonClick(R3) ) {
      r3++;
    }

    Record.rx = rx;
    Record.ry = ry;
    Record.lx = lx;
    Record.ly = ly;
    Record.up = up;
    Record.down = down;
    Record.left = left;
    Record.right = right;
    Record.green = green;
    Record.blue = blue;
    Record.red = red;
    Record.yellow = yellow;
    Record.l1 = l1;
    Record.r1 = r1;
    Record.l2 = l2;
    Record.r2 = r2;

    Record.l3 = l3;
    Record.r3 = r3;
    Serial.print( radio.write(&Record, sizeof(Record)));

    //    Serial.print("  RightHatY:");
    //    Serial.print(Xbox.getAnalogHat(RightHatX));
    //    Serial.print("    LeftHatX : ");
    //    Serial.print(Xbox.getAnalogHat(RightHatY));
    //    Serial.print("    rightHatX:");
    Serial.print(rx);
    //    Serial.print("    RightHaty:");
    //    Serial.print(ly);
    //    Serial.print(ly);
    //    Serial.print("    Up:");
    //    Serial.print(Record.up);
    //    Serial.print("    Down:");
    //    Serial.print(Record.down);
    //    Serial.print("    Left:");
    //    Serial.print(Record.left);
    //    Serial.print("    Right:");
    //    Serial.print(Record.right);
    //    Serial.print("    Green:");
    //    Serial.print(Record.green);
    //    Serial.print("    Blue:");
    //    Serial.print(Record.blue);
    //    Serial.print("    Red:");
    //    Serial.print(Record.red);
    //    Serial.print("    Yellow:");
    //    Serial.print(Record.yellow);
    //    Serial.print("    l1:");
    //    Serial.print(Record.l1);
    //    Serial.print("    r1:");
    //    Serial.print(Record.r1);
    //    Serial.print("    l2:");
    //    Serial.print(Record.l2);
    //    Serial.print("    r2:");
    //    Serial.print(Record.r2);
    //    Serial.print("    l3:");
    //    Serial.print(Record.l3);
//        Serial.print("    r3:");
//        Serial.print(Record.r3);
    //    Serial.print(" Flag = ");
    //    Serial.print(flag);
    Serial.println();
    //  delay(1000);
    //timer2=millis();
    //Serial.println(    timer2-timer1);
  }
}
