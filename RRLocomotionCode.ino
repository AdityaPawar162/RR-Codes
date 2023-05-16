
//*************************LIBRARIES*****************//
#include <Wire.h>
#include <MPU6050.h>
MPU6050 mpu;
#include<usbhid.h>
#include <hiduniversal.h>
#include <usbhub.h>
#include <SPI.h>
#include "hidjoystickrptparser.h"
#include "CytronMotorDriver.h"
#include <ODriveArduino.h>

template<class T> inline Print& operator <<(Print &obj, T arg) {
  obj.print(arg);
  return obj;
}
template<>        inline Print& operator <<(Print &obj, float arg) {
  obj.print(arg, 4);
  return obj;
}

HardwareSerial& odrive_serial = Serial3;
USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);
JoystickEvents JoyEvents;
JoystickReportParser Joy(&JoyEvents);
ODriveArduino odrive(odrive_serial);

// Configure the motor driver.
CytronMD motor1(PWM_DIR, 5, 32);
CytronMD motor2(PWM_DIR, 6, 34);
CytronMD motor3(PWM_DIR, 3, 36);

//************************PID GAINS******************//
float Kp = 15;// 6;//13;;8
float Ki =  0.000;//0.002;//0.003;//0.003
float Kd =  30;//37;//30

//*********************DECLARATIONS************//
int M1 , M2 , M3, m1, m2, m3;
unsigned long timer = 0;
unsigned long timer1 = 0;
unsigned long timer2 = 0;
float timeStep = 0.0122;
volatile float yaw = 0;
int pre_error = 0;
int setpoint = 0 ;
int error ;
int Val;
float pi = 3.14;
float ang = 0;
int var_100 = 100;
float rate, incre, PRO, DER, INT, output, pid ;

//______________________________________

int flag911 = 0;
int flag811 = 0;
int flag711 = 0;//For Ramp Mode
float flag611 = 0;//For R1
float flag511 = 0;//For L1
int flot_auto = 1; //BLDC
int flag11 = 0;//Gpad
int flag12 = 0;
int flag13 = 0;
int flag14 = 0;
int flag1 = 0; //locomotion angle 5
int flag2 = 0;
int flag3 = 0; //locomotion angle 90
int flag4 = 0;
float var0 ;
float var1 ;
int cnt = 0;//For BLDC
int cnt1 = 1;//For Vertical Angle DCV
int cnt2 = 0;//locomotion angle
int cnt3 = 0;//Temp variable
int cnt4 = 0;


//_______________________________________


void setup()
{
  //  delay(2000);
  Serial.begin(115200);
  // Initialize MPU6050
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(1);

#if !defined(MIPSEL)
  while (!Serial);
#endif
  Serial.println("Start");

  while (Usb.Init() == -1)
    Serial.println("OSC did not start.");

  delay(200);
  pinMode(28, OUTPUT);
  pinMode(29, OUTPUT);
  if (!Hid.SetReportParser(0, &Joy))
    ErrorMessage<uint8_t > (PSTR("SetReportParser"), 1);

  //______________odrive_________________

  // ODrive uses 115200 baud
  odrive_serial.begin(115200);
  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open
  Serial.println("ODriveArduino");
  Serial.println("Setting parameters...");
  odrive.SetVelocity(0, 0);
  odrive.SetVelocity(1, 0);
  //________________________________________
}

void loop()
{
  timer = millis();
  Vector norm = mpu.readNormalizeGyro();
  yaw = yaw + norm.ZAxis * timeStep;

//  if (yaw <= 0.9 && yaw >= -0.9){
//    yaw = 0;
//  }
  Usb.Task();

    float Buttons = JoyEvents.bu;
  float rxa = JoyEvents.rx;
  float rya = JoyEvents.ry;
  float lxa =  JoyEvents.lx;
  float lya = JoyEvents.ly;
  float back = Joy.bk;
  float start = Joy.st;
  float blue = Joy.blue;
  float green = Joy.green;
  float red = Joy.red;
  float yellow = Joy.yellow;
  float gpad = JoyEvents.ht;
  float L1 = Joy.lb;
  float R1 = Joy.rb;
  float L2 = Joy.lt;
  float R2 = Joy.rt;


  //*****************LOCOMOTION******************





  Serial.print(" Yaw = ");
  Serial.print(yaw);

  rxa = rxa - 128;
  rya = -rya + 127;
rya = -rya ;
  int g = rxa - 30;
  int h = rya - 30;
  if (rya < 30 && rya > -31) {
    h = 0;
  }
  if (rya < -30) {
    h = h + 60;
  }

  if (rxa < 30 && rxa > -31) {
    g = 0;
  }
  if (rxa < -30) {
    g = g + 60;
  }

  int c = square(g);
  int d = square(h);
  float r = sqrt(c + d);

  float angle = atan2 (h, g) ;
  float theta = angle * (180 / pi);


  //************************ BLDC ***********************

  if ( start == 1 && flot_auto == 0)
  { var0 = 20;
    var1 = 20;
    cnt++;
    flot_auto = 1 ;
  }

  if ( start == 0 && flot_auto == 1 )
  {
    flot_auto = 0 ;
  }
  //  Serial.print("cnt ");
  //  Serial.print(cnt);


  if (cnt % 2 == 1 )
  {
    //    ----------------------------------Increment of 0axis---
    if (gpad == 0 && flag11 == 0)
    {
      var0 += 0.1;
      odrive.SetVelocity(0, var0);
      odrive.SetVelocity(1, -var0);
      flag11 = 1;
      Serial.print(" Increment of 0axis");
    }
    if (gpad == 9 && flag11 == 1)
    {
      flag11 = 0;
    }
  }
  //    ----------------------------------Decrement of 0axis---
  if (gpad == 4 && flag12 == 0)
  {
    var0 -= 0.1;
    odrive.SetVelocity(0, var0);
    odrive.SetVelocity(1, -var0);
    flag12 = 1;
    Serial.print(" Decrement of 0axis");
  }
  if (gpad == 9 && flag12 == 1)
  {
    flag12 = 0;
  }

  //    ----------------------------------Increment of 1axis---
  if (gpad == 2 && flag13 == 0)
  {

    var1 += 1;
    odrive.SetVelocity(0, var1);
    odrive.SetVelocity(1, -var1);
    flag13 = 1;
    Serial.print(" Increment of 1axis");
  }
  if (gpad == 9 && flag13 == 1)
  {
    flag13 = 0;
  }
  //    ----------------------------------Decrement of 1axis---
  if (gpad == 6 && flag14 == 0)
  {

    var1 -= 1;
    odrive.SetVelocity(0, var1);
    odrive.SetVelocity(1, -var1);
    flag14 = 1;
    Serial.print(" Decrement of 1axis");
  }
  if (gpad == 9 && flag14 == 1)
  {
    flag14 = 0;
  }

 
  Serial.print("var0===");
  Serial.print(var0);
  Serial.print("var1===");
  Serial.print(var1);


  //    ----------------------------------Stop BLDC---

  if (cnt % 2 == 0)
  {
    var0 = 0 ;
    var1 = 0 ;
    odrive.SetVelocity(0, var0);
    odrive.SetVelocity(1, -var1);
    Serial.print("stop");
  }


  // *************************************** VERTICAL ANGLE ***

  if (L1 == 1 && flag511 == 0)//Use flag511 for L1
  {
    cnt1++;
    flag511 = 1;
  }
  if (L1 == 0 && flag511 == 1)
  {
    flag511 = 0;
  }
  Serial.print("cnt1 ");
  Serial.print(cnt1);

  if (cnt1 % 2 == 0)
  {
    digitalWrite(29, HIGH);
  }
  if (cnt1 % 2 == 1)
  {
    digitalWrite(29, LOW);
  }

  // *************************************** CLAW ***

  if (R1 == 1 && flag611 == 0)//Use flag611 for R1
  {
    digitalWrite(28, HIGH);
    flag611 = 1;
  }
  if (R1 == 0 && flag611 == 1)
  {
    digitalWrite(28, LOW);
    flag611 = 0;
  }

  // *************************************** ANGLE LOCOMOTION (5)***

  if (L2 == 1 && flag1 == 0)
  {
    setpoint = setpoint + 5;
    flag1 = 1;
  }
  if (L2 == 0)
  {
    flag1 = 0;
  }

  if (R2 == 1 && flag2 == 0)
  {
    setpoint = setpoint - 5;
    flag2 = 1;
  }
  if (R2 == 0)
  {
    flag2 = 0;
  }

 

  //  ============ (90 long button press)

  if (blue == 1 && flag3 == 0 && red == 0)
  { Kp = 6;// 6;//13;;8
    Ki =  0.004;//0.002;//0.003;//0.003
    Kd =  30;//37;//30
    setpoint = setpoint - 90;
    flag3 = 1;
  }
  if (blue  == 0 && red == 0)
  { Kp = 15;// 6;//13;;8
    Ki =  0.00;//0.002;//0.003;//0.003
    Kd =  30;//37;//30
    flag3 = 0;
  }

  if (red == 1 && flag4 == 0 && blue == 0)
  {
    Kp = 6;// 6;//13;;8
    Ki =  0.004;//0.002;//0.003;//0.003
    Kd =  30;//37;//30
    setpoint = setpoint + 90;
    flag4 = 1;
  }
  if (blue  == 0 && red == 0)
  { Kp = 15;// 6;//13;;8
    Ki =  0.00;//0.002;//0.003;//0.003
    Kd =  30;//37;//30
    flag4 = 0;
  }

  //************************************* RAMP MODE *******************

  if (yellow == 1 && flag711 == 0) {
    cnt3++;
    flag711 = 1;
  }
  Serial.print("cnt3");
  Serial.print(cnt3);

  if (yellow == 0 && flag711 == 1) {
    flag711 = 0;
  }




  //************************************* EQUATION OF LOCOMOTION****


  pre_error = error ;
  error = setpoint - yaw ;
  rate = error - pre_error ;
  incre = +error ;
  PRO = Kp * (error);
  DER = Kd * (rate);
  INT = Ki * (incre);
  output = PRO + DER + INT ;

  pid = constrain(output , -80, 80 );

  M1 =  -(r * (sin((pi / 2 - angle) )) * 1.15);
  M2 =  (r * (sin((7 * pi / 6 - angle) )) * 1.15);
  M3 =  (r * (sin((33 * pi / 18 - angle) )) * 1.15);
  M1 = map(M1, 0, 91, 0, 127);
  M2 = map(M2, 0, 91, 0, 127);
  M3 = map(M3, 0, 91, 0, 127);
  if (cnt3 % 2 == 0) {
    Serial.print("In 100 constraint");
    var_100 = 100;
  }

  if (cnt3 % 2 == 1) {

    Serial.print("In Ramp Mode ");
    M1 = map(M1, 0, 127 , 0, 255);
    M2 = map(M2, 0, 127, 0, 255);
    M3 = map(M3, 0, 127, 0, 255);
    Kp = 16;
    Ki = 0.001;
    Kd = 32;
    var_100 = 180;

  }

  M1 = constrain(M1, -var_100, var_100);
  M2 = constrain(M2, -var_100, var_100);
  M3 = constrain(M3, -var_100, var_100);

  m1 =  M1 + pid ;
  m2 = -M2 + pid  ;
  m3 = M3 - pid   ;

 
  motor1.setSpeed(m1);
  motor2.setSpeed(-m2);
  motor3.setSpeed(-m3);

  Serial.print("\t error=");
  Serial.print( error );


  Serial.print("\t pidM1=");
  Serial.print( m1 );
  Serial.print("\t pidM2=");
  Serial.print(m2 );
  Serial.print("\t pidM3=");
  Serial.print( m3 );

  Serial.print("\t kp=");
  Serial.print( Kp );
  Serial.print("\t ki=");
  Serial.print(Ki );
  Serial.print("\t kd=");
  Serial.println( Kd );

}
