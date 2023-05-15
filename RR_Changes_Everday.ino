//........//*************************LIBRARIES*****************//
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
#include<SoftwareSerial.h>
#include<SabertoothSimplified.h>

template<class T> inline Print& operator <<(Print &obj, T arg) {
  obj.print(arg);
  return obj;
}
template<>        inline Print& operator <<(Print &obj, float arg) {
  obj.print(arg, 4);
  return obj;
}

HardwareSerial& odrive_serial = Serial3 ;
USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);
JoystickEvents JoyEvents;
JoystickReportParser Joy(&JoyEvents);
ODriveArduino odrive(odrive_serial);

SoftwareSerial SWSerial(NOT_A_PIN, 4);
SabertoothSimplified ST(SWSerial);



// Configure the motor driver.
CytronMD motor1(PWM_DIR, 5, 32);
CytronMD motor2(PWM_DIR, 6, 34);
CytronMD motor3(PWM_DIR, 3, 36);
//************************PID GAINS******************//
float Kp ;// 6;//13;;8
float Ki ;//0.002;//0.003;//0.003
float Kd ;//37;//30

//*********************DECLARATIONS************//
int M1 , M2 , M3, m1, m2, m3;
unsigned long timer = 0;
unsigned long timer1 = 0;
unsigned long timer2 = 0;
float timeStep = 0.018998;
float yaw = 0;
int pre_error = 0;
int setpoint = 0 ;
int error ;
int Val;
float pi = 3.14;
float ang = 0;
int var_100 = 100;
float rate, incre, PRO, DER, INT, output, pid ;
int Base_M ;
int rot_speed = 30 ;
int rotation_counter, rot_counter, rot_flag = 0;

int Pick_M = 0, up_speed = 35, down_speed = -30 ;

int atuate_flag = 0;

int load_flag = 0;
int load_count = 0;

int slow_flag = 0;
int slow_opposite_flag = 0;
int slow_side_flag = 0;
int slow_opposite_side_flag = 0;
int slow_speed_m1 = 0;
int slow_speed_m2 = 0;
int slow_speed_m3 = 0;
int m1_speed = 35;
int m2_speed = 40;
int m3_speed = 40;


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
int red_flag = 0;
int red_cnt = 0;
int throw_flag = 0;


//_______________________________________


void setup()
{
  //  delay(2000);
  Serial.begin(115200);

  SWSerial.begin(9600);
  // Initialize MPU6050
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(3);

#if !defined(MIPSEL)
  while (!Serial);
#endif
  Serial.println("Start");

  while (Usb.Init() == -1)
    Serial.println("OSC did not start.");

  delay(200);
  pinMode(24, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(26, OUTPUT);
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

  delay(1000);

  //  ST.motor(1, 0);

  //________________________________________
}

void loop()
{
  ST.motor(1, Pick_M);

  timer = millis();
  Vector norm = mpu.readNormalizeGyro();
  yaw = yaw + norm.ZAxis * timeStep;
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
  rya = -rya;
  lxa = lxa - 128;
  lya = -lya + 127;
  lya = -lya;

  int hor = lxa - 30;
  int ver = lya - 30;

  if (lxa < 30 && lxa > -31)hor = 0;
  if (lya < 30 && lya > -31)ver = 0;
  if (hor < -30)hor += 60;
  if (ver < -30)ver += 60;
  hor = map(hor, -128, 128, -45, 45);
  ver = map(ver, -128, 128, -45, 45);

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
  g += hor;
  h += ver;

  int c = square(g);
  int d = square(h);
  float r = sqrt(c + d);

  float angle = atan2 (h, g);
  float theta = angle * (180 / pi);


  //************************ BLDC ***********************



  if ( start == 1 && flot_auto == 0) {
    var0 = -20;
    var1 = 20;
    odrive.SetVelocity(0, var0);
    odrive.SetVelocity(1, var1);
    cnt++;
    flot_auto = 1 ;
  }

  if ( start == 0 && flot_auto == 1 )
  {
    flot_auto = 0 ;
  }
  //  Serial.print("cnt ");
  //  Serial.print(cnt);


  if (cnt % 2 == 1 )              // **************************************************** In Start Condition
  {

    //   ********************************************************* BLDC START ***********************************
    //    ----------------------------------Increment of 0axis---
    if (gpad == 0 && flag11 == 0)
    {
      var0 -= 0.1;
      odrive.SetVelocity(0, var0);
      odrive.SetVelocity(1, -var0);
      flag11 = 1;
      Serial.print(" Increment of 0axis");
    }
    if (gpad == 4 && flag12 == 0)
    {
      var0 += 0.1;

      odrive.SetVelocity(0, var0);
      odrive.SetVelocity(1, -var0);
      flag12 = 1;
      Serial.print(" Decrement of 0axis");
    }

    //    ----------------------------------Increment of 1axis---
    if (gpad == 2 && flag13 == 0)
    {

      var0 += 1;

      odrive.SetVelocity(0, var0);
      odrive.SetVelocity(1, -var0);
      flag13 = 1;
      Serial.print(" Increment of 1axis");
    }
    //    ----------------------------------Decrement of 1axis---
    if (gpad == 6 && flag14 == 0)
    {

      var0 -= 1;

      odrive.SetVelocity(0, var0);
      odrive.SetVelocity(1, -var0);
      flag14 = 1;
      Serial.print(" Decrement of 1axis");
    }
    if (gpad == 9 && flag11 == 1)   flag11 = 0;
    if (gpad == 9 && flag12 == 1)   flag12 = 0;
    if (gpad == 9 && flag14 == 1)   flag14 = 0;
    if (gpad == 9 && flag13 == 1)   flag13 = 0;

    //********************************************************* BLDC OVER ************************************
    Serial.print(" In start");

    // *******************************************    Yaw Zero Condition ****************************************
    if (red == 1 && red_flag == 0)
    {
      red_cnt++;
      red_flag = 1;
    }
    if (red == 0 && red_flag == 1)
    {
      red_flag = 0;
    }
    Serial.print(" red_cnt: ");
    Serial.print(red_cnt);

    if (red_cnt % 2 == 1) {
      pid = 0;
      Serial.print("| pid is zero - Start |");
    }

    //******************************************** Grab R1 ******************************************
    if (R1 == 1 && load_flag == 0) {
      load_count++;
      load_flag = 1;
    }
    if (R1 == 0 && load_flag == 1) {
      load_flag = 0;
    }

    // *************************************** VERTICAL ANGLE ***

    if (back == 1 && flag511 == 0)//Use flag511 for L1
    {
      cnt1++;
      flag511 = 1;
    }
    if (back == 0 && flag511 == 1)
    {
      flag511 = 0;
    }
    Serial.print(" cnt1 ");
    Serial.print(cnt1);
    if (cnt1 % 2 == 0)   digitalWrite(24, HIGH);
    if (cnt1 % 2 == 1)  digitalWrite(24, LOW);

    // ***************************************** Motor Pick  *************
    if ( yellow == 1 || green == 1 ) {
      if (yellow == 1) {
        Pick_M = up_speed;
      }
      if (green == 1) {
        Pick_M = down_speed;
      }
    }
    else
    {
      Pick_M = 0;
    }
     //**************************************** Throwing ******************
     if(blue == 1 && throw_flag == 0){
      digitalWrite(26,HIGH);
      throw_flag = 1;
     }
     if(blue == 0 && throw_flag == 1){
      digitalWrite(26,LOW);
      throw_flag = 0;
     }



    // ------------------------------------------- Start over ----------------------------------------------------
  }
  //    ----------------------------------Decrement of 0axis---



  Serial.print("var0 = ");
  Serial.print(-var0);



  //    ----------------------------------Stop BLDC---

  if (cnt % 2 == 0)// Picking Mechaism
  { 
    var0 = 0 ;
    var1 = 0 ;
    odrive.SetVelocity(0, var0);
    odrive.SetVelocity(1, -var0);
    //************* Grab R1
    if (R1 == 1 && load_flag == 0) {
      load_count++;
      load_flag = 1;
    }
    if (R1 == 0 && load_flag == 1) {
      load_flag = 0;
    }
    //************** Grab R1
    //************* Motor Pick
    if ( yellow == 1 || green == 1 ) {
      if (yellow == 1) {
        Pick_M = up_speed;
      }
      if (green == 1) {
        Pick_M = down_speed;
      }
    }
    else
    {
      Pick_M = 0;
    }
    //***************** Motor Pick
    //***************** Slow Motion

    if (gpad == 0 && slow_flag == 0) {
      slow_speed_m2 = -m2_speed;
      slow_speed_m3 = -m3_speed;
      slow_flag = 1;
    }

    if (gpad == 4 && slow_opposite_flag == 0) {
      slow_speed_m2 = m2_speed;
      slow_speed_m3 = m3_speed;
      slow_opposite_flag = 1;
    }

    if (gpad == 2 && slow_side_flag == 0) {
      slow_speed_m1 = -m1_speed;
      slow_speed_m2 = m2_speed;
      slow_speed_m3 = -m3_speed;
      slow_side_flag = 1;
    }
    if (gpad == 6 && slow_opposite_side_flag == 0) {
      slow_speed_m1 = m1_speed;
      slow_speed_m2 = -m2_speed;
      slow_speed_m3 = m3_speed;
      slow_opposite_side_flag = 1;
    }

    if (gpad == 9 && (slow_flag = 1 || slow_opposite_flag == 1 || slow_side_flag == 1 || slow_opposite_side_flag == 1))
    {
      slow_speed_m1 = 0;
      slow_speed_m2 = 0;
      slow_speed_m3 = 0;
      slow_flag = 0;
      slow_opposite_flag = 0;
      slow_side_flag = 0;
      slow_opposite_side_flag = 0;
    }

    //******************** Slow Motion

    
  //************************************* RAMP MODE *******************

  if (back == 1 && flag711 == 0) {
    cnt3++;
    flag711 = 1;
  }
  Serial.print(" cnt3");
  Serial.print(cnt3);

  if (back == 0 && flag711 == 1) {
    flag711 = 0;
  }
  }





  


  if ( L2 == 1 || R2 == 1 )
  {
    if ( L2 == 1 )  Base_M = rot_speed ;

    if ( R2 == 1 )  Base_M = -rot_speed ;

  }
  else {
    Base_M = 0;
  }

  if (load_count % 2 == 0 && load_count != 0) {
    digitalWrite(22, HIGH);
  }
  if (load_count % 2 != 0) {
    digitalWrite(22, LOW);
  }

  // *************************************** ANGLE LOCOMOTION (90)***
  //  ============ (90 long button press)

  if (blue == 1 && flag3 == 0 && cnt % 2 != 1)
  { Kp = 16;// 6;//13;;8
    Ki =  0.004;//0.002;//0.003;//0.003
    Kd =  50;//37;//30
    setpoint = setpoint + 84;
    flag3 = 1;
    Serial.print("in 1");
  }
  if (blue  == 0 &&  flag3 == 1) {
    //  { Kp = 16;// 6;//13;;8
    //    Ki =  0.00;//0.002;//0.003;//0.003
    //    Kd =  60;//37;//30
    flag3 = 0;
    Serial.print("In 2");

  }

  if (red == 1 && flag4 == 0 && cnt % 2 != 1 )
  {
    Kp = 16;// 6;//13;;8
    Ki =  0.004;//0.002;//0.003;//0.003
    Kd =  50;//37;//30
    setpoint = setpoint - 84;
    flag4 = 1;
    Serial.print("In 3");
  }
  if (red == 0 && flag4 == 1) {
    //  { Kp = 15;// 6;//13;;8
    //    Ki =  0.00;//0.002;//0.003;//0.003
    //    Kd =  60;//37;//30
    flag4 = 0;
    Serial.print("In 4");
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

  if (cnt % 2 == 1 && red_cnt % 2 == 0 )
    pid = constrain(output , -100, 100);

  if (cnt % 2 == 0 ) pid = constrain(output , -100, 100);





  M1 =  -(r * (sin((pi / 2 - angle) )) * 1.15);
  M2 =  (r * (sin((7 * pi / 6 - angle) )) * 1.15);
  M3 =  (r * (sin((33 * pi / 18 - angle) )) * 1.15);
  M1 = map(M1, 0, 91, 0, 127);
  M2 = map(M2, 0, 91, 0, 127);
  M3 = map(M3, 0, 91, 0, 127);
  if (cnt3 % 2 == 0 && hor == 0 && ver == 0) {
    Serial.print("| In 230 constraint |");
    M1 = map(M1, 0, 91, 0, 230);
    M2 = map(M2, 0, 91, 0, 230);
    M3 = map(M3, 0, 91, 0, 230);
    var_100 = 230;
  }

  if (cnt3 % 2 == 1  ) {

    Serial.print(" In Ramp Mode ");
    M1 = map(M1, 0, 127 , 0, 255);
    M2 = map(M2, 0, 127, 0, 255);
    M3 = map(M3, 0, 127, 0, 255);
    var_100 = 160;

  }

  if ( rot_flag == 0 && blue != 1 && red != 1 )
  {

    Kp = 24 ;
    Ki = 0.000001 ;
    Kd = 35  ;

  }

  if (Base_M != 0 )
  {

    Kp = 0   ;
    Ki = 0   ;
    Kd = 0 ;
    rot_flag = 1 ;
    rotation_counter = 0 ;

  }

  if (Base_M == 0 && rot_flag == 1 )
  {
    rotation_counter++ ;

  }

  if ( rotation_counter == 6 )
  {
    setpoint =  yaw ;
  }

  if ( rotation_counter == 7 )
  {
    rot_flag = 0 ;

  }
  Serial.print("rotation_counter: ");
  Serial.print(rotation_counter);

  M1 = constrain(M1, -var_100, var_100);
  M2 = constrain(M2, -var_100, var_100);
  M3 = constrain(M3, -var_100, var_100);


  m1 =  M1 + pid  + Base_M + slow_speed_m1;//  m1 =  M1 + pid  + Base_M + slow_speed_m1 ;
  m2 = -M2 + pid  + Base_M + slow_speed_m2; //m2 = -M2 + pid  + Base_M + slow_speed_m2;
  m3 = M3 - pid  - Base_M + slow_speed_m3; //m3 = M3 - pid  - Base_M + slow_speed_m3;



  motor1.setSpeed(-m1);
  motor2.setSpeed(-m2);
  motor3.setSpeed(m3);

  Serial.print("\t lya=");
  Serial.print( lya );
  Serial.print(" lxa ");
  Serial.print(lxa);

  Serial.print(" hor =");
  Serial.print( hor);
  Serial.print(" ver ");
  Serial.print(ver);

  Serial.print("\t pid=");
  Serial.print( pid );

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
